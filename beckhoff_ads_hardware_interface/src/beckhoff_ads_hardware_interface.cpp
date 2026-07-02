// Copyright (c) 2025, b-robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.
//
// Author: Nikola Banovic
// Contributor: Hajar Bartakh

#include <limits>
#include <vector>
#include <cstdint>
#include <cstring>   // std::memcpy
#include <algorithm> // std::transform
#include <atomic>
#include <chrono>
#include <cmath>     // std::isnan
#include <deque>
#include <memory>
#include <mutex>
#include <thread>    // std::this_thread::yield

#include "beckhoff_ads_hardware_interface/beckhoff_ads_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace beckhoff_ads_hardware_interface
{
    namespace
    {
        // Decodes one PLC element at src into a double. Shared by the reader thread and the
        // notification callback so the two paths never diverge. Returns NaN for unsupported
        // types (UNKNOWN/STRING are filtered out at configure time).
        double decode_plc_element(PLCType plc_type, const uint8_t *src)
        {
            switch (plc_type)
            {
            case PLCType::LREAL:
            {
                double v;
                std::memcpy(&v, src, sizeof(v));
                return v;
            }
            case PLCType::REAL:
            {
                float v;
                std::memcpy(&v, src, sizeof(v));
                return static_cast<double>(v);
            }
            case PLCType::BOOL:
            {
                uint8_t v;
                std::memcpy(&v, src, sizeof(v));
                return (v != 0) ? 1.0 : 0.0;
            }
            case PLCType::SINT:
            {
                int8_t v;
                std::memcpy(&v, src, sizeof(v));
                return static_cast<double>(v);
            }
            case PLCType::USINT:
            case PLCType::BYTE:
            {
                uint8_t v;
                std::memcpy(&v, src, sizeof(v));
                return static_cast<double>(v);
            }
            case PLCType::INT:
            {
                int16_t v;
                std::memcpy(&v, src, sizeof(v));
                return static_cast<double>(v);
            }
            case PLCType::UINT:
            {
                uint16_t v;
                std::memcpy(&v, src, sizeof(v));
                return static_cast<double>(v);
            }
            case PLCType::DINT:
            {
                int32_t v;
                std::memcpy(&v, src, sizeof(v));
                return static_cast<double>(v);
            }
            case PLCType::UDINT:
            {
                uint32_t v;
                std::memcpy(&v, src, sizeof(v));
                return static_cast<double>(v);
            }
            default:
                return std::numeric_limits<double>::quiet_NaN();
            }
        }

        // One destination per ROS state interface that targets a given PLC symbol.
        struct ElementTarget
        {
            size_t sample_byte_offset;   // index * elem_byte_size within the symbol's sample
            std::atomic<double> *dest;   // points into the owning context's stable storage
        };

        // Per-symbol decode context. Pooled and reused, never destroyed (see registry below).
        struct NotificationContext
        {
            PLCType plc_type{PLCType::UNKNOWN};
            size_t elem_byte_size{0};
            uint32_t expected_sample_size{0};
            std::deque<std::atomic<double>> values;   // one slot per interface; deque keeps addresses stable
            std::vector<ElementTarget> targets;
            std::atomic<bool> ready{false};           // gates decoding; cleared before a context is reused
            std::atomic<int> active{0};               // in-flight callbacks; teardown drains this to zero
            std::atomic<long long> last_update_steady_ns{0};
        };

        // Process-static context registry, indexed by the uint32_t hUser handed to ADS.
        // Contexts are never destroyed: DeleteNotification does not join the dispatcher, so a
        // callback can fire after its handle is deleted. Freed slots are reused instead, so the
        // pool stays bounded by the peak count of live symbols.
        std::deque<NotificationContext> g_context_pool;   // stable addresses; never shrinks
        std::vector<uint32_t> g_free_indices;
        std::mutex g_registry_mutex;

        // Immutable snapshot the callback reads with one atomic load, so it takes no mutex.
        // Republished on pool growth; old snapshots stay alive for in-flight callbacks.
        using ContextSnapshot = std::vector<NotificationContext *>;
        std::atomic<const ContextSnapshot *> g_context_snapshot{nullptr};
        std::vector<std::unique_ptr<const ContextSnapshot>> g_snapshot_keepalive; // guarded by g_registry_mutex

        long long steady_now_ns()
        {
            return std::chrono::duration_cast<std::chrono::nanoseconds>(
                       std::chrono::steady_clock::now().time_since_epoch())
                .count();
        }

        // Rebuilds and publishes the index->context* snapshot. Caller must hold g_registry_mutex.
        void republish_context_snapshot_locked()
        {
            auto snapshot = std::make_unique<ContextSnapshot>();
            snapshot->reserve(g_context_pool.size());
            for (auto &ctx : g_context_pool)
            {
                snapshot->push_back(&ctx);
            }
            const ContextSnapshot *published = snapshot.get();
            g_snapshot_keepalive.push_back(std::move(snapshot));
            g_context_snapshot.store(published, std::memory_order_release);
        }

        // Reserves a context slot, reusing a freed one when available. Returns its hUser index.
        uint32_t acquire_context_slot()
        {
            std::lock_guard<std::mutex> lock(g_registry_mutex);
            uint32_t index;
            if (!g_free_indices.empty())
            {
                index = g_free_indices.back();
                g_free_indices.pop_back();
                // Its context* is already in the snapshot; addresses are stable.
            }
            else
            {
                index = static_cast<uint32_t>(g_context_pool.size());
                g_context_pool.emplace_back();
                republish_context_snapshot_locked();
            }
            return index;
        }

        // Returns a stable pointer into the never-relocating pool.
        NotificationContext *context_at(uint32_t index)
        {
            std::lock_guard<std::mutex> lock(g_registry_mutex);
            return &g_context_pool[index];
        }

        // Returns a slot to the free-list once its notification has been deleted. Drains any
        // in-flight callback first so its fields can later be rewritten for reuse.
        void release_context_slot(uint32_t index)
        {
            NotificationContext *ctx = context_at(index);
            // Notification already deleted, so active only falls. Clear ready first so a late
            // callback skips decoding.
            ctx->ready.store(false, std::memory_order_release);
            while (ctx->active.load(std::memory_order_acquire) != 0)
            {
                std::this_thread::yield();
            }
            std::lock_guard<std::mutex> lock(g_registry_mutex);
            g_free_indices.push_back(index);
        }

        // Runs on the ADS dispatcher thread: stay fast, no ADS calls, no ros2_control handles,
        // no mutex. Decodes the sample into the lock-free cache.
        void notification_callback(const AmsAddr * /*addr*/,
                                   const AdsNotificationHeader *header,
                                   uint32_t hUser)
        {
            const ContextSnapshot *snapshot = g_context_snapshot.load(std::memory_order_acquire);
            if (snapshot == nullptr || hUser >= snapshot->size())
            {
                return;
            }
            NotificationContext *ctx = (*snapshot)[hUser];
            if (ctx == nullptr)
            {
                return;
            }

            // Mark busy so a concurrent teardown drains before reusing this context.
            ctx->active.fetch_add(1, std::memory_order_acquire);
            if (ctx->ready.load(std::memory_order_acquire) &&
                header->cbSampleSize >= ctx->expected_sample_size)
            {
                const uint8_t *data = reinterpret_cast<const uint8_t *>(header + 1);
                for (const auto &target : ctx->targets)
                {
                    target.dest->store(decode_plc_element(ctx->plc_type, data + target.sample_byte_offset),
                                       std::memory_order_release);
                }
                ctx->last_update_steady_ns.store(steady_now_ns(), std::memory_order_relaxed);
            }
            ctx->active.fetch_sub(1, std::memory_order_release);
        }
    } // namespace

    BeckhoffADSHardwareInterface::~BeckhoffADSHardwareInterface()
    {
        // Backstop if no lifecycle shutdown ran: a joinable std::thread would call std::terminate.
        // ads_device_ outlives the threads (declared first), so an in-flight ADS call completes.
        stop_io_threads();
    }

    hardware_interface::CallbackReturn BeckhoffADSHardwareInterface::on_init(
        const hardware_interface::HardwareComponentInterfaceParams &params)
    {
        if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }

        logging_throttle_clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);

        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn BeckhoffADSHardwareInterface::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // Tear down any notifications from a previous configure cycle while their owning
        // AdsDevice is still alive. configure_ads_device() replaces ads_device_ below, which
        // would otherwise leave the notification handle deleters pointing at a freed device.
        teardown_notifications();
        // Release prior-cycle symbol handles before the device is replaced (same hazard).
        release_ads_handles();

        // Configure ADS Client Device
        if (!configure_ads_device())
        {
            RCLCPP_FATAL(getLogger(), "Failed to configure ADS device from URDF parameters.");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Select the read strategy. Default is the synchronous SUM read on a background thread.
        // Set the <hardware> parameter read_mode="notification" to read via PLC-pushed ADS device
        // notifications instead. The write path always uses the SUM write.
        read_via_notifications_ = false;
        {
            auto it = info_.hardware_parameters.find("read_mode");
            if (it != info_.hardware_parameters.end())
            {
                std::string mode = it->second;
                std::transform(mode.begin(), mode.end(), mode.begin(), ::tolower);
                read_via_notifications_ = (mode == "notification" || mode == "notifications" ||
                                           mode == "notify");
            }
            RCLCPP_INFO(getLogger(), "Read strategy: %s",
                        read_via_notifications_ ? "ADS device notifications (push)"
                                                : "synchronous SUM read (polling)");
        }

        notif_staleness_ns_ = 0;
        read_poll_period_ns_ = 0;
        try
        {
            auto timeout_it = info_.hardware_parameters.find("notify_timeout_ms");
            if (timeout_it != info_.hardware_parameters.end())
            {
                const double ms = std::stod(timeout_it->second);
                if (std::isfinite(ms) && ms > 0.0)
                {
                    notif_staleness_ns_ = static_cast<long long>(ms * 1e6);
                }
            }
            auto period_it = info_.hardware_parameters.find("read_poll_period_ms");
            if (period_it != info_.hardware_parameters.end())
            {
                const double ms = std::stod(period_it->second);
                if (std::isfinite(ms) && ms > 0.0)
                {
                    read_poll_period_ns_ = static_cast<long long>(ms * 1e6);
                }
            }
        }
        catch (const std::exception &ex)
        {
            RCLCPP_WARN(getLogger(), "Invalid notify_timeout_ms/read_poll_period_ms: %s. Using defaults (disabled).", ex.what());
        }

        // Fill the ADSDataLayout vectors for read and write operations
        ads_read_layout_configure();
        ads_write_layout_configure();

        // Request handles for symbolic PLC variable names. In notification mode the read
        // symbols are resolved internally by AdsNotification, so explicit read handles are
        // only needed when polling. Write handles are always required.
        RCLCPP_INFO(getLogger(), "Fetching ADS handles for configured PLC variables...");
        if (!read_via_notifications_)
        {
            for (auto &layout : ads_item_layouts_read_)
            {
                try
                {
                    layout.ads_handle_owner.emplace(ads_device_->GetHandle(layout.plc_name_symbolic));
                    layout.ads_handle = **layout.ads_handle_owner;
                }
                catch (const std::exception &ex)
                {
                    RCLCPP_ERROR(getLogger(), "\tADS Exception getting handle for '%s': %s. Read operations for this variable will fail.", layout.plc_name_symbolic.c_str(), ex.what());
                }
            }
        }
        for (auto &layout : ads_item_layouts_write_)
        {
            try
            {
                layout.ads_handle_owner.emplace(ads_device_->GetHandle(layout.plc_name_symbolic));
                layout.ads_handle = **layout.ads_handle_owner;
            }
            catch (const std::exception &ex)
            {
                RCLCPP_ERROR(getLogger(), "\tADS Exception getting handle for '%s': %s. Write operations for this variable will fail.", layout.plc_name_symbolic.c_str(), ex.what());
            }
        }
        RCLCPP_INFO(getLogger(), "\tHandles acquired");

        // Pre-pack the read path: SUM-read buffers when polling, device notifications otherwise.
        if (read_via_notifications_)
        {
            if (!setup_notifications())
            {
                RCLCPP_FATAL(getLogger(), "\tFailed to register ADS device notifications.");
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        else
        {
            if (!build_sum_read_buffers())
            {
                RCLCPP_FATAL(getLogger(), "\tFailed to build ADS sum read buffer.");
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        if (!build_sum_write_buffers())
        {
            RCLCPP_FATAL(getLogger(), "\tFailed to build ADS sum write buffer.");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Link command interfaces to their corresponding state interfaces
        RCLCPP_INFO(getLogger(), "Linking command interfaces to state interfaces...");
        for (auto &command_layout : ads_item_layouts_write_)
        {
            for (const auto &state_layout : ads_item_layouts_read_)
            {
                if (command_layout.plc_name_symbolic == state_layout.plc_name_symbolic)
                {
                    for (size_t k = 0; k < command_layout.num_elements; ++k)
                    {
                        // The pair is made of (command_interface_name, corresponding_state_interface_name)
                        auto pair = std::make_pair(command_layout.ros2_interfaces_.find(k)->second, state_layout.ros2_interfaces_.find(k)->second);
                        command_layout.state_command_interfaces_map_.emplace(pair);
                    }
                }
            }
        }

        return CallbackReturn::SUCCESS;
    }

    bool BeckhoffADSHardwareInterface::build_sum_read_buffers()
    {
        num_items_read_ = ads_item_layouts_read_.size();
        if (num_items_read_ == 0)
        {
            RCLCPP_INFO(getLogger(), "No items to configure for ADS Sum READ.");
            return true;
        }
        RCLCPP_INFO(getLogger(), "Building ADS sum READ buffer...");

        size_t total_error_block_size = num_items_read_ * sizeof(uint32_t);
        size_t total_data_block_size = 0;
        for (const auto &layout : ads_item_layouts_read_)
        {
            total_data_block_size += layout.plc_element_byte_size * layout.num_elements;
        }

        ads_buffer_sum_read_response_.resize(total_error_block_size + total_data_block_size);
        ads_buffer_sum_read_request_.clear();

        size_t current_data_offset = 0;
        size_t current_error_offset = 0;

        for (auto &layout : ads_item_layouts_read_)
        {
            layout.offset_in_read_response_data = total_error_block_size + current_data_offset;
            layout.offset_in_read_response_error = current_error_offset;

            ADS_ITEM_REQ_HEADER header;
            header.indexGroup = ADSIGRP_SYM_VALBYHND;
            header.indexOffset = layout.ads_handle;
            header.NumBytesData = layout.plc_element_byte_size * layout.num_elements;
            const uint8_t *ptr = reinterpret_cast<const uint8_t *>(&header);
            ads_buffer_sum_read_request_.insert(ads_buffer_sum_read_request_.end(), ptr, ptr + sizeof(ADS_ITEM_REQ_HEADER));

            // For interfaces targeting the same PLC symbol
            for (const auto &[index, interface_name] : layout.ros2_interfaces_)
            {
                // Fill the read instruction vector
                ReadInstruction read_instruction;
                read_instruction.read_buffer_offset_error_code = layout.offset_in_read_response_error;
                read_instruction.read_buffer_offset_data = layout.offset_in_read_response_data + index * layout.plc_element_byte_size;
                read_instruction.plc_type = layout.plc_type;
                read_instruction.state_interface_name = interface_name;

                // The state interfaces' names are ordered by ascending indexes of the PLC array thanks to layout.ros2_interfaces_ being a map
                ads_read_instructions_.push_back(read_instruction);
            }

            current_data_offset += header.NumBytesData;
            current_error_offset += sizeof(uint32_t);
        }
        // One cache slot per read instruction for the reader thread. Default to NaN so read()
        // reports "no sample yet" until the first SUM read completes. A deque keeps each slot's
        // address stable; std::atomic<double> is neither copyable nor movable.
        polling_read_cache_.clear();
        for (size_t i = 0; i < ads_read_instructions_.size(); ++i)
        {
            polling_read_cache_.emplace_back(std::numeric_limits<double>::quiet_NaN());
        }

        RCLCPP_INFO(getLogger(), "ADS Sum READ configured for %zu items. Request: %zu bytes, Response: %zu bytes.",
                    num_items_read_, ads_buffer_sum_read_request_.size(), ads_buffer_sum_read_response_.size());
        return true;
    }

    bool BeckhoffADSHardwareInterface::build_sum_write_buffers()
    {
        RCLCPP_INFO(getLogger(), "Building ADS sum WRITE buffer...");
        num_items_write_ = ads_item_layouts_write_.size();
        if (num_items_write_ == 0)
        {
            RCLCPP_INFO(getLogger(), "No items to configure for ADS Sum WRITE.");
            return true;
        }

        size_t total_data_size = 0;
        size_t total_header_size = num_items_write_ * sizeof(ADS_ITEM_REQ_HEADER);
        for (const auto &layout : ads_item_layouts_write_)
        {
            total_data_size += layout.plc_element_byte_size * layout.num_elements;
        }

        ads_buffer_sum_write_request_.resize(total_header_size + total_data_size);
        ads_buffer_sum_write_response_.resize(num_items_write_ * sizeof(uint32_t));

        auto *header_block_ptr = reinterpret_cast<ADS_ITEM_REQ_HEADER *>(ads_buffer_sum_write_request_.data());
        size_t current_data_offset = 0;
        size_t i = 0; // Index for the header block pointer

        for (auto &layout : ads_item_layouts_write_)
        {
            header_block_ptr[i].indexGroup = ADSIGRP_SYM_VALBYHND;
            header_block_ptr[i].indexOffset = layout.ads_handle;
            header_block_ptr[i].NumBytesData = layout.plc_element_byte_size * layout.num_elements;

            layout.offset_in_write_request_data = total_header_size + current_data_offset;

            // For interfaces targeting the same PLC symbol
            for (const auto &[index, interface_name] : layout.ros2_interfaces_)
            {
                // Fill the write instruction vector
                WriteInstruction write_instruction;
                write_instruction.write_buffer_offset_data = layout.offset_in_write_request_data + index * layout.plc_element_byte_size;
                write_instruction.plc_type = layout.plc_type;
                write_instruction.command_interface_name = interface_name;
                write_instruction.fallback_state_interface_name = "";

                // There exists a state interface for the same PLC symbol
                if (!layout.state_command_interfaces_map_.empty())
                {
                    write_instruction.fallback_state_interface_name = layout.state_command_interfaces_map_.find(interface_name)->second;
                }

                // The command interfaces' names are ordered by ascending indexes of the PLC array thanks to layout.ros2_interfaces_ being a map
                ads_write_instructions_.push_back(write_instruction);
            }

            current_data_offset += header_block_ptr[i].NumBytesData;
            i++;
        }

        RCLCPP_INFO(getLogger(), "ADS Sum WRITE configured for %zu items. Request: %zu bytes, Response: %zu bytes.",
                    num_items_write_, ads_buffer_sum_write_request_.size(), ads_buffer_sum_write_response_.size());
        return true;
    }

    bool BeckhoffADSHardwareInterface::setup_notifications()
    {
        // Release any existing notifications and their slots before acquiring fresh ones.
        teardown_notifications();

        if (ads_item_layouts_read_.empty())
        {
            RCLCPP_INFO(getLogger(), "No items to configure for ADS notifications.");
            return true;
        }
        RCLCPP_INFO(getLogger(), "Registering ADS device notifications for %zu read symbol(s)...",
                    ads_item_layouts_read_.size());

        read_notifications_.reserve(ads_item_layouts_read_.size());
        notif_slot_indices_.reserve(ads_item_layouts_read_.size());

        for (auto &layout : ads_item_layouts_read_)
        {
            const uint32_t sample_size =
                static_cast<uint32_t>(layout.plc_element_byte_size * layout.num_elements);
            const bool cyclic = (layout.notify_trans_mode == ADSTRANS_SERVERCYCLE);

            // Reserve/reuse a registry slot. It is quiescent, so rewriting its fields is safe.
            const uint32_t h_user = acquire_context_slot();
            NotificationContext *ctx = context_at(h_user);
            notif_slot_indices_.push_back(h_user);

            ctx->plc_type = layout.plc_type;
            ctx->elem_byte_size = layout.plc_element_byte_size;
            ctx->expected_sample_size = sample_size;
            ctx->last_update_steady_ns.store(0, std::memory_order_relaxed);
            ctx->values.clear();
            ctx->targets.clear();

            // One cache slot per interface targeting this symbol. ros2_interfaces_ is keyed by
            // the URDF array index (not a dense 0..n-1), so the sample offset must be derived
            // from that index, mirroring how the SUM-read path computes read_buffer_offset_data.
            for (const auto &[index, interface_name] : layout.ros2_interfaces_)
            {
                ctx->values.emplace_back(std::numeric_limits<double>::quiet_NaN());
                std::atomic<double> *dest = &ctx->values.back();

                ElementTarget target;
                target.sample_byte_offset = index * layout.plc_element_byte_size;
                target.dest = dest;
                ctx->targets.push_back(target);

                notif_read_targets_.push_back(
                    NotifReadTarget{interface_name, dest, &ctx->last_update_steady_ns, cyclic});
            }

            // Publish the context BEFORE registering: the AdsNotification constructor can fire
            // the callback immediately (e.g. the initial on-change sample) on another thread.
            ctx->ready.store(true, std::memory_order_release);

            AdsNotificationAttrib attrib{};
            attrib.cbLength = sample_size;
            attrib.nTransMode = layout.notify_trans_mode;
            attrib.nMaxDelay = layout.notify_max_delay_100ns;
            attrib.nCycleTime = layout.notify_cycle_100ns;

            try
            {
                read_notifications_.emplace_back(*ads_device_, layout.plc_name_symbolic,
                                                 attrib, &notification_callback, h_user);
                RCLCPP_INFO(getLogger(), "\t%s [%s, cycle=%.1f ms, maxDelay=%.1f ms] -> %zu interface(s)",
                            layout.plc_name_symbolic.c_str(),
                            cyclic ? "cyclic" : "on-change",
                            layout.notify_cycle_100ns / 10000.0,
                            layout.notify_max_delay_100ns / 10000.0,
                            layout.ros2_interfaces_.size());
            }
            catch (const std::exception &ex)
            {
                ctx->ready.store(false, std::memory_order_release);
                RCLCPP_ERROR(getLogger(), "\tFailed to register notification for '%s': %s",
                             layout.plc_name_symbolic.c_str(), ex.what());
                read_notifications_.clear();
                release_notification_slots();
                notif_read_targets_.clear();
                return false;
            }
        }
        RCLCPP_INFO(getLogger(), "\tADS notifications registered.");
        return true;
    }

    void BeckhoffADSHardwareInterface::teardown_notifications()
    {
        // Delete notifications first (needs ads_device_ alive, stops new callbacks), then drain slots.
        read_notifications_.clear();
        release_notification_slots();
        notif_read_targets_.clear();
    }

    void BeckhoffADSHardwareInterface::release_notification_slots()
    {
        for (uint32_t index : notif_slot_indices_)
        {
            release_context_slot(index);
        }
        notif_slot_indices_.clear();
    }

    void BeckhoffADSHardwareInterface::ads_read_layout_configure()
    {
        // Count all state interfaces to pre-allocate memory once and avoid reallocations.
        size_t num_state_interfaces = gpio_state_interfaces_.size() + joint_state_interfaces_.size() + sensor_state_interfaces_.size();

        // Reserve worst-case scenario for layouts (each interface targets a different PLC symbol)
        ads_item_layouts_read_.clear();
        ads_item_layouts_read_.reserve(num_state_interfaces);

        // Keep track of multiple interfaces targeting the same PLC symbol of type ARRAY[x], but different index
        std::map<std::string, bool> processed_plc_symbols;

        auto init_ads_read_layout =
            [&](const auto &type_state_interfaces_)
        {
            for (const auto &[name, descr] : type_state_interfaces_)
            {
                std::string plc_symbol;
                std::string plc_type_str;
                size_t num_elements = 1;
                size_t plc_index = 0;
                // Notification tuning (read path only). Defaults: deliver each change as soon
                // as the PLC detects it, checking at most every 10 ms.
                std::string notify_mode_str = "onchange";
                double notify_cycle_ms = 10.0;
                double notify_max_delay_ms = 0.0;
                try
                {
                    plc_symbol = descr.interface_info.parameters.at("PLC_symbol");
                    plc_type_str = descr.interface_info.parameters.at("PLC_type");
                    if (descr.interface_info.parameters.count("n_elements"))
                    {
                        num_elements = std::stoul(descr.interface_info.parameters.at("n_elements"));
                    }
                    if (descr.interface_info.parameters.count("index"))
                    {
                        plc_index = std::stoul(descr.interface_info.parameters.at("index"));
                    }
                    if (descr.interface_info.parameters.count("notify_mode"))
                    {
                        notify_mode_str = descr.interface_info.parameters.at("notify_mode");
                    }
                    if (descr.interface_info.parameters.count("notify_cycle_ms"))
                    {
                        notify_cycle_ms = std::stod(descr.interface_info.parameters.at("notify_cycle_ms"));
                    }
                    if (descr.interface_info.parameters.count("notify_max_delay_ms"))
                    {
                        notify_max_delay_ms = std::stod(descr.interface_info.parameters.at("notify_max_delay_ms"));
                    }
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(getLogger(), "Error parsing PLC parameters for state interface '%s': %s. Check URDF.",
                                 name.c_str(), e.what());
                    continue;
                }

                // If this is the first time we see this symbol, create the layout
                if (processed_plc_symbols.find(plc_symbol) == processed_plc_symbols.end())
                {
                    ADSDataLayout layout;
                    layout.plc_name_symbolic = plc_symbol;
                    layout.num_elements = num_elements;
                    layout.plc_type = strToPlcType(plc_type_str);
                    layout.ros2_interfaces_.emplace(std::make_pair(plc_index, name));

                    // Per-symbol notification config comes from the first interface that names
                    // the symbol (one notification covers the whole symbol/array). ADS cycle and
                    // max-delay fields are in 100 ns ticks, so convert from the URDF milliseconds.
                    std::string notify_mode_upper = notify_mode_str;
                    std::transform(notify_mode_upper.begin(), notify_mode_upper.end(),
                                   notify_mode_upper.begin(), ::toupper);
                    layout.notify_trans_mode =
                        (notify_mode_upper == "CYCLIC" || notify_mode_upper == "CYCLE")
                            ? ADSTRANS_SERVERCYCLE
                            : ADSTRANS_SERVERONCHA;
                    // ms -> 100 ns ticks, clamped so a bad value cannot wrap uint32_t.
                    auto ms_to_ticks = [this, &name](double ms) -> uint32_t
                    {
                        const double ticks = ms * 10000.0;
                        if (!std::isfinite(ticks) || ticks < 0.0 ||
                            ticks > static_cast<double>(std::numeric_limits<uint32_t>::max()))
                        {
                            RCLCPP_WARN(getLogger(),
                                        "notify timing for '%s' out of range (%.3f ms); clamping.",
                                        name.c_str(), ms);
                            return (std::isfinite(ticks) && ticks > 0.0)
                                       ? std::numeric_limits<uint32_t>::max()
                                       : 0u;
                        }
                        return static_cast<uint32_t>(ticks);
                    };
                    layout.notify_cycle_100ns = ms_to_ticks(notify_cycle_ms);
                    layout.notify_max_delay_100ns = ms_to_ticks(notify_max_delay_ms);

                    if (layout.plc_type == PLCType::UNKNOWN || layout.plc_type == PLCType::STRING)
                    {
                        RCLCPP_ERROR(getLogger(), "Skipping state variable '%s' due to UNSUPPORTED or UNKNOWN PLC_type '%s'.", layout.plc_name_symbolic.c_str(), plc_type_str.c_str());
                    }
                    else
                    {
                        layout.plc_element_byte_size = plcTypeByteSize(layout.plc_type);
                        ads_item_layouts_read_.push_back(std::move(layout));
                        processed_plc_symbols[plc_symbol] = true;
                    }
                }
                // The symbol already exists
                else
                {
                    // Find the ADS Data Layout object of the corresponding PLC symbol
                    auto it = std::find_if(ads_item_layouts_read_.begin(), ads_item_layouts_read_.end(),
                                           [&plc_symbol](const ADSDataLayout &layout)
                                           { return layout.plc_name_symbolic == plc_symbol; });

                    // Add the interface name the layout
                    (*it).ros2_interfaces_.emplace(std::make_pair(plc_index, name));
                }
            }
        };

        init_ads_read_layout(joint_state_interfaces_);
        init_ads_read_layout(gpio_state_interfaces_);
        init_ads_read_layout(sensor_state_interfaces_);
    }

    void BeckhoffADSHardwareInterface::ads_write_layout_configure()
    {
        // Count all command interfaces to pre-allocate memory once and avoid reallocations.
        size_t num_command_interfaces = joint_command_interfaces_.size() + gpio_command_interfaces_.size();

        // Reserve worst-case scenario for layouts (each interface targets a different PLC symbol)
        ads_item_layouts_write_.clear();
        ads_item_layouts_write_.reserve(num_command_interfaces);

        // Keep track of multiple interfaces targeting the same PLC symbol of type ARRAY[x], but different index
        std::map<std::string, bool> processed_plc_symbols;

        auto init_ads_write_layout =
            [&](const auto &type_command_interfaces_)
        {
            for (const auto &[name, descr] : type_command_interfaces_)
            {
                [[maybe_unused]] double initial_value = std::numeric_limits<double>::quiet_NaN();

                if (descr.interface_info.parameters.count("initial_value"))
                {
                    try
                    {
                        initial_value = std::stod(descr.interface_info.parameters.at("initial_value"));
                    }
                    catch (const std::exception &ex)
                    { // Catch conversion errors
                        RCLCPP_WARN(
                            getLogger(),
                            "Invalid 'initial_value' ('%s') for command interface '%s'. Using NaN. Error: %s",
                            descr.interface_info.parameters.at("initial_value").c_str(),
                            name.c_str(),
                            ex.what());
                    }
                }

                std::string plc_symbol;
                std::string plc_type_str;
                size_t num_elements = 1;
                size_t plc_index = 0;
                try
                {
                    plc_symbol = descr.interface_info.parameters.at("PLC_symbol");
                    plc_type_str = descr.interface_info.parameters.at("PLC_type");
                    if (descr.interface_info.parameters.count("n_elements"))
                    {
                        num_elements = std::stoul(descr.interface_info.parameters.at("n_elements"));
                    }
                    if (descr.interface_info.parameters.count("index"))
                    {
                        plc_index = std::stoul(descr.interface_info.parameters.at("index"));
                    }
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(getLogger(), "Error parsing PLC parameters for command interface '%s': %s. Check URDF.",
                                 name.c_str(), e.what());
                    continue;
                }

                if (processed_plc_symbols.find(plc_symbol) == processed_plc_symbols.end())
                {
                    ADSDataLayout layout;
                    layout.plc_name_symbolic = plc_symbol;
                    layout.num_elements = num_elements;
                    layout.plc_type = strToPlcType(plc_type_str);
                    layout.ros2_interfaces_.emplace(std::make_pair(plc_index, name));

                    if (layout.plc_type == PLCType::UNKNOWN || layout.plc_type == PLCType::STRING)
                    {
                        RCLCPP_ERROR(getLogger(), "Skipping command variable '%s' due to UNSUPPORTED or UNKNOWN PLC_type '%s'.", layout.plc_name_symbolic.c_str(), plc_type_str.c_str());
                    }
                    else
                    {
                        layout.plc_element_byte_size = plcTypeByteSize(layout.plc_type);
                        ads_item_layouts_write_.push_back(std::move(layout));
                        processed_plc_symbols[plc_symbol] = true;
                    }
                }
                // The symbol already exists
                else
                {
                    // Look for the ADS Data Layout of the corresponding PLC symbol
                    auto it = std::find_if(ads_item_layouts_write_.begin(), ads_item_layouts_write_.end(),
                                           [&plc_symbol](const ADSDataLayout &layout)
                                           { return layout.plc_name_symbolic == plc_symbol; });

                    // Add the command interface name the layout
                    (*it).ros2_interfaces_.emplace(std::make_pair(plc_index, name));
                }
            }
        };

        init_ads_write_layout(joint_command_interfaces_);
        init_ads_write_layout(gpio_command_interfaces_);
    }

    hardware_interface::CallbackReturn BeckhoffADSHardwareInterface::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        start_io_threads();
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn BeckhoffADSHardwareInterface::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // TODO: send some safety commands to the PLC?
        stop_io_threads();
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type BeckhoffADSHardwareInterface::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // Notification mode: publish the cached values, no network I/O here. A NaN slot means no
        // sample yet (normal at startup), which is not an error. The only fault raised is a cyclic
        // notification that stopped updating within notif_staleness_ns_; on-change symbols and
        // not-yet-seen symbols stay exempt.
        if (read_via_notifications_)
        {
            bool ok = true;
            const long long now_ns = (notif_staleness_ns_ > 0) ? steady_now_ns() : 0;
            for (const auto &target : notif_read_targets_)
            {
                set_state(target.state_interface_name, target.cache->load(std::memory_order_acquire));

                if (target.cyclic && notif_staleness_ns_ > 0)
                {
                    const long long last = target.last_update_ns->load(std::memory_order_relaxed);
                    if (last != 0 && (now_ns - last) > notif_staleness_ns_)
                    {
                        ok = false;
                    }
                }
            }

            if (!ok)
            {
                RCLCPP_WARN_THROTTLE(getLogger(), *logging_throttle_clock_, 1000,
                                     "Notification read: a cyclic state interface has gone stale.");
            }
            return ok ? hardware_interface::return_type::OK
                      : hardware_interface::return_type::ERROR;
        }

        // Polling mode: the reader thread performs the SUM read off the control loop and decodes
        // into polling_read_cache_. Here we only publish the latest cached values.
        if (num_items_read_ == 0)
        {
            return hardware_interface::return_type::OK;
        }

        for (size_t i = 0; i < ads_read_instructions_.size(); ++i)
        {
            set_state(ads_read_instructions_[i].state_interface_name,
                      polling_read_cache_[i].load(std::memory_order_acquire));
        }
        return read_comms_ok_.load(std::memory_order_acquire)
                   ? hardware_interface::return_type::OK
                   : hardware_interface::return_type::ERROR;
    }

    void BeckhoffADSHardwareInterface::reader_loop()
    {
        // Back off after a failure so a broken link does not spin at full CPU.
        constexpr auto ERROR_BACKOFF = std::chrono::milliseconds(10);

        while (!read_stop_.load(std::memory_order_acquire))
        {
            const auto cycle_start = std::chrono::steady_clock::now();

            // Only the reader touches the read response buffer, so decode happens outside the
            // lock; the lock only guards the shared single-port round-trip against the writer.
            uint32_t bytes_read_from_plc = 0;
            long ads_sum_read_error;
            {
                std::lock_guard<std::mutex> io_lock(ads_io_mutex_);
                ads_sum_read_error = ads_device_->ReadWriteReqEx2(
                    ADSIGRP_SUMUP_READ,
                    num_items_read_,
                    ads_buffer_sum_read_response_.size(),
                    ads_buffer_sum_read_response_.data(),
                    ads_buffer_sum_read_request_.size(),
                    ads_buffer_sum_read_request_.data(),
                    &bytes_read_from_plc);
            }

            if (ads_sum_read_error != ADSERR_NOERR)
            {
                RCLCPP_ERROR_THROTTLE(getLogger(), *logging_throttle_clock_, 1000,
                                      "Overall ADS Sum Read Error: 0x%lX.", ads_sum_read_error);
                read_comms_ok_.store(false, std::memory_order_release);
                std::this_thread::sleep_for(ERROR_BACKOFF);
                continue;
            }

            if (bytes_read_from_plc != ads_buffer_sum_read_response_.size())
            {
                RCLCPP_ERROR_THROTTLE(getLogger(), *logging_throttle_clock_, 1000,
                                      "ADS Sum Read size mismatch. Expected %zu, Got %u.",
                                      ads_buffer_sum_read_response_.size(), bytes_read_from_plc);
                read_comms_ok_.store(false, std::memory_order_release);
                std::this_thread::sleep_for(ERROR_BACKOFF);
                continue;
            }

            bool any_item_read_failed = false;
            for (size_t i = 0; i < ads_read_instructions_.size(); ++i)
            {
                const auto &read_instruction = ads_read_instructions_[i];
                uint32_t item_error_code;
                memcpy(&item_error_code,
                       ads_buffer_sum_read_response_.data() + read_instruction.read_buffer_offset_error_code,
                       sizeof(uint32_t));

                if (item_error_code != ADSERR_NOERR)
                {
                    RCLCPP_WARN_THROTTLE(getLogger(), *logging_throttle_clock_, 1000,
                                         "ADS Sum Read operation corresponding to the state interface '%s' failed: 0x%X.",
                                         read_instruction.state_interface_name.c_str(), item_error_code);
                    any_item_read_failed = true;
                    continue;
                }

                const uint8_t *ptr_plc_element_current = ads_buffer_sum_read_response_.data() + read_instruction.read_buffer_offset_data;
                polling_read_cache_[i].store(decode_plc_element(read_instruction.plc_type, ptr_plc_element_current),
                                             std::memory_order_release);
            }
            read_comms_ok_.store(!any_item_read_failed, std::memory_order_release);

            // Optional pacing to cap PLC load; period 0 = unpaced.
            if (read_poll_period_ns_ > 0)
            {
                const auto target = cycle_start + std::chrono::nanoseconds(read_poll_period_ns_);
                std::this_thread::sleep_until(target);
            }
        }
    }

    hardware_interface::return_type BeckhoffADSHardwareInterface::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        if (num_items_write_ == 0)
        {
            return hardware_interface::return_type::OK;
        }

        for (const auto &write_instruction : ads_write_instructions_)
        {
            uint8_t *ptr_write_buffer_destination_current = ads_buffer_sum_write_request_.data() + write_instruction.write_buffer_offset_data;

            // TODO: performance - Hoist the switch/case above for loop?

            // store the current val and reset the ros-side command value
            double val = get_command(write_instruction.command_interface_name);
            set_command(write_instruction.command_interface_name, std::numeric_limits<double>::quiet_NaN());

            if (std::isnan(val))
            {
                // if the original value was NaN and there exist a state interface of the same name, write corresponding state interface
                if (!write_instruction.fallback_state_interface_name.empty())
                {
                    val = get_state(write_instruction.fallback_state_interface_name);
                }

                // if we STILL don't have a fallback value on, don't update the write buffer.
                // the last valid command is written
                if (std::isnan(val))
                {
                    continue;
                }
            }

            switch (write_instruction.plc_type)
            {
            case PLCType::LREAL:
            {
                // val is already double (LREAL is 8 bytes - 64 bit)
                memcpy(ptr_write_buffer_destination_current, &val, plcTypeByteSize(write_instruction.plc_type));
                break;
            }
            case PLCType::REAL:
            {
                float plc_val = static_cast<float>(val);
                memcpy(ptr_write_buffer_destination_current, &plc_val, plcTypeByteSize(write_instruction.plc_type));
                break;
            }
            case PLCType::BOOL:
            {
                // bool is size of byte in PLC
                uint8_t plc_val = (val != 0.0) ? 1 : 0;
                memcpy(ptr_write_buffer_destination_current, &plc_val, plcTypeByteSize(write_instruction.plc_type));
                break;
            }
            case PLCType::SINT:
            {
                int8_t plc_val = static_cast<int8_t>(std::round(val));
                memcpy(ptr_write_buffer_destination_current, &plc_val, plcTypeByteSize(write_instruction.plc_type));
                break;
            }
            case PLCType::USINT:
            case PLCType::BYTE:
            {
                uint8_t plc_val = static_cast<uint8_t>(std::round(val));
                memcpy(ptr_write_buffer_destination_current, &plc_val, plcTypeByteSize(write_instruction.plc_type));
                break;
            }
            case PLCType::INT:
            {
                int16_t plc_val = static_cast<int16_t>(std::round(val));
                memcpy(ptr_write_buffer_destination_current, &plc_val, plcTypeByteSize(write_instruction.plc_type));
                break;
            }
            case PLCType::UINT:
            {
                uint16_t plc_val = static_cast<uint16_t>(std::round(val));
                memcpy(ptr_write_buffer_destination_current, &plc_val, plcTypeByteSize(write_instruction.plc_type));
                break;
            }
            case PLCType::DINT:
            {
                int32_t plc_val = static_cast<int32_t>(std::round(val));
                memcpy(ptr_write_buffer_destination_current, &plc_val, plcTypeByteSize(write_instruction.plc_type));
                break;
            }
            case PLCType::UDINT:
            {
                uint32_t plc_val = static_cast<uint32_t>(std::round(val));
                memcpy(ptr_write_buffer_destination_current, &plc_val, plcTypeByteSize(write_instruction.plc_type));
                break;
            }
            /* String not supported for now
            case PLCType::STRING: break;
            */
            case PLCType::UNKNOWN:
            default:
                RCLCPP_FATAL(getLogger(), "UNKNOWN PLC type (%d) for the interface '%s' during write. Sending zeroed data of size %zu.",
                             static_cast<int>(write_instruction.plc_type), write_instruction.command_interface_name.c_str(), plcTypeByteSize(write_instruction.plc_type));
                return hardware_interface::return_type::ERROR;
                break;
            }
        }

        // Hand the freshly packed request to the writer thread and return immediately. A newer
        // buffer overwrites one not yet sent, so the writer never backlogs (only the latest
        // setpoints matter).
        {
            std::lock_guard<std::mutex> lock(write_mutex_);
            write_pending_request_ = ads_buffer_sum_write_request_;
            write_pending_ = true;
        }
        write_cv_.notify_one();

        return write_comms_ok_.load(std::memory_order_acquire)
                   ? hardware_interface::return_type::OK
                   : hardware_interface::return_type::ERROR;
    }

    void BeckhoffADSHardwareInterface::writer_loop()
    {
        std::vector<uint8_t> send_buffer;
        while (true)
        {
            {
                std::unique_lock<std::mutex> lock(write_mutex_);
                write_cv_.wait(lock, [this]
                               { return write_pending_ || write_stop_; });
                if (write_stop_)
                {
                    break;
                }
                // O(1) swap, no copy. write() keeps its own stable buffer, so skipped fields
                // retain their last value.
                send_buffer.swap(write_pending_request_);
                write_pending_ = false;
            }

            // Lock only around the round-trip; the writer owns the write response buffer, so its
            // error decoding below runs unlocked. The gap between the reader's round-trips lets
            // the writer take the port, so it is not starved by the continuous reader.
            uint32_t bytes_response_buffer_from_plc = 0;
            long ads_sum_write_error;
            {
                std::lock_guard<std::mutex> io_lock(ads_io_mutex_);
                ads_sum_write_error = ads_device_->ReadWriteReqEx2(
                    ADSIGRP_SUMUP_WRITE,
                    num_items_write_,
                    ads_buffer_sum_write_response_.size(),
                    ads_buffer_sum_write_response_.data(),
                    send_buffer.size(),
                    send_buffer.data(),
                    &bytes_response_buffer_from_plc);
            }

            if (ads_sum_write_error != ADSERR_NOERR)
            {
                RCLCPP_ERROR_THROTTLE(getLogger(), *logging_throttle_clock_, 1000,
                                      "Overall ADS Sum Write Error: 0x%lX.", ads_sum_write_error);
                write_comms_ok_.store(false, std::memory_order_release);
                continue;
            }

            if (bytes_response_buffer_from_plc != ads_buffer_sum_write_response_.size())
            {
                RCLCPP_ERROR_THROTTLE(getLogger(), *logging_throttle_clock_, 1000,
                                      "ADS Sum Write response size mismatch (error codes). Expected %zu, Got %u.",
                                      ads_buffer_sum_write_response_.size(), bytes_response_buffer_from_plc);
            }

            // One error code per write item, in request order: index i maps to layout i.
            bool any_item_write_failed = false;
            for (size_t i = 0; i < num_items_write_; ++i)
            {
                uint32_t item_error_code;
                memcpy(&item_error_code, ads_buffer_sum_write_response_.data() + i * sizeof(uint32_t), sizeof(uint32_t));
                if (item_error_code != ADSERR_NOERR)
                {
                    RCLCPP_WARN_THROTTLE(getLogger(), *logging_throttle_clock_, 1000,
                                         "ADS Sum Write sub-op for '%s' (handle 0x%X) failed: 0x%X",
                                         ads_item_layouts_write_[i].plc_name_symbolic.c_str(), ads_item_layouts_write_[i].ads_handle, item_error_code);
                    any_item_write_failed = true;
                }
            }
            write_comms_ok_.store(!any_item_write_failed, std::memory_order_release);
        }
    }

    void BeckhoffADSHardwareInterface::start_io_threads()
    {
        // Clear any stale fault before (re)starting.
        write_comms_ok_.store(true, std::memory_order_release);
        read_comms_ok_.store(true, std::memory_order_release);

        if (num_items_write_ > 0 && !write_thread_.joinable())
        {
            {
                std::lock_guard<std::mutex> lock(write_mutex_);
                write_stop_ = false;
                write_pending_ = false;
            }
            write_thread_ = std::thread(&BeckhoffADSHardwareInterface::writer_loop, this);
        }

        // Reader thread only needed in polling mode; notifications cache via the dispatcher.
        if (!read_via_notifications_ && num_items_read_ > 0 && !read_thread_.joinable())
        {
            read_stop_.store(false, std::memory_order_release);
            read_thread_ = std::thread(&BeckhoffADSHardwareInterface::reader_loop, this);
        }
    }

    void BeckhoffADSHardwareInterface::stop_io_threads()
    {
        {
            std::lock_guard<std::mutex> lock(write_mutex_);
            write_stop_ = true;
        }
        write_cv_.notify_all();
        if (write_thread_.joinable())
        {
            write_thread_.join();
        }

        read_stop_.store(true, std::memory_order_release);
        if (read_thread_.joinable())
        {
            read_thread_.join();
        }
    }

    hardware_interface::CallbackReturn BeckhoffADSHardwareInterface::on_shutdown(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(getLogger(), "Releasing ADS resources...");
        // Stop the worker threads before touching the device; both call through ads_device_.
        // Safe to call even if on_deactivate already joined them.
        stop_io_threads();
        // Delete notifications BEFORE the device: each AdsNotification's destructor calls
        // DeleteNotification, which dereferences ads_device_. This also drains in-flight
        // callbacks and returns the registry slots.
        teardown_notifications();
        // Release symbol handles before the device, same reason; this fixed the Ctrl-C segfault.
        release_ads_handles();
        if (ads_device_)
        {
            ads_device_.reset();
        }
        RCLCPP_INFO(getLogger(), "ADS resources released.");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    void BeckhoffADSHardwareInterface::release_ads_handles()
    {
        // reset() destroys the held AdsHandle, whose deleter releases the PLC symbol handle via
        // ads_device_. Callers guarantee ads_device_ is still valid at this point.
        for (auto &layout : ads_item_layouts_read_)
        {
            layout.ads_handle_owner.reset();
        }
        for (auto &layout : ads_item_layouts_write_)
        {
            layout.ads_handle_owner.reset();
        }
    }

    bool BeckhoffADSHardwareInterface::configure_ads_device()
    {
        RCLCPP_INFO(getLogger(), "Configuring ADS device...");
        const auto &params = info_.hardware_parameters;
        try
        {
            std::string plc_ip = params.at("plc_ip_address");
            std::string plc_ams_net_id_str = params.at("plc_ams_net_id");
            std::string local_ams_net_id_str = params.at("local_ams_net_id");
            uint16_t plc_ams_port = std::stoul(params.at("plc_ams_port"));

            AmsNetId remote_net_id;
            if (sscanf(plc_ams_net_id_str.c_str(), "%hhu.%hhu.%hhu.%hhu.%hhu.%hhu",
                       &remote_net_id.b[0], &remote_net_id.b[1], &remote_net_id.b[2],
                       &remote_net_id.b[3], &remote_net_id.b[4], &remote_net_id.b[5]) != 6)
            {
                RCLCPP_FATAL(getLogger(), "\tInvalid format for 'plc_ams_net_id'. Expected 'x.x.x.x.x.x'.");
                return false;
            }

            AmsNetId local_net_id;
            if (sscanf(local_ams_net_id_str.c_str(), "%hhu.%hhu.%hhu.%hhu.%hhu.%hhu",
                       &local_net_id.b[0], &local_net_id.b[1], &local_net_id.b[2],
                       &local_net_id.b[3], &local_net_id.b[4], &local_net_id.b[5]) != 6)
            {
                RCLCPP_FATAL(getLogger(), "\tInvalid format for 'local_ams_net_id'. Expected 'x.x.x.x.x.x'.");
                return false;
            }

            bhf::ads::SetLocalAddress(local_net_id);
            ads_device_ = std::make_unique<AdsDevice>(plc_ip, remote_net_id, plc_ams_port);
            RCLCPP_INFO(getLogger(), "\tTimeout is: %u", ads_device_->GetTimeout());

            RCLCPP_INFO(getLogger(), "\tADS Device configured for PLC: %s, Port: %u", plc_ip.c_str(), plc_ams_port);
            RCLCPP_INFO(getLogger(), "\tPLC AMS NetID: %s", plc_ams_net_id_str.c_str());

            RCLCPP_INFO(getLogger(), "Requesting Device state...");
            AdsDeviceState deviceState = ads_device_->GetState();
            RCLCPP_INFO(getLogger(), "\tCommunication successful! ADS State: %d, DeviceState: %d", deviceState.ads, deviceState.device);
        }
        catch (const std::out_of_range &ex)
        {
            RCLCPP_FATAL(getLogger(), "\tMissing required URDF <hardware> parameter: %s", ex.what());
            return false;
        }
        catch (const AdsException &ex)
        {
            RCLCPP_FATAL(getLogger(), "\tADS Exception during connection: %s (Error Code: 0x%lX)", ex.what(), ex.errorCode);
            return false;
        }
        catch (const std::exception &ex)
        {
            RCLCPP_FATAL(getLogger(), "\tError during ADS connection config: %s", ex.what());
            return false;
        }

        return true;
    }

    PLCType BeckhoffADSHardwareInterface::strToPlcType(const std::string &type_str_param)
    {
        std::string type_str = type_str_param;
        std::transform(type_str.begin(), type_str.end(), type_str.begin(), ::toupper);

        if (type_str == "LREAL")
            return PLCType::LREAL;
        if (type_str == "REAL")
            return PLCType::REAL;
        if (type_str == "BOOL")
            return PLCType::BOOL;
        if (type_str == "UDINT")
            return PLCType::UDINT;
        if (type_str == "DINT")
            return PLCType::DINT;
        if (type_str == "UINT")
            return PLCType::UINT;
        if (type_str == "INT")
            return PLCType::INT;
        if (type_str == "USINT")
            return PLCType::USINT;
        if (type_str == "SINT")
            return PLCType::SINT;
        if (type_str == "BYTE")
            return PLCType::BYTE;
        if (type_str == "STRING")
            return PLCType::STRING;

        RCLCPP_ERROR(getLogger(), "Unknown PLC type string: '%s'", type_str_param.c_str());
        return PLCType::UNKNOWN;
    }

    size_t BeckhoffADSHardwareInterface::plcTypeByteSize(PLCType plc_type_enum)
    {
        switch (plc_type_enum)
        {
        case PLCType::LREAL:
            return 8;
        case PLCType::REAL:
            return 4;
        case PLCType::BOOL:
            return 1;
        case PLCType::UDINT:
            return 4;
        case PLCType::DINT:
            return 4;
        case PLCType::UINT:
            return 2;
        case PLCType::INT:
            return 2;
        case PLCType::USINT:
            return 1;
        case PLCType::SINT:
            return 1;
        case PLCType::BYTE:
            return 1;
        // case PLCType::STRING: not currently supported
        case PLCType::UNKNOWN:
        default:
            RCLCPP_ERROR(getLogger(), "Cannot get byte size for UNKNOWN or unhandled PLC type enum value: %d", static_cast<int>(plc_type_enum));
            return 0;
        }
    }

} // namespace beckhoff_ads_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    beckhoff_ads_hardware_interface::BeckhoffADSHardwareInterface, hardware_interface::SystemInterface)