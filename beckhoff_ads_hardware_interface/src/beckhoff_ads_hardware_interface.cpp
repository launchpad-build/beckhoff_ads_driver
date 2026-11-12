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

#include <array>
#include <cmath>
#include <limits>
#include <vector>
#include <cstdint>
#include <cstring> // std::memcpy
#include <chrono>
#include <thread>
#include <tuple>

#include <pthread.h>
#include <sched.h>
#include <algorithm> // std::transform

#include "beckhoff_ads_hardware_interface/ads_interface_utilities.hpp"
#include "beckhoff_ads_hardware_interface/beckhoff_ads_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace beckhoff_ads_hardware_interface
{
    namespace
    {
        // Decodes one PLC element at src into a double.
        // so the two never diverge. Returns NaN for unsupported types.
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

        // Encodes a double into one PLC element at dst; the inverse of decode_plc_element.
        bool encode_plc_element(PLCType plc_type, double val, uint8_t *dst)
        {
            bool result = true;
            switch (plc_type)
            {
            case PLCType::LREAL:
            {
                std::memcpy(dst, &val, sizeof(val));
                break;
            }
            case PLCType::REAL:
            {
                const float v = static_cast<float>(val);
                std::memcpy(dst, &v, sizeof(v));
                break;
            }
            case PLCType::BOOL:
            {
                const uint8_t v = (val != 0.0) ? 1 : 0;
                std::memcpy(dst, &v, sizeof(v));
                break;
            }
            case PLCType::SINT:
            {
                const int8_t v = static_cast<int8_t>(std::round(val));
                std::memcpy(dst, &v, sizeof(v));
                break;
            }
            case PLCType::USINT:
            case PLCType::BYTE:
            {
                const uint8_t v = static_cast<uint8_t>(std::round(val));
                std::memcpy(dst, &v, sizeof(v));
                break;
            }
            case PLCType::INT:
            {
                const int16_t v = static_cast<int16_t>(std::round(val));
                std::memcpy(dst, &v, sizeof(v));
                break;
            }
            case PLCType::UINT:
            {
                const uint16_t v = static_cast<uint16_t>(std::round(val));
                std::memcpy(dst, &v, sizeof(v));
                break;
            }
            case PLCType::DINT:
            {
                const int32_t v = static_cast<int32_t>(std::round(val));
                std::memcpy(dst, &v, sizeof(v));
                break;
            }
            case PLCType::UDINT:
            {
                const uint32_t v = static_cast<uint32_t>(std::round(val));
                std::memcpy(dst, &v, sizeof(v));
                break;
            }
            default:
                result = false;
                break;
            }
            return result;
        }
    } // namespace

    BeckhoffADSHardwareInterface::~BeckhoffADSHardwareInterface()
    {
        // Backstop if no lifecycle shutdown ran: a joinable std::thread would call std::terminate.
        stop_io();
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
        stop_io();

        // Release any symbol handles from a previous configure cycle before configure_ads_device()
        // replaces the devices below; the handle deleters call through the issuing device.
        release_ads_handles();

        // Optional pacing between SUM reads, to cap PLC load. Absent or invalid = unpaced.
        read_poll_period_ns_ = 0;
        try
        {
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
            RCLCPP_WARN(getLogger(), "Invalid read_poll_period_ms: %s. Running unpaced.", ex.what());
        }

        try
        {
            auto grace_it = info_.hardware_parameters.find("comms_outage_grace_ms");
            if (grace_it != info_.hardware_parameters.end())
            {
                const double ms = std::stod(grace_it->second);
                if (std::isfinite(ms) && ms >= 0.0)
                {
                    comms_outage_grace_ = std::chrono::milliseconds(static_cast<long long>(ms));
                }
            }
        }
        catch (const std::exception &ex)
        {
            RCLCPP_WARN(getLogger(), "Invalid comms_outage_grace_ms: %s. Using default %ld ms.",
                        ex.what(), static_cast<long>(comms_outage_grace_.count()));
        }

        read_outage_grace_ = comms_outage_grace_;
        try
        {
            const auto grace_it = info_.hardware_parameters.find("read_outage_grace_ms");
            if (grace_it != info_.hardware_parameters.end())
            {
                const double ms = std::stod(grace_it->second);
                if (std::isfinite(ms) && ms >= 0.0 &&
                    ms < static_cast<double>(std::chrono::milliseconds::max().count()))
                {
                    read_outage_grace_ = std::chrono::milliseconds(static_cast<long long>(ms));
                }
            }
        }
        catch (const std::exception &ex)
        {
            RCLCPP_WARN(getLogger(), "Invalid read_outage_grace_ms: %s. Using %ld ms.",
                        ex.what(), static_cast<long>(read_outage_grace_.count()));
        }

        outage_latches_ = true;
        {
            const auto behaviour_it = info_.hardware_parameters.find("outage_behaviour");
            if (behaviour_it != info_.hardware_parameters.end())
            {
                if (behaviour_it->second == "resume")
                {
                    outage_latches_ = false;
                }
                else if (behaviour_it->second != "latch")
                {
                    RCLCPP_WARN(getLogger(), "Unknown outage_behaviour '%s'. Expected latch or resume. Latching.",
                                behaviour_it->second.c_str());
                }
            }
        }

        read_staleness_timeout_ns_ = 100000000;
        try
        {
            const auto staleness_it = info_.hardware_parameters.find("read_staleness_timeout_ms");
            if (staleness_it != info_.hardware_parameters.end())
            {
                const double ms = std::stod(staleness_it->second);
                if (std::isfinite(ms) && ms >= 0.0)
                {
                    read_staleness_timeout_ns_ = static_cast<long long>(ms * 1e6);
                }
            }
        }
        catch (const std::exception &ex)
        {
            RCLCPP_WARN(getLogger(), "Invalid read_staleness_timeout_ms: %s. Using default 100 ms.", ex.what());
        }
        if (read_staleness_timeout_ns_ > 0 && read_poll_period_ns_ > 0)
        {
            read_staleness_timeout_ns_ = std::max(read_staleness_timeout_ns_, 3 * read_poll_period_ns_);
        }

        activation_first_sample_timeout_ = std::chrono::milliseconds(1000);
        try
        {
            const auto activation_it = info_.hardware_parameters.find("activation_first_sample_timeout_ms");
            if (activation_it != info_.hardware_parameters.end())
            {
                const double ms = std::stod(activation_it->second);
                if (std::isfinite(ms) && ms >= 0.0)
                {
                    activation_first_sample_timeout_ = std::chrono::milliseconds(static_cast<long long>(ms));
                }
            }
        }
        catch (const std::exception &ex)
        {
            RCLCPP_WARN(getLogger(), "Invalid activation_first_sample_timeout_ms: %s. Using default 1000 ms.",
                        ex.what());
        }

        {
            auto param_or_empty = [this](const char *key) -> std::string
            {
                const auto it = info_.hardware_parameters.find(key);
                return (it != info_.hardware_parameters.end()) ? it->second : std::string{};
            };
            const utilities::ThreadSchedulingParseResult scheduling_parse =
                utilities::parseThreadScheduling(param_or_empty("io_thread_scheduling_policy"),
                                                 param_or_empty("io_thread_priority"),
                                                 param_or_empty("io_thread_cpu_affinity"));
            if (!scheduling_parse.valid)
            {
                RCLCPP_WARN(getLogger(), "Invalid I/O thread scheduling parameters: %s. Using the defaults.",
                            scheduling_parse.error.c_str());
            }
            io_thread_scheduling_ = scheduling_parse.config;
        }

        heartbeat_symbol_.clear();
        auto heartbeat_it = info_.hardware_parameters.find("heartbeat_plc_symbol");
        if (heartbeat_it != info_.hardware_parameters.end() && !heartbeat_it->second.empty())
        {
            heartbeat_symbol_ = heartbeat_it->second;
            RCLCPP_INFO(getLogger(), "ADS link heartbeat enabled on PLC symbol '%s'.", heartbeat_symbol_.c_str());
        }

        setpoint_sequence_symbol_.clear();
        const auto sequence_it = info_.hardware_parameters.find("setpoint_sequence_plc_symbol");
        if (sequence_it != info_.hardware_parameters.end() && !sequence_it->second.empty())
        {
            setpoint_sequence_symbol_ = sequence_it->second;
            RCLCPP_INFO(getLogger(), "Setpoint sequence enabled on PLC symbol '%s'.",
                        setpoint_sequence_symbol_.c_str());
        }
        setpoint_timestamp_symbol_.clear();
        const auto stamp_it = info_.hardware_parameters.find("setpoint_time_plc_symbol");
        if (stamp_it != info_.hardware_parameters.end() && !stamp_it->second.empty())
        {
            setpoint_timestamp_symbol_ = stamp_it->second;
            RCLCPP_INFO(getLogger(), "Setpoint timestamp enabled on PLC symbol '%s'.",
                        setpoint_timestamp_symbol_.c_str());
        }

        // Configure ADS Client Device
        if (!configure_ads_device())
        {
            RCLCPP_FATAL(getLogger(), "Failed to configure ADS device from URDF parameters.");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Fill the ADSDataLayout vectors for read and write operations
        if (!ads_read_layout_configure() || !ads_write_layout_configure())
        {
            return CallbackReturn::ERROR;
        }

        // Request handles for all symbolic PLC variable names
        RCLCPP_INFO(getLogger(), "Fetching ADS handles for configured PLC variables...");
        bool required_symbol_missing = false;

        auto resolve_handles = [&](std::vector<ADSDataLayout> &layouts, const char *direction,
                                   AdsDevice &device, std::vector<std::string> &dropped_interfaces)
        {
            for (auto &layout : layouts)
            {
                try
                {
                    layout.ads_handle_owner.emplace(device.GetHandle(layout.plc_name_symbolic));
                    layout.ads_handle = **layout.ads_handle_owner;
                    layout.handle_resolved = true;
                }
                catch (const std::exception &ex)
                {
                    layout.handle_resolved = false;
                    if (layout.optional)
                    {
                        RCLCPP_WARN(getLogger(),
                                    "\tOptional %s symbol '%s' is not on the PLC (%s). Skipping it; its interfaces are set to their declared initial_value, or 0.0 without one.",
                                    direction, layout.plc_name_symbolic.c_str(), ex.what());
                        for (const auto &[index, interface_name] : layout.ros2_interfaces_)
                        {
                            dropped_interfaces.push_back(interface_name);
                        }
                    }
                    else
                    {
                        RCLCPP_FATAL(getLogger(),
                                     "\tRequired %s symbol '%s' is not on the PLC (%s). Flash the PLC project first, or mark the interface optional.",
                                     direction, layout.plc_name_symbolic.c_str(), ex.what());
                        required_symbol_missing = true;
                    }
                }
            }

            // ADSDataLayout is not move-assignable, so no erase-remove here.
            std::vector<ADSDataLayout> resolved;
            resolved.reserve(layouts.size());
            for (auto &layout : layouts)
            {
                if (layout.handle_resolved)
                {
                    resolved.push_back(std::move(layout));
                }
            }
            layouts = std::move(resolved);
        };

        std::vector<std::string> dropped_state_interfaces;
        std::vector<std::string> dropped_command_interfaces;
        resolve_handles(ads_item_layouts_read_, "read", *ads_device_, dropped_state_interfaces);
        resolve_handles(ads_item_layouts_write_, "write", *ads_device_, dropped_command_interfaces);
        settle_dropped_state_interfaces(dropped_state_interfaces);

        if (required_symbol_missing)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        RCLCPP_INFO(getLogger(), "\tHandles acquired");

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
                        const auto command_it = command_layout.ros2_interfaces_.find(k);
                        const auto state_it = state_layout.ros2_interfaces_.find(k);
                        if (command_it == command_layout.ros2_interfaces_.end() ||
                            state_it == state_layout.ros2_interfaces_.end())
                        {
                            continue; // sparse array indices: this element has no matching pair
                        }
                        command_layout.state_command_interfaces_map_.emplace(command_it->second, state_it->second);
                    }
                }
            }
        }

        // Pre-pack what we can for SUM read/write commands
        if (!build_sum_read_buffers())
        {
            RCLCPP_FATAL(getLogger(), "\tFailed to build ADS sum read buffer.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        for (auto &command_layout : ads_item_layouts_write_)
        {
            for (const auto &state_layout : ads_item_layouts_read_)
            {
                if (command_layout.plc_name_symbolic != state_layout.plc_name_symbolic) { continue; }
                if (command_layout.plc_type != state_layout.plc_type ||
                    command_layout.num_elements != state_layout.num_elements)
                {
                    RCLCPP_ERROR(getLogger(), "Mismatched read/write definitions for '%s'.",
                                 command_layout.plc_name_symbolic.c_str());
                    return CallbackReturn::ERROR;
                }
                command_layout.readback_data_offset = state_layout.offset_in_read_response_data;
                command_layout.readback_error_offset = state_layout.offset_in_read_response_error;
            }
            if (command_layout.ros2_interfaces_.size() != command_layout.num_elements &&
                !command_layout.readback_data_offset)
            {
                RCLCPP_ERROR(getLogger(), "Partial array '%s' requires a matching PLC readback.",
                             command_layout.plc_name_symbolic.c_str());
                return CallbackReturn::ERROR;
            }
        }

        if (!build_sum_write_buffers())
        {
            RCLCPP_FATAL(getLogger(), "\tFailed to build ADS sum write buffer.");
            return hardware_interface::CallbackReturn::ERROR;
        }

        read_hard_fault_.store(false, std::memory_order_release);
        write_hard_fault_.store(false, std::memory_order_release);
        return CallbackReturn::SUCCESS;
    }

    bool BeckhoffADSHardwareInterface::stateInterfaceHasDeclaredInitialValue(const std::string &interface_name) const
    {
        bool result = false;
        const std::array<const std::unordered_map<std::string, hardware_interface::InterfaceDescription> *, 3> maps = {
            &joint_state_interfaces_, &gpio_state_interfaces_, &sensor_state_interfaces_};
        for (const auto *map : maps)
        {
            const auto it = map->find(interface_name);
            if (it != map->end() && !it->second.interface_info.initial_value.empty())
            {
                result = true;
            }
        }
        return result;
    }

    void BeckhoffADSHardwareInterface::settle_dropped_state_interfaces(const std::vector<std::string> &dropped_interfaces)
    {
        for (const std::string &interface_name : dropped_interfaces)
        {
            if (!stateInterfaceHasDeclaredInitialValue(interface_name))
            {
                set_state(interface_name, 0.0);
            }
        }
    }

    bool BeckhoffADSHardwareInterface::build_sum_read_buffers()
    {
        ads_read_instructions_.clear();
        last_decoded_values_.clear();
        read_sample_sequence_ = 0;
        ads_buffer_sum_read_request_.clear();
        ads_buffer_sum_read_response_.clear();

        num_items_read_ = ads_item_layouts_read_.size();
        if (num_items_read_ == 0)
        {
            RCLCPP_INFO(getLogger(), "No items to configure for ADS Sum READ.");
        }
        else
        {
            RCLCPP_INFO(getLogger(), "Building ADS sum READ buffer...");

            size_t total_error_block_size = num_items_read_ * sizeof(uint32_t);
            size_t total_data_block_size = 0;
            for (const auto &layout : ads_item_layouts_read_)
            {
                total_data_block_size += layout.plc_element_byte_size * layout.num_elements;
            }

            ads_buffer_sum_read_response_.resize(total_error_block_size + total_data_block_size);

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
                    read_instruction.state_handle = get_state_interface_handle(interface_name);
                    const auto critical_it = layout.critical_policies_.find(interface_name);
                    read_instruction.critical =
                        (critical_it != layout.critical_policies_.end()) && critical_it->second;

                    // The state interfaces' names are ordered by ascending indexes of the PLC array thanks to layout.ros2_interfaces_ being a map
                    ads_read_instructions_.push_back(read_instruction);
                }

                current_data_offset += header.NumBytesData;
                current_error_offset += sizeof(uint32_t);
            }
            // NaN until the first SUM read; all slots pre-sized for the I/O worker.
            const size_t instruction_count = ads_read_instructions_.size();
            last_decoded_values_.assign(instruction_count, std::numeric_limits<double>::quiet_NaN());
            item_failure_start_.assign(instruction_count, std::nullopt);
            read_sample_buffer_.initialiseSlots(
                [instruction_count](ReadSample &slot)
                {
                    slot.values.assign(instruction_count, std::numeric_limits<double>::quiet_NaN());
                    slot.stamp = std::chrono::steady_clock::time_point{};
                    slot.sequence = 0;
                });

            RCLCPP_INFO(getLogger(), "ADS Sum READ configured for %zu items. Request: %zu bytes, Response: %zu bytes.",
                        num_items_read_, ads_buffer_sum_read_request_.size(), ads_buffer_sum_read_response_.size());
        }

        return true;
    }

    bool BeckhoffADSHardwareInterface::build_sum_write_buffers()
    {
        RCLCPP_INFO(getLogger(), "Building ADS sum WRITE buffer...");

        ads_write_instructions_.clear();
        ads_buffer_sum_write_request_.clear();
        ads_buffer_sum_write_response_.clear();
        write_item_spans_.clear();
        write_layout_dirty_.clear();
        write_layout_provided_count_.clear();
        ads_buffer_sum_write_compact_.clear();
        compact_layout_indices_.clear();

        num_items_write_ = ads_item_layouts_write_.size();
        if (num_items_write_ == 0)
        {
            RCLCPP_INFO(getLogger(), "No items to configure for ADS Sum WRITE.");
        }
        else
        {
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

                utilities::SumWriteItemSpan span;
                span.header_offset = i * sizeof(ADS_ITEM_REQ_HEADER);
                span.header_length = sizeof(ADS_ITEM_REQ_HEADER);
                span.data_offset = layout.offset_in_write_request_data;
                span.data_length = header_block_ptr[i].NumBytesData;
                write_item_spans_.push_back(span);

                // For interfaces targeting the same PLC symbol
                for (const auto &[index, interface_name] : layout.ros2_interfaces_)
                {
                    // Fill the write instruction vector
                    WriteInstruction write_instruction;
                    write_instruction.write_buffer_offset_data = layout.offset_in_write_request_data + index * layout.plc_element_byte_size;
                    write_instruction.plc_type = layout.plc_type;
                    write_instruction.command_interface_name = interface_name;
                    write_instruction.fallback_state_interface_name = "";
                    write_instruction.source = classifyWriteValueSource(interface_name);
                    write_instruction.layout_index = i;
                    if (write_instruction.source == WriteValueSource::CONTROLLER)
                    {
                        write_instruction.command_handle = get_command_interface_handle(interface_name);
                    }

                    const auto policy_it = layout.fallback_policies_.find(interface_name);
                    write_instruction.fallback = (policy_it != layout.fallback_policies_.end())
                                                     ? policy_it->second
                                                     : CommandFallback::HOLD_LAST;

                    if (write_instruction.fallback == CommandFallback::MIRROR_STATE)
                    {
                        const auto state_it = layout.state_command_interfaces_map_.find(interface_name);
                        if (state_it != layout.state_command_interfaces_map_.end())
                        {
                            write_instruction.fallback_state_interface_name = state_it->second;
                            write_instruction.fallback_state_handle = get_state_interface_handle(state_it->second);
                        }
                        else
                        {
                            RCLCPP_WARN(getLogger(),
                                        "Command interface '%s' asks for a mirror_state fallback but no state interface "
                                        "shares its PLC symbol. Holding the last value instead.",
                                        interface_name.c_str());
                            write_instruction.fallback = CommandFallback::HOLD_LAST;
                        }
                    }

                    // The command interfaces' names are ordered by ascending indexes of the PLC array thanks to layout.ros2_interfaces_ being a map
                    ads_write_instructions_.push_back(write_instruction);
                }

                current_data_offset += header_block_ptr[i].NumBytesData;
                i++;
            }

            write_layout_dirty_.assign(num_items_write_, 0);
            write_layout_provided_count_.assign(num_items_write_, 0);
            write_values_.assign(ads_write_instructions_.size(), std::numeric_limits<double>::quiet_NaN());
            write_pending_request_.values = write_values_;

            RCLCPP_INFO(getLogger(), "ADS Sum WRITE configured for %zu items. Request: %zu bytes, Response: %zu bytes.",
                        num_items_write_, ads_buffer_sum_write_request_.size(), ads_buffer_sum_write_response_.size());
        }

        return true;
    }

    void BeckhoffADSHardwareInterface::warn_if_joint_motion_interface_is_32_bit(
        const std::string &interface_name,
        const hardware_interface::InterfaceDescription &description,
        bool is_joint,
        const std::string &plc_type_str)
    {
        const bool motion_interface =
            is_joint && (description.interface_info.name == hardware_interface::HW_IF_POSITION ||
                         description.interface_info.name == hardware_interface::HW_IF_VELOCITY);
        if (motion_interface && utilities::toUpperCopy(plc_type_str) == "REAL")
        {
            RCLCPP_WARN(getLogger(),
                        "Joint interface '%s' is declared as 32-bit REAL. A position or velocity "
                        "stream loses precision through a float; declare it LREAL on the PLC and here.",
                        interface_name.c_str());
        }
    }

    bool BeckhoffADSHardwareInterface::ads_read_layout_configure()
    {
        // Count all state interfaces to pre-allocate memory once and avoid reallocations.
        size_t num_state_interfaces = gpio_state_interfaces_.size() + joint_state_interfaces_.size() + sensor_state_interfaces_.size();

        // Reserve worst-case scenario for layouts (each interface targets a different PLC symbol)
        ads_item_layouts_read_.clear();
        ads_item_layouts_read_.reserve(num_state_interfaces);

        // Keep track of multiple interfaces targeting the same PLC symbol of type ARRAY[x], but different index
        std::map<std::string, bool> processed_plc_symbols;

        auto init_ads_read_layout =
            [&](const auto &type_state_interfaces_, const bool is_joint)
        {
            for (const auto &[name, descr] : type_state_interfaces_)
            {
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
                    RCLCPP_ERROR(getLogger(), "Error parsing PLC parameters for state interface '%s': %s. Check URDF.",
                                 name.c_str(), e.what());
                    return false;
                }

                if (plc_index >= num_elements)
                {
                    RCLCPP_ERROR(getLogger(), "PLC index for '%s' is outside the configured array.", name.c_str());
                    return false;
                }

                const bool interface_optional = descr.interface_info.parameters.count("optional") &&
                                                descr.interface_info.parameters.at("optional") == "true";

                bool interface_critical =
                    is_joint && (descr.interface_info.name == hardware_interface::HW_IF_POSITION ||
                                 descr.interface_info.name == hardware_interface::HW_IF_VELOCITY);
                if (descr.interface_info.parameters.count("critical"))
                {
                    interface_critical = descr.interface_info.parameters.at("critical") == "true";
                }

                warn_if_joint_motion_interface_is_32_bit(name, descr, is_joint, plc_type_str);

                // If this is the first time we see this symbol, create the layout
                if (processed_plc_symbols.find(plc_symbol) == processed_plc_symbols.end())
                {
                    ADSDataLayout layout;
                    layout.plc_name_symbolic = plc_symbol;
                    layout.num_elements = num_elements;
                    layout.plc_type = strToPlcType(plc_type_str);
                    layout.ros2_interfaces_.emplace(std::make_pair(plc_index, name));
                    layout.optional = interface_optional;
                    layout.critical_policies_.emplace(name, interface_critical);

                    if (layout.plc_type == PLCType::UNKNOWN || layout.plc_type == PLCType::STRING)
                    {
                        RCLCPP_ERROR(getLogger(), "Skipping state variable '%s' due to UNSUPPORTED or UNKNOWN PLC_type '%s'.", layout.plc_name_symbolic.c_str(), plc_type_str.c_str());
                        return false;
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

                    if (it->num_elements != num_elements || it->plc_type != strToPlcType(plc_type_str))
                    {
                        RCLCPP_ERROR(getLogger(), "Conflicting PLC array definitions for '%s'.", plc_symbol.c_str());
                        return false;
                    }
                    // Add the interface name the layout
                    (*it).ros2_interfaces_.emplace(std::make_pair(plc_index, name));
                    (*it).critical_policies_.emplace(name, interface_critical);
                    (*it).optional = (*it).optional && interface_optional;
                }
            }
            return true;
        };

        return init_ads_read_layout(joint_state_interfaces_, true) &&
               init_ads_read_layout(gpio_state_interfaces_, false) &&
               init_ads_read_layout(sensor_state_interfaces_, false);
    }

    bool BeckhoffADSHardwareInterface::ads_write_layout_configure()
    {
        // Count all command interfaces to pre-allocate memory once and avoid reallocations.
        size_t num_command_interfaces = joint_command_interfaces_.size() + gpio_command_interfaces_.size();

        // Reserve worst-case scenario for layouts (each interface targets a different PLC symbol)
        ads_item_layouts_write_.clear();
        ads_item_layouts_write_.reserve(num_command_interfaces);

        // Keep track of multiple interfaces targeting the same PLC symbol of type ARRAY[x], but different index
        std::map<std::string, bool> processed_plc_symbols;

        auto init_ads_write_layout =
            [&](const auto &type_command_interfaces_, const bool is_joint)
        {
            for (const auto &[name, descr] : type_command_interfaces_)
            {
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
                    return false;
                }

                if (plc_index >= num_elements)
                {
                    RCLCPP_ERROR(getLogger(), "PLC index for '%s' is outside the configured array.", name.c_str());
                    return false;
                }

                std::string fallback_str;
                if (descr.interface_info.parameters.count("command_fallback"))
                {
                    fallback_str = descr.interface_info.parameters.at("command_fallback");
                }
                warn_if_joint_motion_interface_is_32_bit(name, descr, is_joint, plc_type_str);

                CommandFallback fallback_policy;
                if (fallback_str.empty() && is_joint &&
                    descr.interface_info.name == hardware_interface::HW_IF_VELOCITY)
                {
                    // Zero is the safe default for a velocity command.
                    fallback_policy = CommandFallback::ZERO;
                    RCLCPP_INFO(getLogger(),
                                "Joint velocity command interface '%s' defaults to a zero fallback. "
                                "Set command_fallback to hold_last or mirror_state to override.",
                                name.c_str());
                }
                else
                {
                    fallback_policy = parseCommandFallback(fallback_str, name);
                }
                const bool interface_optional = descr.interface_info.parameters.count("optional") &&
                                                descr.interface_info.parameters.at("optional") == "true";

                if (processed_plc_symbols.find(plc_symbol) == processed_plc_symbols.end())
                {
                    ADSDataLayout layout;
                    layout.plc_name_symbolic = plc_symbol;
                    layout.num_elements = num_elements;
                    layout.plc_type = strToPlcType(plc_type_str);
                    layout.ros2_interfaces_.emplace(std::make_pair(plc_index, name));
                    layout.fallback_policies_.emplace(name, fallback_policy);
                    layout.optional = interface_optional;

                    if (layout.plc_type == PLCType::UNKNOWN || layout.plc_type == PLCType::STRING)
                    {
                        RCLCPP_ERROR(getLogger(), "Skipping command variable '%s' due to UNSUPPORTED or UNKNOWN PLC_type '%s'.", layout.plc_name_symbolic.c_str(), plc_type_str.c_str());
                        return false;
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

                    if (it->num_elements != num_elements || it->plc_type != strToPlcType(plc_type_str))
                    {
                        RCLCPP_ERROR(getLogger(), "Conflicting PLC array definitions for '%s'.", plc_symbol.c_str());
                        return false;
                    }
                    if (it->ros2_interfaces_.count(plc_index) != 0)
                    {
                        RCLCPP_ERROR(getLogger(), "Multiple command interfaces own '%s' element %zu.",
                                     plc_symbol.c_str(), plc_index);
                        return false;
                    }
                    // Add the command interface name the layout
                    (*it).ros2_interfaces_.emplace(std::make_pair(plc_index, name));
                    (*it).fallback_policies_.emplace(name, fallback_policy);
                    (*it).optional = (*it).optional && interface_optional;
                }
            }
            return true;
        };

        if (!init_ads_write_layout(joint_command_interfaces_, true) ||
            !init_ads_write_layout(gpio_command_interfaces_, false)) { return false; }

        if (!heartbeat_symbol_.empty())
        {
            append_synthetic_write_layout(heartbeat_symbol_, PLCType::UDINT, HEARTBEAT_INTERFACE_NAME);
        }
        if (!setpoint_sequence_symbol_.empty())
        {
            append_synthetic_write_layout(setpoint_sequence_symbol_, PLCType::UDINT,
                                          SETPOINT_SEQUENCE_INTERFACE_NAME);
        }
        if (!setpoint_timestamp_symbol_.empty())
        {
            append_synthetic_write_layout(setpoint_timestamp_symbol_, PLCType::LREAL,
                                          SETPOINT_TIMESTAMP_INTERFACE_NAME);
        }
        return true;
    }

    void BeckhoffADSHardwareInterface::append_synthetic_write_layout(
        const std::string &plc_symbol, PLCType plc_type, const char *interface_name)
    {
        ADSDataLayout layout;
        layout.plc_name_symbolic = plc_symbol;
        layout.num_elements = 1;
        layout.plc_type = plc_type;
        layout.plc_element_byte_size = plcTypeByteSize(plc_type);
        layout.ros2_interfaces_.emplace(0, interface_name);
        ads_item_layouts_write_.push_back(std::move(layout));
    }

    hardware_interface::CallbackReturn BeckhoffADSHardwareInterface::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        CallbackReturn result = CallbackReturn::SUCCESS;

        if (read_hard_fault_.load(std::memory_order_acquire) ||
            write_hard_fault_.load(std::memory_order_acquire))
        {
            RCLCPP_ERROR(getLogger(), "Reconfigure ADS handles before reactivating after an I/O failure.");
            return CallbackReturn::ERROR;
        }
        if (has_activated_)
        {
            for (auto &instruction : ads_write_instructions_)
            {
                instruction.last_value = std::numeric_limits<double>::quiet_NaN();
                instruction.has_been_commanded = false;
                if (instruction.command_handle &&
                    !set_command(instruction.command_handle, std::numeric_limits<double>::quiet_NaN(), false))
                {
                    return CallbackReturn::ERROR;
                }
            }
        }
        register_transaction_statistics();
        read_samples_published_.store(0, std::memory_order_release);
        start_io();

        if (num_items_read_ > 0 && activation_first_sample_timeout_.count() > 0)
        {
            const std::chrono::steady_clock::time_point deadline =
                std::chrono::steady_clock::now() + activation_first_sample_timeout_;
            while (result == CallbackReturn::SUCCESS &&
                   read_samples_published_.load(std::memory_order_acquire) == 0)
            {
                if (read_hard_fault_.load(std::memory_order_acquire))
                {
                    RCLCPP_ERROR(getLogger(), "Activation failed: the ADS link faulted before the first sample.");
                    result = CallbackReturn::FAILURE;
                }
                else if (std::chrono::steady_clock::now() >= deadline)
                {
                    RCLCPP_ERROR(getLogger(),
                                 "Activation failed: no PLC sample arrived within %ld ms.",
                                 static_cast<long>(activation_first_sample_timeout_.count()));
                    result = CallbackReturn::FAILURE;
                }
                else
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            }
            if (result != CallbackReturn::SUCCESS)
            {
                stop_io();
            }
        }

        if (result == CallbackReturn::SUCCESS) { has_activated_ = true; }
        return result;
    }

    hardware_interface::CallbackReturn BeckhoffADSHardwareInterface::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // TODO: send some safety commands to the PLC?
        stop_io();
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type BeckhoffADSHardwareInterface::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // The I/O worker performs the SUM read off the control loop and publishes whole
        // decoded samples. Here we only take the latest sample and copy it out.
        if (num_items_read_ == 0)
        {
            return hardware_interface::return_type::OK;
        }

        if (read_hard_fault_.load(std::memory_order_acquire) ||
            write_hard_fault_.load(std::memory_order_acquire))
        {
            return hardware_interface::return_type::ERROR;
        }
        if (io_stop_.load(std::memory_order_acquire))
        {
            for (const auto &instruction : ads_read_instructions_)
            {
                std::ignore = set_state(instruction.state_handle, std::numeric_limits<double>::quiet_NaN(), false);
            }
            return hardware_interface::return_type::OK;
        }
        read_sample_buffer_.refreshReadSlot();
        const ReadSample &sample = read_sample_buffer_.readSlot();

        for (size_t i = 0; i < ads_read_instructions_.size(); ++i)
        {
            std::ignore = set_state(ads_read_instructions_[i].state_handle, sample.values[i], false);
        }

        long long sample_age_ns = 0;
        if (sample.sequence > 0)
        {
            sample_age_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                std::chrono::steady_clock::now() - sample.stamp)
                                .count();
            stat_read_sample_age_ms_ = static_cast<double>(sample_age_ns) * 1e-6;
        }
        else
        {
            stat_read_sample_age_ms_ = std::numeric_limits<double>::quiet_NaN();
        }

        const bool sample_stale = read_staleness_timeout_ns_ > 0 && sample.sequence > 0 &&
                                  sample_age_ns > read_staleness_timeout_ns_;
        if (sample_stale)
        {
            if (outage_latches_) { read_hard_fault_.store(true, std::memory_order_release); }
            RCLCPP_ERROR_THROTTLE(getLogger(), *logging_throttle_clock_, 1000,
                                  "The published PLC state sample is %.1f ms old, over the %.1f ms staleness "
                                  "timeout. Surfacing an error rather than frozen feedback.",
                                  static_cast<double>(sample_age_ns) * 1e-6,
                                  static_cast<double>(read_staleness_timeout_ns_) * 1e-6);
        }

        return (sample_stale || read_hard_fault_.load(std::memory_order_acquire))
                   ? hardware_interface::return_type::ERROR
                   : hardware_interface::return_type::OK;
    }

    const char *BeckhoffADSHardwareInterface::adsErrorText(long error_code)
    {
        const char *text = "unrecognised ADS error code";
        switch (error_code)
        {
        case GLOBALERR_TARGET_PORT:
            text = "target port not found; the PLC runtime is probably not running";
            break;
        case GLOBALERR_MISSING_ROUTE:
            text = "target machine not found; no ADS route to the target";
            break;
        case ADSERR_DEVICE_SRVNOTSUPP:
            text = "service is not supported by the server";
            break;
        case ADSERR_DEVICE_INVALIDACCESS:
            text = "reading or writing this symbol is not permitted";
            break;
        case ADSERR_DEVICE_SYMBOLNOTFOUND:
            text = "symbol not found on the PLC; the PLC program may have changed";
            break;
        case ADSERR_CLIENT_SYNCTIMEOUT:
            text = "request timed out; no response from the PLC, check the network link and PLC state";
            break;
        default:
            break;
        }
        return text;
    }

    WriteValueSource BeckhoffADSHardwareInterface::classifyWriteValueSource(const std::string &interface_name)
    {
        WriteValueSource result = WriteValueSource::CONTROLLER;
        if (interface_name == HEARTBEAT_INTERFACE_NAME)
        {
            result = WriteValueSource::HEARTBEAT;
        }
        else if (interface_name == SETPOINT_SEQUENCE_INTERFACE_NAME)
        {
            result = WriteValueSource::SETPOINT_SEQUENCE;
        }
        else if (interface_name == SETPOINT_TIMESTAMP_INTERFACE_NAME)
        {
            result = WriteValueSource::SETPOINT_TIMESTAMP;
        }
        return result;
    }

    CommandFallback BeckhoffADSHardwareInterface::parseCommandFallback(
        const std::string &policy_str, const std::string &interface_name)
    {
        CommandFallback result = CommandFallback::HOLD_LAST;

        if (policy_str == "none")
        {
            result = CommandFallback::NONE;
        }
        else if (policy_str == "mirror_state")
        {
            result = CommandFallback::MIRROR_STATE;
        }
        else if (policy_str == "zero")
        {
            result = CommandFallback::ZERO;
        }
        else if (!policy_str.empty() && policy_str != "hold_last")
        {
            RCLCPP_WARN(getLogger(),
                        "Unknown command_fallback '%s' on interface '%s'. Expected none, hold_last, mirror_state or zero. "
                        "Holding the last value.",
                        policy_str.c_str(), interface_name.c_str());
        }

        return result;
    }

    void BeckhoffADSHardwareInterface::refresh_transaction_statistics()
    {
        stat_read_rtt_ms_ = static_cast<double>(read_rtt_ns_.load(std::memory_order_relaxed)) * 1e-6;
        stat_write_rtt_ms_ = static_cast<double>(write_rtt_ns_.load(std::memory_order_relaxed)) * 1e-6;
        stat_write_buffer_age_ms_ =
            static_cast<double>(write_buffer_age_ns_.load(std::memory_order_relaxed)) * 1e-6;
        stat_write_handoff_latency_ms_ =
            static_cast<double>(write_handoff_latency_ns_.load(std::memory_order_relaxed)) * 1e-6;
        stat_read_transactions_ = static_cast<double>(read_transactions_total_.load(std::memory_order_relaxed));
        stat_write_transactions_ = static_cast<double>(write_transactions_total_.load(std::memory_order_relaxed));
        stat_write_coalesced_ = static_cast<double>(write_coalesced_total_.load(std::memory_order_relaxed));
        stat_read_failures_ = static_cast<double>(read_failures_total_.load(std::memory_order_relaxed));
        stat_write_failures_ = static_cast<double>(write_failures_total_.load(std::memory_order_relaxed));
        stat_fallback_activations_ = static_cast<double>(fallback_activations_.load(std::memory_order_relaxed));
        stat_fallback_activations_per_cycle_ =
            static_cast<double>(fallback_activations_cycle_.load(std::memory_order_relaxed));
        stat_never_commanded_interfaces_ =
            static_cast<double>(never_commanded_interfaces_.load(std::memory_order_relaxed));
        stat_heartbeat_ = static_cast<double>(heartbeat_counter_);
        stat_setpoint_sequence_ = static_cast<double>(setpoint_sequence_counter_.current());
    }

    void BeckhoffADSHardwareInterface::register_transaction_statistics()
    {
        REGISTER_ROS2_CONTROL_INTROSPECTION("ads_read_rtt_ms", &stat_read_rtt_ms_);
        REGISTER_ROS2_CONTROL_INTROSPECTION("ads_write_rtt_ms", &stat_write_rtt_ms_);
        REGISTER_ROS2_CONTROL_INTROSPECTION("ads_write_buffer_age_ms", &stat_write_buffer_age_ms_);
        REGISTER_ROS2_CONTROL_INTROSPECTION("ads_write_handoff_latency_ms", &stat_write_handoff_latency_ms_);
        REGISTER_ROS2_CONTROL_INTROSPECTION("ads_read_transactions", &stat_read_transactions_);
        REGISTER_ROS2_CONTROL_INTROSPECTION("ads_write_transactions", &stat_write_transactions_);
        REGISTER_ROS2_CONTROL_INTROSPECTION("ads_write_coalesced", &stat_write_coalesced_);
        REGISTER_ROS2_CONTROL_INTROSPECTION("ads_read_failures", &stat_read_failures_);
        REGISTER_ROS2_CONTROL_INTROSPECTION("ads_write_failures", &stat_write_failures_);
        REGISTER_ROS2_CONTROL_INTROSPECTION("ads_fallback_activations", &stat_fallback_activations_);
        REGISTER_ROS2_CONTROL_INTROSPECTION("ads_fallback_activations_per_cycle", &stat_fallback_activations_per_cycle_);
        REGISTER_ROS2_CONTROL_INTROSPECTION("ads_never_commanded_interfaces", &stat_never_commanded_interfaces_);
        REGISTER_ROS2_CONTROL_INTROSPECTION("ads_heartbeat", &stat_heartbeat_);
        REGISTER_ROS2_CONTROL_INTROSPECTION("ads_setpoint_sequence", &stat_setpoint_sequence_);
        REGISTER_ROS2_CONTROL_INTROSPECTION("ads_read_sample_age_ms", &stat_read_sample_age_ms_);
    }

    void BeckhoffADSHardwareInterface::record_read_failure()
    {
        const std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        read_failures_total_.fetch_add(1, std::memory_order_relaxed);
        if (read_consecutive_failures_ == 0 || read_recovery_stable_since_)
        {
            read_outage_start_ = now;
        }
        ++read_consecutive_failures_;
        read_recovery_stable_since_.reset();
        if (now - read_outage_start_ >= read_outage_grace_)
        {
            read_hard_fault_.store(true, std::memory_order_release);
        }
    }

    void BeckhoffADSHardwareInterface::record_write_failure()
    {
        const std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        write_failures_total_.fetch_add(1, std::memory_order_relaxed);
        if (write_consecutive_failures_ == 0)
        {
            write_outage_start_ = now;
        }
        ++write_consecutive_failures_;
        if (now - write_outage_start_ >= comms_outage_grace_)
        {
            write_hard_fault_.store(true, std::memory_order_release);
        }
    }

    bool BeckhoffADSHardwareInterface::read_plc()
    {
        uint32_t bytes_read_from_plc = 0;
        long ads_sum_read_error;
        const auto read_rtt_start = std::chrono::steady_clock::now();
        next_read_time_ = read_rtt_start + std::chrono::nanoseconds(read_poll_period_ns_);
        ads_sum_read_error = ads_device_->ReadWriteReqEx2(
            ADSIGRP_SUMUP_READ,
            num_items_read_,
            ads_buffer_sum_read_response_.size(),
            ads_buffer_sum_read_response_.data(),
            ads_buffer_sum_read_request_.size(),
            ads_buffer_sum_read_request_.data(),
            &bytes_read_from_plc);
        read_rtt_ns_.store(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now() - read_rtt_start)
                .count(),
            std::memory_order_relaxed);
        read_transactions_total_.fetch_add(1, std::memory_order_relaxed);

        if (ads_sum_read_error != ADSERR_NOERR)
        {
            if (read_consecutive_failures_ == 0)
            {
                RCLCPP_ERROR(getLogger(),
                             "ADS Sum Read failed: 0x%lX (%s). Target %s, AMS NetId %s, port %u.",
                             ads_sum_read_error, adsErrorText(ads_sum_read_error),
                             plc_ip_address_.c_str(), plc_ams_net_id_str_.c_str(), plc_ams_port_);
            }
            else
            {
                RCLCPP_ERROR_THROTTLE(getLogger(), *logging_throttle_clock_, 1000,
                                      "ADS Sum Read still failing: 0x%lX (%s).",
                                      ads_sum_read_error, adsErrorText(ads_sum_read_error));
            }
            record_read_failure();
            return false;
        }

        if (bytes_read_from_plc != ads_buffer_sum_read_response_.size())
        {
            RCLCPP_ERROR_THROTTLE(getLogger(), *logging_throttle_clock_, 1000,
                                  "ADS Sum Read size mismatch. Expected %zu, Got %u.",
                                  ads_buffer_sum_read_response_.size(), bytes_read_from_plc);
            record_read_failure();
            return false;
        }

        if (read_staleness_timeout_ns_ > 0 &&
            std::chrono::steady_clock::now() - read_rtt_start > std::chrono::nanoseconds(read_staleness_timeout_ns_))
        {
            RCLCPP_ERROR(getLogger(), "ADS read response exceeded the configured freshness limit.");
            record_read_failure();
            read_hard_fault_.store(true, std::memory_order_release);
            return false;
        }

        // Per-item errors hold that item's last value; only every item failing is a link failure.
        size_t items_failed = 0;
        bool critical_item_missing = false;
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
                                     "ADS Sum Read operation corresponding to the state interface '%s' failed: 0x%X (%s).",
                                     read_instruction.state_interface_name.c_str(), item_error_code,
                                     adsErrorText(static_cast<long>(item_error_code)));
                ++items_failed;
                critical_item_missing = critical_item_missing || read_instruction.critical;
                const std::chrono::steady_clock::time_point item_now = std::chrono::steady_clock::now();
                if (!item_failure_start_[i])
                {
                    item_failure_start_[i] = item_now;
                }
                if (read_instruction.critical &&
                    item_now - *item_failure_start_[i] >= read_outage_grace_)
                {
                    RCLCPP_ERROR_THROTTLE(getLogger(), *logging_throttle_clock_, 1000,
                                          "Critical state interface '%s' has been failing beyond the grace "
                                          "window. Hard-faulting instead of freezing its feedback.",
                                          read_instruction.state_interface_name.c_str());
                    read_hard_fault_.store(true, std::memory_order_release);
                }
                continue;
            }
            item_failure_start_[i].reset();

            const uint8_t *ptr_plc_element_current = ads_buffer_sum_read_response_.data() + read_instruction.read_buffer_offset_data;
            last_decoded_values_[i] = decode_plc_element(read_instruction.plc_type, ptr_plc_element_current);
        }

        if (items_failed == ads_read_instructions_.size() && !ads_read_instructions_.empty())
        {
            record_read_failure();
        }
        else if (!critical_item_missing)
        {
            ReadSample &sample = read_sample_buffer_.writeSlot();
            sample.values = last_decoded_values_;
            sample.stamp = read_rtt_start;
            last_read_sample_time_ = read_rtt_start;
            sample.sequence = ++read_sample_sequence_;
            read_sample_buffer_.publish();
            read_samples_published_.fetch_add(1, std::memory_order_release);

            if (!(outage_latches_ && read_hard_fault_.load(std::memory_order_acquire)))
            {
                read_hard_fault_.store(false, std::memory_order_release);
            }
            // Declare recovery only after a stable period, so a flapping link logs once.
            if (read_consecutive_failures_ > 0)
            {
                const std::chrono::steady_clock::time_point now_steady = std::chrono::steady_clock::now();
                if (!read_recovery_stable_since_)
                {
                    read_recovery_stable_since_ = now_steady;
                }
                else if (now_steady - *read_recovery_stable_since_ >= RECOVERY_STABLE_PERIOD)
                {
                    const int64_t outage_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                                  *read_recovery_stable_since_ - read_outage_start_)
                                                  .count();
                    RCLCPP_INFO(getLogger(),
                                "ADS Sum Read recovered after %zu failed cycles (%.1f s).",
                                read_consecutive_failures_, static_cast<double>(outage_ms) / 1000.0);
                    read_consecutive_failures_ = 0;
                    read_recovery_stable_since_.reset();
                }
            }
        }
        return !critical_item_missing &&
               (items_failed < ads_read_instructions_.size() || ads_read_instructions_.empty());
    }

    hardware_interface::return_type BeckhoffADSHardwareInterface::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        if ((write_hard_fault_.load(std::memory_order_acquire) &&
             (outage_latches_ || io_stop_.load(std::memory_order_acquire))) ||
            read_hard_fault_.load(std::memory_order_acquire))
        {
            return hardware_interface::return_type::ERROR;
        }
        if (num_items_write_ == 0 || io_stop_.load(std::memory_order_acquire))
        {
            return hardware_interface::return_type::OK;
        }

        const uint32_t sequence = setpoint_sequence_counter_.next();
        const double stamp = utilities::monotonicSeconds(std::chrono::steady_clock::now());
        uint64_t fallbacks_this_cycle = 0;
        uint64_t never_commanded_this_cycle = 0;
        const char *first_fallback_interface = nullptr;
        bool provided = false;
        std::fill(write_values_.begin(), write_values_.end(), std::numeric_limits<double>::quiet_NaN());

        for (size_t i = 0; i < ads_write_instructions_.size(); ++i)
        {
            auto &instruction = ads_write_instructions_[i];
            double value = std::numeric_limits<double>::quiet_NaN();
            if (instruction.source == WriteValueSource::HEARTBEAT)
            {
                value = ++heartbeat_counter_;
            }
            else if (instruction.source == WriteValueSource::SETPOINT_SEQUENCE)
            {
                value = sequence;
            }
            else if (instruction.source == WriteValueSource::SETPOINT_TIMESTAMP)
            {
                value = stamp;
            }
            else
            {
                if (!get_command(instruction.command_handle, value, false)) { continue; }
                if (!std::isnan(value))
                {
                    // Consume only after clearing succeeds, so a contended handle is retried once.
                    if (!set_command(instruction.command_handle, std::numeric_limits<double>::quiet_NaN(), false))
                    {
                        continue;
                    }
                    instruction.has_been_commanded = true;
                    instruction.last_value = value;
                }
                else
                {
                    if (instruction.fallback == CommandFallback::NONE)
                    {
                        if (!instruction.has_been_commanded) { ++never_commanded_this_cycle; }
                        continue;
                    }
                    if (instruction.has_been_commanded)
                    {
                        ++fallbacks_this_cycle;
                        if (first_fallback_interface == nullptr)
                        {
                            first_fallback_interface = instruction.command_interface_name.c_str();
                        }
                    }
                    else
                    {
                        ++never_commanded_this_cycle;
                    }
                    value = instruction.last_value;
                    if (instruction.fallback == CommandFallback::ZERO)
                    {
                        value = 0.0;
                    }
                    else if (instruction.fallback == CommandFallback::MIRROR_STATE &&
                             instruction.fallback_state_handle)
                    {
                        std::ignore = get_state(instruction.fallback_state_handle, value, false);
                    }
                }
            }
            if (!std::isnan(value) && !std::isfinite(value))
            {
                RCLCPP_ERROR(getLogger(), "Non-finite ADS command on '%s'.", instruction.command_interface_name.c_str());
                write_hard_fault_.store(true, std::memory_order_release);
                {
                    std::lock_guard<std::mutex> lock(write_mutex_);
                    io_stop_.store(true, std::memory_order_release);
                }
                write_cv_.notify_all();
                return hardware_interface::return_type::ERROR;
            }
            if (instruction.source == WriteValueSource::CONTROLLER && !std::isnan(value))
            {
                instruction.last_value = value;
            }
            write_values_[i] = value;
            provided = provided || !std::isnan(value);
        }

        fallback_activations_.fetch_add(fallbacks_this_cycle, std::memory_order_relaxed);
        fallback_activations_cycle_.store(fallbacks_this_cycle, std::memory_order_relaxed);
        never_commanded_interfaces_.store(never_commanded_this_cycle, std::memory_order_relaxed);
        if (first_fallback_interface != nullptr)
        {
            RCLCPP_WARN_THROTTLE(getLogger(), *logging_throttle_clock_, 5000,
                                 "%lu command interfaces used their configured fallback ('%s' among them).",
                                 static_cast<unsigned long>(fallbacks_this_cycle), first_fallback_interface);
        }
        refresh_transaction_statistics();

        if (provided)
        {
            {
                std::lock_guard<std::mutex> lock(write_mutex_);
                if (write_pending_)
                {
                    write_coalesced_total_.fetch_add(1, std::memory_order_relaxed);
                }
                else
                {
                    write_pending_request_.handoff_stamp = std::chrono::steady_clock::now();
                }
                for (size_t i = 0; i < write_values_.size(); ++i)
                {
                    if (!std::isnan(write_values_[i])) { write_pending_request_.values[i] = write_values_[i]; }
                }
                write_pending_ = true;
            }
            write_cv_.notify_one();
        }
        return write_hard_fault_.load(std::memory_order_acquire)
                   ? hardware_interface::return_type::ERROR
                   : hardware_interface::return_type::OK;
    }

    bool BeckhoffADSHardwareInterface::write_plc(const PendingWrite &request, bool read_fresh)
    {
        bool no_retry = false;
        std::fill(write_layout_dirty_.begin(), write_layout_dirty_.end(), 0);
        std::fill(write_layout_provided_count_.begin(), write_layout_provided_count_.end(), 0);
        for (size_t i = 0; i < request.values.size(); ++i)
        {
            if (std::isnan(request.values[i])) { continue; }
            const auto &instruction = ads_write_instructions_[i];
            ++write_layout_provided_count_[instruction.layout_index];
            write_layout_dirty_[instruction.layout_index] = 1;
            no_retry = no_retry || instruction.fallback == CommandFallback::NONE;
        }
        bool needs_readback = false;
        for (size_t i = 0; i < num_items_write_; ++i)
        {
            const auto &layout = ads_item_layouts_write_[i];
            needs_readback = needs_readback || (write_layout_dirty_[i] &&
                write_layout_provided_count_[i] < layout.num_elements && layout.readback_data_offset.has_value());
        }
        if (needs_readback && !read_fresh && !read_plc())
        {
            // No write was sent. Preserve its unsent fields while newer commands take priority.
            next_read_time_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(10);
            std::lock_guard<std::mutex> lock(write_mutex_);
            if (!io_stop_.load(std::memory_order_acquire) &&
                !read_hard_fault_.load(std::memory_order_acquire) &&
                !write_hard_fault_.load(std::memory_order_acquire))
            {
                for (size_t i = 0; i < request.values.size(); ++i)
                {
                    if (std::isnan(write_pending_request_.values[i]))
                    {
                        write_pending_request_.values[i] = request.values[i];
                    }
                }
                write_pending_request_.handoff_stamp = write_pending_
                    ? std::min(write_pending_request_.handoff_stamp, request.handoff_stamp)
                    : request.handoff_stamp;
                write_pending_ = true;
            }
            return false;
        }
        for (size_t i = 0; i < num_items_write_; ++i)
        {
            const auto &layout = ads_item_layouts_write_[i];
            if (!write_layout_dirty_[i] || write_layout_provided_count_[i] == layout.num_elements) { continue; }
            if (!layout.readback_data_offset)
            {
                // Streamed arrays wait for all initial fields. A sparse one-shot requires readback.
                write_layout_dirty_[i] = 0;
                const bool one_shot = std::any_of(layout.fallback_policies_.begin(), layout.fallback_policies_.end(),
                                                  [](const auto &policy) { return policy.second == CommandFallback::NONE; });
                if (one_shot)
                {
                    RCLCPP_ERROR(getLogger(), "Partial write to '%s' requires PLC readback.", layout.plc_name_symbolic.c_str());
                    write_hard_fault_.store(true, std::memory_order_release);
                    io_stop_.store(true, std::memory_order_release);
                    return false;
                }
                continue;
            }
            uint32_t error;
            memcpy(&error, ads_buffer_sum_read_response_.data() + *layout.readback_error_offset, sizeof(error));
            if (error != ADSERR_NOERR)
            {
                RCLCPP_ERROR(getLogger(), "Partial write to '%s' has invalid PLC readback.", layout.plc_name_symbolic.c_str());
                write_hard_fault_.store(true, std::memory_order_release);
                io_stop_.store(true, std::memory_order_release);
                return false;
            }
            memcpy(ads_buffer_sum_write_request_.data() + layout.offset_in_write_request_data,
                   ads_buffer_sum_read_response_.data() + *layout.readback_data_offset,
                   layout.num_elements * layout.plc_element_byte_size);
        }
        for (size_t i = 0; i < request.values.size(); ++i)
        {
            const auto &instruction = ads_write_instructions_[i];
            if (std::isnan(request.values[i]) || !write_layout_dirty_[instruction.layout_index]) { continue; }
            if (!encode_plc_element(instruction.plc_type, request.values[i],
                                    ads_buffer_sum_write_request_.data() + instruction.write_buffer_offset_data))
            {
                write_hard_fault_.store(true, std::memory_order_release);
                io_stop_.store(true, std::memory_order_release);
                return false;
            }
        }
        utilities::compactSumWriteRequest(ads_buffer_sum_write_request_, write_item_spans_,
                                          write_layout_dirty_, ads_buffer_sum_write_compact_, compact_layout_indices_);
        if (compact_layout_indices_.empty()) { return true; }
        ads_buffer_sum_write_response_.resize(compact_layout_indices_.size() * sizeof(uint32_t));
        const auto send_start = std::chrono::steady_clock::now();
        write_buffer_age_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                       send_start - request.handoff_stamp).count(), std::memory_order_relaxed);
        uint32_t bytes_read = 0;
        // A mandatory read can block while lifecycle shutdown or a fault revokes writes.
        if (io_stop_.load(std::memory_order_acquire) ||
            read_hard_fault_.load(std::memory_order_acquire) ||
            (outage_latches_ && write_hard_fault_.load(std::memory_order_acquire)))
        {
            return false;
        }
        const long error = ads_device_->ReadWriteReqEx2(
            ADSIGRP_SUMUP_WRITE, compact_layout_indices_.size(), ads_buffer_sum_write_response_.size(),
            ads_buffer_sum_write_response_.data(), ads_buffer_sum_write_compact_.size(),
            ads_buffer_sum_write_compact_.data(), &bytes_read);
        const auto completed = std::chrono::steady_clock::now();
        write_rtt_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(completed - send_start).count(),
                            std::memory_order_relaxed);
        write_handoff_latency_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                           completed - request.handoff_stamp).count(), std::memory_order_relaxed);
        write_transactions_total_.fetch_add(1, std::memory_order_relaxed);
        bool failed = error != ADSERR_NOERR || bytes_read != ads_buffer_sum_write_response_.size();
        if (!failed)
        {
            for (size_t i = 0; i < compact_layout_indices_.size(); ++i)
            {
                uint32_t item_error;
                memcpy(&item_error, ads_buffer_sum_write_response_.data() + i * sizeof(item_error), sizeof(item_error));
                if (item_error != ADSERR_NOERR)
                {
                    RCLCPP_ERROR(getLogger(), "ADS write '%s' failed: 0x%X (%s).",
                                 ads_item_layouts_write_[compact_layout_indices_[i]].plc_name_symbolic.c_str(),
                                 item_error, adsErrorText(item_error));
                    failed = true;
                }
            }
        }
        if (failed)
        {
            record_write_failure();
            if (no_retry)
            {
                // An absent reply cannot prove that a one-shot command was never executed.
                write_hard_fault_.store(true, std::memory_order_release);
                io_stop_.store(true, std::memory_order_release);
            }
            RCLCPP_ERROR_THROTTLE(getLogger(), *logging_throttle_clock_, 1000,
                                  "ADS write failed: 0x%lX (%s), response %u/%zu bytes.",
                                  error, adsErrorText(error), bytes_read, ads_buffer_sum_write_response_.size());
            return false;
        }
        if (!(outage_latches_ && write_hard_fault_.load(std::memory_order_acquire)))
        {
            write_hard_fault_.store(false, std::memory_order_release);
        }
        write_consecutive_failures_ = 0;
        return true;
    }

    void BeckhoffADSHardwareInterface::io_loop()
    {
        PendingWrite request;
        request.values.resize(ads_write_instructions_.size(), std::numeric_limits<double>::quiet_NaN());
        bool read_available = num_items_read_ == 0;
        try
        {
            while (!io_stop_.load(std::memory_order_acquire))
            {
                if (num_items_read_ == 0)
                {
                    std::unique_lock<std::mutex> lock(write_mutex_);
                    write_cv_.wait(lock, [this] { return io_stop_.load(std::memory_order_acquire) || write_pending_; });
                }
                if (io_stop_.load(std::memory_order_acquire)) { break; }
                bool read_fresh = false;
                if (num_items_read_ > 0 && std::chrono::steady_clock::now() >= next_read_time_)
                {
                    read_fresh = read_plc();
                    read_available = read_fresh;
                    if (!read_available)
                    {
                        next_read_time_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(10);
                    }
                }
                if (read_available && num_items_read_ > 0 && read_staleness_timeout_ns_ > 0 &&
                    std::chrono::steady_clock::now() - last_read_sample_time_ >
                        std::chrono::nanoseconds(read_staleness_timeout_ns_))
                {
                    read_hard_fault_.store(true, std::memory_order_release);
                    read_available = false;
                    next_read_time_ = std::chrono::steady_clock::now();
                }
                if (outage_latches_ && (read_hard_fault_.load(std::memory_order_acquire) ||
                                       write_hard_fault_.load(std::memory_order_acquire))) { break; }
                bool send = false;
                if (read_available)
                {
                    std::lock_guard<std::mutex> lock(write_mutex_);
                    if (io_stop_.load(std::memory_order_acquire)) { break; }
                    send = write_pending_;
                    if (send)
                    {
                        request.values.swap(write_pending_request_.values);
                        request.handoff_stamp = write_pending_request_.handoff_stamp;
                        std::fill(write_pending_request_.values.begin(), write_pending_request_.values.end(),
                                  std::numeric_limits<double>::quiet_NaN());
                        write_pending_ = false;
                    }
                }
                // A mandatory pre-write read can invalidate the earlier successful read.
                if (send && !write_plc(request, read_fresh)) { read_available = false; }
                if (num_items_read_ > 0)
                {
                    std::unique_lock<std::mutex> lock(write_mutex_);
                    write_cv_.wait_until(lock, next_read_time_, [this, read_available] {
                        return io_stop_.load(std::memory_order_acquire) || (read_available && write_pending_);
                    });
                }
            }
        }
        catch (const std::exception &error)
        {
            RCLCPP_ERROR(getLogger(), "ADS I/O failed: %s. Reconfigure before resuming.", error.what());
            read_hard_fault_.store(true, std::memory_order_release);
            write_hard_fault_.store(true, std::memory_order_release);
        }
    }

    void BeckhoffADSHardwareInterface::start_io()
    {
        if (io_thread_.joinable()) { return; }
        {
            std::lock_guard<std::mutex> lock(write_mutex_);
            write_pending_ = false;
            std::fill(write_pending_request_.values.begin(), write_pending_request_.values.end(),
                      std::numeric_limits<double>::quiet_NaN());
        }
        read_consecutive_failures_ = 0;
        write_consecutive_failures_ = 0;
        read_recovery_stable_since_.reset();
        next_read_time_ = {};
        last_read_sample_time_ = {};
        std::fill(item_failure_start_.begin(), item_failure_start_.end(), std::nullopt);
        std::fill(last_decoded_values_.begin(), last_decoded_values_.end(), std::numeric_limits<double>::quiet_NaN());
        read_sample_buffer_.initialiseSlots([](ReadSample &sample) {
            std::fill(sample.values.begin(), sample.values.end(), std::numeric_limits<double>::quiet_NaN());
            sample.stamp = {};
            sample.sequence = 0;
        });
        io_stop_.store(false, std::memory_order_release);
        if (num_items_read_ > 0 || num_items_write_ > 0)
        {
            io_thread_ = std::thread(&BeckhoffADSHardwareInterface::io_loop, this);
            apply_io_thread_scheduling(io_thread_, "ADS I/O");
        }
    }

    void BeckhoffADSHardwareInterface::apply_io_thread_scheduling(std::thread &thread, const char *thread_name)
    {
        if (io_thread_scheduling_.policy != utilities::SchedulingPolicy::INHERIT)
        {
            int native_policy = SCHED_OTHER;
            if (io_thread_scheduling_.policy == utilities::SchedulingPolicy::FIFO)
            {
                native_policy = SCHED_FIFO;
            }
            else if (io_thread_scheduling_.policy == utilities::SchedulingPolicy::ROUND_ROBIN)
            {
                native_policy = SCHED_RR;
            }
            sched_param scheduling_parameters{};
            scheduling_parameters.sched_priority = io_thread_scheduling_.priority;
            const int scheduling_error =
                pthread_setschedparam(thread.native_handle(), native_policy, &scheduling_parameters);
            if (scheduling_error != 0)
            {
                RCLCPP_WARN(getLogger(),
                            "Could not set the %s thread scheduling (policy %d, priority %d): %s. "
                            "The thread keeps normal scheduling; grant the process CAP_SYS_NICE or an "
                            "rtprio limit, or set io_thread_scheduling_policy to inherit.",
                            thread_name, native_policy, io_thread_scheduling_.priority,
                            std::strerror(scheduling_error));
            }
            else
            {
                RCLCPP_INFO(getLogger(), "%s thread scheduling set: policy %d, priority %d.",
                            thread_name, native_policy, io_thread_scheduling_.priority);
            }
        }

        if (!io_thread_scheduling_.cpu_affinity.empty())
        {
            cpu_set_t cpu_set;
            CPU_ZERO(&cpu_set);
            for (const unsigned int cpu : io_thread_scheduling_.cpu_affinity)
            {
                CPU_SET(cpu, &cpu_set);
            }
            const int affinity_error =
                pthread_setaffinity_np(thread.native_handle(), sizeof(cpu_set), &cpu_set);
            if (affinity_error != 0)
            {
                RCLCPP_WARN(getLogger(), "Could not set the %s thread CPU affinity: %s.",
                            thread_name, std::strerror(affinity_error));
            }
        }
    }

    void BeckhoffADSHardwareInterface::stop_io()
    {
        {
            std::lock_guard<std::mutex> lock(write_mutex_);
            io_stop_.store(true, std::memory_order_release);
        }
        write_cv_.notify_all();
        if (io_thread_.joinable()) { io_thread_.join(); }
    }

    hardware_interface::CallbackReturn BeckhoffADSHardwareInterface::on_shutdown(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        teardown_ads_device();
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn BeckhoffADSHardwareInterface::on_error(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_ERROR(getLogger(), "ADS component entering the error state. Dropping the link.");
        teardown_ads_device();
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    void BeckhoffADSHardwareInterface::teardown_ads_device()
    {
        RCLCPP_INFO(getLogger(), "Releasing ADS resources...");
        // Stop the I/O owner before touching its device.
        // Safe to call even if on_deactivate already joined it.
        stop_io();
        // Release symbol handles before the device: their deleters call DeleteSymbolHandle
        // through the device that issued them. Skipping this is what segfaulted on Ctrl-C.
        release_ads_handles();
        ads_device_.reset();
        RCLCPP_INFO(getLogger(), "ADS resources released.");
    }

    void BeckhoffADSHardwareInterface::release_ads_handles()
    {
        // reset() destroys the held AdsHandle, whose deleter releases the PLC symbol handle via
        // the device that issued it. Callers guarantee the device is still valid here.
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
            const utilities::AmsPortParseResult port_parse = utilities::parseAmsPort(params.at("plc_ams_port"));
            if (!port_parse.valid)
            {
                RCLCPP_FATAL(getLogger(), "\tInvalid 'plc_ams_port' '%s': %s.",
                             params.at("plc_ams_port").c_str(), port_parse.error.c_str());
                return false;
            }
            uint16_t plc_ams_port = port_parse.port;
            plc_ip_address_ = plc_ip;
            plc_ams_net_id_str_ = plc_ams_net_id_str;
            plc_ams_port_ = plc_ams_port;

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

            // Bounds how long a blocking ADS call can stall stop_io; 0 keeps the library default.
            uint32_t request_timeout_ms = 100;
            const auto timeout_it = params.find("ads_request_timeout_ms");
            if (timeout_it != params.end())
            {
                try
                {
                    const double ms = std::stod(timeout_it->second);
                    request_timeout_ms = (std::isfinite(ms) && ms >= 0.0) ? static_cast<uint32_t>(ms) : 100;
                }
                catch (const std::exception &ex)
                {
                    RCLCPP_WARN(getLogger(), "Invalid ads_request_timeout_ms: %s. Using default 100 ms.", ex.what());
                }
            }
            if (request_timeout_ms > 0)
            {
                ads_device_->SetTimeout(request_timeout_ms);
            }
            RCLCPP_INFO(getLogger(), "\tTimeout is: %u", ads_device_->GetTimeout());

            RCLCPP_INFO(getLogger(), "\tADS device configured for PLC: %s, Port: %u", plc_ip.c_str(), plc_ams_port);
            RCLCPP_INFO(getLogger(), "\tPLC AMS NetID: %s. I/O owner on local AMS port %ld.",
                        plc_ams_net_id_str.c_str(), ads_device_->GetLocalPort());

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
            RCLCPP_FATAL(getLogger(),
                         "\tADS Exception during connection to %s (AMS NetId %s): %s (error 0x%lX: %s)",
                         plc_ip_address_.c_str(), plc_ams_net_id_str_.c_str(),
                         ex.what(), static_cast<long>(ex.errorCode), adsErrorText(static_cast<long>(ex.errorCode)));
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
        std::string type_str = utilities::toUpperCopy(type_str_param);

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
