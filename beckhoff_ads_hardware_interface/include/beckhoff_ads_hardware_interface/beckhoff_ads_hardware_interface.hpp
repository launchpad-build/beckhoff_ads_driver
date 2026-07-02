// Copyright (c) 2025, b-robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

// Author: Nikola Banovic
// Contributor: Hajar Bartakh

#ifndef beckhoff_ads_hardware_interface__BECKHOFF_SYSTEM_HPP_
#define beckhoff_ads_hardware_interface__BECKHOFF_SYSTEM_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>
#include <limits>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "ads/AdsLib.h"
#include "ads/AdsDevice.h"
#include "ads/AdsNotificationOOI.h"
namespace beckhoff_ads_hardware_interface
{

  enum class PLCType
  {
    // Using enum instead of raw strings for faster iterations in read/write
    UNKNOWN,
    BOOL,
    LREAL,
    REAL,
    UDINT,
    DINT,
    INT,
    UINT,
    SINT,
    USINT,
    BYTE,
    STRING,
  };

  // Describes each PLC item (array or single variable) for polling via sum commands.
  struct ADSDataLayout
  {
    // Configured from yaml
    std::string plc_name_symbolic; // e.g., "MAIN.Joint_Pos_State". Used to get the handle.
    PLCType plc_type;
    std::optional<AdsHandle> ads_handle_owner; // RAII owner; releases SYM_HNDBYNAME on destruction. Must outlive any use of ads_handle.
    uint32_t ads_handle; // PLC handle value cached for the sum-request headers; copied from ads_handle_owner.

    size_t num_elements;          // 6 for LREAL[6], 1 for single LREAL/BOOL etc.
    size_t plc_element_byte_size; // byte size of ONE element on PLC (e.g., 8 for LREAL, 1 for BOOL).

    // ADS device-notification parameters (read path only; populated from URDF).
    // nCycleTime / nMaxDelay are expressed in ADS 100 ns ticks (URDF gives milliseconds).
    uint32_t notify_trans_mode = ADSTRANS_SERVERONCHA; // ADSTRANS_SERVERONCHA or ADSTRANS_SERVERCYCLE
    uint32_t notify_cycle_100ns = 10u * 10000u;        // default 10 ms change-check / push interval
    uint32_t notify_max_delay_100ns = 0u;              // 0 = deliver each sample immediately (no batching)

    // If we have an identically named command and state interface, in case there are no new commands to be sent to the robot, we want to use the value read in the state interface for the next request.
    // for mapping the ros2 state interfaces names to the corresponding command interfaces names <command_interface_name, state_interface_name>
    std::map<std::string, std::string> state_command_interfaces_map_;

    // for unpacking sum read response  [Err1_ULONG,...,ErrN_ULONG | Data1_bytes,...,DataN_bytes]
    size_t offset_in_read_response_error; // Byte offset where this item's ULONG error code starts.
    size_t offset_in_read_response_data;  // Byte offset where this item's data starts.

    // for packing sum write response [ADS_ITEM_REQ_HEADER_1,...,ADS_ITEM_REQ_HEADER_N | Data1_bytes,...,DataN_bytes]
    size_t offset_in_write_request_data; // Byte offset where this item's data starts.

    // For interfaces targeting the same PLC symbol, store all their names with their corresponding index inside a map. This will be useful when calling thr ROS2 set_state and set_command functions.
    std::map<size_t, std::string> ros2_interfaces_;
  };

  // Packed header for sum read/write request item headers
  typedef struct
  {
    uint32_t indexGroup;   // ADSIGRP_SYM_VALBYHND
    uint32_t indexOffset;  // The ADS Handle
    uint32_t NumBytesData; // total num of bytes in this data section
  } ADS_ITEM_REQ_HEADER;

  struct ReadInstruction
  {
    size_t read_buffer_offset_error_code;
    size_t read_buffer_offset_data;
    PLCType plc_type;
    std::string state_interface_name;
  };

  struct WriteInstruction
  {
    size_t write_buffer_offset_data;
    PLCType plc_type;
    std::string command_interface_name;
    std::string fallback_state_interface_name; // The state interface name corresponding to the current command interface name
  };

  // Notification read path: maps a state interface to the lock-free cache slot that the
  // background ADS notification callback writes into. read() simply loads the latest value.
  struct NotifReadTarget
  {
    std::string state_interface_name;
    std::atomic<double> *cache;
    std::atomic<long long> *last_update_ns;
    bool cyclic;
  };

  class BeckhoffADSHardwareInterface : public hardware_interface::SystemInterface
  {
  public:
    ~BeckhoffADSHardwareInterface() override; // joins the I/O threads if a lifecycle shutdown did not

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams &params) override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type write(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    rclcpp::Logger getLogger() { return rclcpp::get_logger("BeckhoffADSHardwareInterface"); }
    std::shared_ptr<rclcpp::Clock> logging_throttle_clock_;

    // ========= PLC ==============================

    // PLC Type and Size Helpers
    PLCType strToPlcType(const std::string &type_str);
    size_t plcTypeByteSize(PLCType type_enum);

    // ADS Communication objects
    std::unique_ptr<AdsDevice> ads_device_; // Manages the route/connection to the PLC
    bool configure_ads_device();

    // Read strategy: false = synchronous SUM read every cycle (default),
    // true = PLC-pushed device notifications cached for a non-blocking read().
    bool read_via_notifications_ = false;
    bool setup_notifications();
    void teardown_notifications();
    void release_notification_slots();

    long long notif_staleness_ns_ = 0; // cyclic-notification staleness timeout (ns); 0 disables

    // Releases every cached PLC symbol handle (ADSDataLayout::ads_handle_owner). Each handle's
    // deleter calls DeleteSymbolHandle through ads_device_, so this MUST run while ads_device_
    // is still alive, i.e. before resetting/replacing it. Otherwise the deleters dereference a
    // freed device and segfault (seen on Ctrl-C teardown).
    void release_ads_handles();

    // Metadata (populated in on interface export)
    // Describes each variable on the PLC
    std::vector<ADSDataLayout> ads_item_layouts_read_;
    std::vector<ADSDataLayout> ads_item_layouts_write_;
    void ads_read_layout_configure();
    void ads_write_layout_configure();
    bool build_sum_read_buffers();
    bool build_sum_write_buffers();

    // ADS Sum Command Buffers
    // SENT: List of ADS_ITEM_REQ_HEADER structs
    // RECEIVED: List of [Err1_ULONG,...,ErrN_ULONG | Data1_bytes,...,DataN_bytes]
    std::vector<uint8_t> ads_buffer_sum_read_request_;
    std::vector<uint8_t> ads_buffer_sum_read_response_;
    size_t num_items_read_ = 0;

    // SENT: List of [ADS_ITEM_REQ_HEADER_1,...,ADS_ITEM_REQ_HEADER_N | Data1_bytes,...,DataN_bytes]
    // RECEIVED: List of ErrCode_ULONGs
    std::vector<uint8_t> ads_buffer_sum_write_request_;
    std::vector<uint8_t> ads_buffer_sum_write_response_;
    size_t num_items_write_ = 0;

    std::vector<ReadInstruction> ads_read_instructions_;
    std::vector<WriteInstruction> ads_write_instructions_;

    // ===== Background ADS I/O threads ==========================================
    // The blocking ADS SUM read/write round-trips are moved off the control loop.
    // read()/write() only touch lock-free caches and a coalescing buffer, so the
    // control-loop cycle time no longer depends on PLC/network latency.

    void start_io_threads(); // spawns the writer and reader threads
    void stop_io_threads();  // signals and joins both worker threads; safe to call when idle

    // Serialises the reader's and writer's ADS round-trips. ads_device_ owns a single local AMS
    // port, and the underlying library allows only one in-flight request per port. Without this
    // the writer's port reservation collides with the reader's in-flight read and fails. Held
    // only around each ReadWriteReqEx2 call, never the control loop, so read()/write() stay
    // non-blocking.
    std::mutex ads_io_mutex_;

    // Writer thread: owns the SUM-write round-trip. write() marshals the latest command
    // buffer, hands it over here, and returns. Only the newest buffer is sent (coalescing).
    void writer_loop();
    std::thread write_thread_;
    std::mutex write_mutex_;
    std::condition_variable write_cv_;
    std::vector<uint8_t> write_pending_request_; // latest packed SUM-write request awaiting send
    bool write_pending_ = false;                 // a fresh buffer is waiting (guarded by write_mutex_)
    bool write_stop_ = false;                     // stop request (guarded by write_mutex_)
    std::atomic<bool> write_comms_ok_{true};      // last write transaction health, surfaced by write()

    // Reader thread: owns the SUM-read round-trip. Decodes each sample into
    // polling_read_cache_; read() loads from it without blocking.
    void reader_loop();
    std::thread read_thread_;
    std::atomic<bool> read_stop_{false};
    std::atomic<bool> read_comms_ok_{true};         // last read transaction health, surfaced by read()
    std::deque<std::atomic<double>> polling_read_cache_; // one slot per read instruction; deque keeps addresses stable
    long long read_poll_period_ns_ = 0;             // optional pacing between SUM reads; 0 = unpaced

    // Notification read path. read_notifications_ owns the live ADS subscriptions; each
    // AdsNotification's destructor calls DeleteNotification, which dereferences ads_device_,
    // so this MUST be declared after ads_device_ (destroyed first) and cleared before
    // ads_device_.reset() in on_shutdown/on_deactivate.
    std::vector<AdsNotification> read_notifications_;
    std::vector<NotifReadTarget> notif_read_targets_;
    std::vector<uint32_t> notif_slot_indices_;
  };

} // namespace beckhoff_ads_hardware_interface

#endif // beckhoff_ads_hardware_interface__BECKHOFF_SYSTEM_HPP_