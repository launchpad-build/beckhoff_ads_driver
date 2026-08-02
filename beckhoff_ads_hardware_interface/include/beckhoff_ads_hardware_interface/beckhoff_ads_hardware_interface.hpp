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

  // What to send for a command interface on a cycle where no controller wrote one.
  enum class CommandFallback
  {
    HOLD_LAST,    // keep whatever was packed on the last cycle that carried a command
    MIRROR_STATE, // send the state interface sharing this PLC symbol, so the target tracks reality
    ZERO,         // send zero; the right answer for a velocity or effort command
  };

  // Describes each PLC item (array or single variable) for polling via sum commands.
  struct ADSDataLayout
  {
    // Configured from yaml
    std::string plc_name_symbolic; // e.g., "MAIN.Joint_Pos_State". Used to get the handle.
    PLCType plc_type;
    std::optional<AdsHandle> ads_handle_owner; // RAII owner; releases SYM_HNDBYNAME on destruction. Must outlive any use of ads_handle.
    uint32_t ads_handle; // PLC Handle value cached for the sum-request headers. Mirrors **ads_handle_owner.

    size_t num_elements;          // 6 for LREAL[6], 1 for single LREAL/BOOL etc.
    size_t plc_element_byte_size; // byte size of ONE element on PLC (e.g., 8 for LREAL, 1 for BOOL).

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

    // Per-interface command_fallback, parsed from the interface parameters.
    std::map<std::string, CommandFallback> fallback_policies_;

    // True when every interface on this symbol declared optional="true". An optional symbol
    // the PLC does not have is dropped with a warning; a required one fails configure.
    bool optional = false;
    bool handle_resolved = false;
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
    CommandFallback fallback = CommandFallback::HOLD_LAST;
    bool is_heartbeat = false; // value comes from the interface's own counter, not a controller
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

    /**
     * @brief Decodes well-known ADS error codes into a short hint for log output
     *
     * @param error_code ADS return code from the AdsLib
     * @returns Static text describing the code, or a generic fallback
     */
    static const char *adsErrorText(long error_code);

    /**
     * @brief Parses a command_fallback interface parameter into its enum
     *
     * @param policy_str Parameter text: hold_last, mirror_state or zero
     * @param interface_name Interface the parameter belongs to, for the warning on a bad value
     * @returns The matching policy, or HOLD_LAST when the text is empty or unrecognised
     */
    CommandFallback parseCommandFallback(const std::string &policy_str, const std::string &interface_name);

    /**
     * @brief Records a failed SUM-read round-trip for outage and recovery logs
     *
     * Stamps the outage start on the first failure and resets recovery tracking.
     * Reader thread only.
     */
    void record_read_failure();

    /**
     * @brief Records a failed SUM-write round-trip for outage and recovery logs
     *
     * Stamps the outage start on the first failure and resets recovery tracking.
     * Writer thread only.
     */
    void record_write_failure();

    // Synthetic interface name for the link heartbeat. Never exported to ros2_control; it
    // only tags the write instruction whose value the interface generates itself.
    static constexpr const char *HEARTBEAT_INTERFACE_NAME = "__ads_link_heartbeat";

    std::string heartbeat_symbol_;        // PLC symbol to beat on; empty disables the heartbeat
    uint32_t heartbeat_counter_{0};       // advanced in write(), so control-loop thread only

    // Connection target details kept for error logs (populated in configure_ads_device).
    std::string plc_ip_address_;
    std::string plc_ams_net_id_str_;
    uint16_t plc_ams_port_{0};

    // A link must stay good this long before an outage is declared over.
    static constexpr std::chrono::seconds RECOVERY_STABLE_PERIOD{1};

    // ========= PLC ==============================

    // PLC Type and Size Helpers
    PLCType strToPlcType(const std::string &type_str);
    size_t plcTypeByteSize(PLCType type_enum);

    // ADS Communication objects
    std::unique_ptr<AdsDevice> ads_device_; // Manages the route/connection to the PLC
    bool configure_ads_device();

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

    // Cycles on which a command interface carried no command and its fallback applied.
    std::atomic<uint64_t> fallback_activations_{0};

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
    // Consecutive failed SUM-write round-trips, for outage and recovery logs. Writer thread only.
    size_t write_consecutive_failures_ = 0;
    std::chrono::steady_clock::time_point write_outage_start_;
    std::optional<std::chrono::steady_clock::time_point> write_recovery_stable_since_;

    // Reader thread: owns the SUM-read round-trip. Decodes each sample into
    // polling_read_cache_; read() loads from it without blocking.
    void reader_loop();
    std::thread read_thread_;
    std::atomic<bool> read_stop_{false};
    std::atomic<bool> read_comms_ok_{true};         // last read transaction health, surfaced by read()
    std::deque<std::atomic<double>> polling_read_cache_; // one slot per read instruction; deque keeps addresses stable
    long long read_poll_period_ns_ = 0;             // optional pacing between SUM reads; 0 = unpaced
    // Consecutive failed SUM-read round-trips, for outage and recovery logs. Reader thread only.
    size_t read_consecutive_failures_ = 0;
    std::chrono::steady_clock::time_point read_outage_start_;
    std::optional<std::chrono::steady_clock::time_point> read_recovery_stable_since_;
  };

} // namespace beckhoff_ads_hardware_interface

#endif // beckhoff_ads_hardware_interface__BECKHOFF_SYSTEM_HPP_