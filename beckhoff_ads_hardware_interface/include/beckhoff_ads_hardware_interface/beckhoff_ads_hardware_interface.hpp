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
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>
#include <limits>

#include "hardware_interface/introspection.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "ads/AdsLib.h"
#include "ads/AdsDevice.h"

#include "beckhoff_ads_hardware_interface/ads_interface_utilities.hpp"
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

    // Per state interface: a critical item failing beyond the grace window is a hard fault
    // instead of freezing silently. Joint position and velocity default to critical; the
    // critical interface parameter overrides either way.
    std::map<std::string, bool> critical_policies_;

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
    // Resolved once at configure so read() stays non-blocking on the control loop.
    hardware_interface::StateInterface::SharedPtr state_handle;
    // A critical item whose per-item read keeps failing beyond the grace window hard-faults
    // the component; a non-critical one holds its last value and logs, as before.
    bool critical = false;
  };

  struct WriteInstruction
  {
    size_t write_buffer_offset_data;
    PLCType plc_type;
    std::string command_interface_name;
    std::string fallback_state_interface_name; // The state interface name corresponding to the current command interface name
    CommandFallback fallback = CommandFallback::HOLD_LAST;
    bool is_heartbeat = false; // value comes from the interface's own counter, not a controller
    bool has_been_commanded = false;
    // An unseeded field keeps its whole item out of the transmitted request.
    bool seeded = false;
    size_t layout_index = 0; // index of the owning layout in ads_item_layouts_write_
    // Resolved once at configure so write() stays non-blocking. Null for the heartbeat.
    hardware_interface::CommandInterface::SharedPtr command_handle;
    hardware_interface::StateInterface::SharedPtr fallback_state_handle;
  };

  // One whole decoded SUM-read sample.
  struct ReadSample
  {
    std::vector<double> values; // one per read instruction, in instruction order
    std::chrono::steady_clock::time_point stamp{};
    uint64_t sequence = 0; // 0 means no sample has been published yet
  };

  // The latest packed sum-write request handed from write() to the writer thread.
  struct PendingWrite
  {
    std::vector<uint8_t> buffer;
    std::vector<size_t> layout_indices; // original layout index of each item in the buffer
    size_t num_items = 0;
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

    // Entered straight from an ERROR in read()/write(), with no on_deactivate first.
    hardware_interface::CallbackReturn on_error(
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
     * @brief Tells whether a state interface declared an initial_value in its description
     *
     * @param interface_name The full state interface name
     * @returns True when the URDF declared a non-empty initial_value for it
     */
    bool stateInterfaceHasDeclaredInitialValue(const std::string &interface_name) const;

    /**
     * @brief Pins the state interfaces of dropped optional symbols to a defined value
     *
     * @param dropped_interfaces State interface names whose PLC symbol was dropped
     */
    void settle_dropped_state_interfaces(const std::vector<std::string> &dropped_interfaces);

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

    // Synthetic heartbeat name, never exported to ros2_control.
    static constexpr const char *HEARTBEAT_INTERFACE_NAME = "__ads_link_heartbeat";

    std::string heartbeat_symbol_;        // PLC symbol to beat on; empty disables the heartbeat
    uint32_t heartbeat_counter_{0};       // advanced in write(), so control-loop thread only

    // Connection target details kept for error logs (populated in configure_ads_device).
    std::string plc_ip_address_;
    std::string plc_ams_net_id_str_;
    uint16_t plc_ams_port_{0};

    // A link must stay good this long before an outage is declared over.
    static constexpr std::chrono::seconds RECOVERY_STABLE_PERIOD{1};

    // A comms outage shorter than this is ridden out on the last cached values; only once it
    // outlives the window does read()/write() surface an error and let the controller manager
    // tear the stack down. Overridable via the comms_outage_grace_ms hardware parameter. The
    // default suits a motion stream: it matches the PLC's own ROS link watchdog, so ROS
    // notices a dead link no later than the machine stops. Telemetry stacks can raise it.
    std::chrono::milliseconds comms_outage_grace_{100};

    // With latching (the default) a hard fault survives link recovery, so the component
    // faults and is restarted deliberately instead of resuming the stream with a silent
    // position step of up to velocity times outage. outage_behaviour=resume restores the
    // self-clearing behaviour, which suits hold-last telemetry and GPIO stacks.
    bool outage_latches_{true};

    // read() returns an error once the published sample is older than this, so a controller
    // cannot close a loop on frozen feedback while everything reports healthy. Overridable
    // via read_staleness_timeout_ms; 0 disables. Raised to three read poll periods when
    // paced reading is slower than the configured value.
    long long read_staleness_timeout_ns_{100000000};

    // How long on_activate may wait for the first good sample before failing activation,
    // so early read() calls never publish NaN into controllers that sample state when they
    // activate. Overridable via activation_first_sample_timeout_ms; 0 skips the wait.
    std::chrono::milliseconds activation_first_sample_timeout_{1000};
    std::atomic<uint64_t> read_samples_published_{0}; // bumped by the reader on every publish

    // ========= PLC ==============================

    // PLC Type and Size Helpers
    PLCType strToPlcType(const std::string &type_str);
    size_t plcTypeByteSize(PLCType type_enum);

    // ADS Communication objects
    // Reset or replace only after stop_io_threads() has joined both I/O threads.
    // Two local AMS ports on one net id, so the reader and writer never contend for a port.
    std::unique_ptr<AdsDevice> ads_read_device_;  // owned by the reader thread's round-trips
    std::unique_ptr<AdsDevice> ads_write_device_; // owned by the writer thread's round-trips
    bool configure_ads_device();

    // Joins the I/O threads, releases the handles and drops the devices, in that order.
    void teardown_ads_device();

    // Releases every cached PLC symbol handle (ADSDataLayout::ads_handle_owner). Each handle's
    // deleter calls DeleteSymbolHandle through the issuing device, so this must run while it
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

    // Owned by the control loop; the writer thread only sees the handed-over PendingWrite.
    std::vector<utilities::SumWriteItemSpan> write_item_spans_;
    std::vector<size_t> identity_layout_indices_;
    std::vector<uint8_t> write_layout_seeded_;
    std::vector<uint8_t> ads_buffer_sum_write_compact_;
    std::vector<size_t> compact_layout_indices_;

    /**
     * @brief Copies the I/O threads' counters into the introspected mirrors
     */
    void refresh_transaction_statistics();

    /**
     * @brief Registers the ADS transaction statistics with the ros2_control introspection
     */
    void register_transaction_statistics();

    // ===== ADS transaction statistics ==========================================
    std::atomic<long long> read_rtt_ns_{0};
    std::atomic<long long> write_rtt_ns_{0};
    std::atomic<uint64_t> read_transactions_total_{0};
    std::atomic<uint64_t> write_transactions_total_{0};
    std::atomic<uint64_t> write_coalesced_total_{0};
    std::atomic<uint64_t> read_failures_total_{0};
    std::atomic<uint64_t> write_failures_total_{0};

    // Excludes interfaces nothing has ever commanded.
    std::atomic<uint64_t> fallback_activations_{0};

    std::atomic<uint64_t> fallback_activations_cycle_{0};

    std::atomic<uint64_t> never_commanded_interfaces_{0};

    // Introspected mirrors. Only the control loop writes these.
    double stat_read_rtt_ms_{0.0};
    double stat_write_rtt_ms_{0.0};
    double stat_read_transactions_{0.0};
    double stat_write_transactions_{0.0};
    double stat_write_coalesced_{0.0};
    double stat_read_failures_{0.0};
    double stat_write_failures_{0.0};
    double stat_fallback_activations_{0.0};
    double stat_fallback_activations_per_cycle_{0.0};
    double stat_never_commanded_interfaces_{0.0};
    double stat_heartbeat_{0.0};
    double stat_read_sample_age_ms_{0.0}; // age of the sample read() last published; control loop only

    std::vector<ReadInstruction> ads_read_instructions_;
    std::vector<WriteInstruction> ads_write_instructions_;

    // ===== Background ADS I/O threads ==========================================
    // The blocking ADS SUM read/write round-trips are moved off the control loop.
    // read()/write() only touch lock-free caches and a coalescing buffer, so the
    // control-loop cycle time no longer depends on PLC/network latency.

    void start_io_threads(); // spawns the writer and reader threads
    void stop_io_threads();  // signals and joins both worker threads; safe to call when idle

    /**
     * @brief Applies the configured scheduling policy, priority and affinity to a thread
     *
     * @param thread The I/O thread to reschedule
     * @param thread_name Human-readable thread name for the log messages
     */
    void apply_io_thread_scheduling(std::thread &thread, const char *thread_name);

    // Overridable via the io_thread_scheduling_policy, io_thread_priority and io_thread_cpu_affinity parameters.
    utilities::ThreadSchedulingConfig io_thread_scheduling_;

    // Writer thread: owns the SUM-write round-trip. write() marshals the latest command
    // buffer, hands it over here, and returns. Only the newest buffer is sent (coalescing).
    void writer_loop();
    std::thread write_thread_;
    std::mutex write_mutex_;
    std::condition_variable write_cv_;
    PendingWrite write_pending_request_; // latest packed SUM-write request awaiting send
    bool write_pending_ = false;                 // a fresh buffer is waiting (guarded by write_mutex_)
    bool write_stop_ = false;                     // stop request (guarded by write_mutex_)
    std::atomic<bool> write_hard_fault_{false};   // outage outlived the grace window, surfaced by write()
    // Consecutive failed SUM-write round-trips, for outage and recovery logs. Writer thread only.
    size_t write_consecutive_failures_ = 0;
    std::chrono::steady_clock::time_point write_outage_start_;
    std::optional<std::chrono::steady_clock::time_point> write_recovery_stable_since_;

    // Reader thread: owns the SUM-read round-trip. Decodes each sample into
    // polling_read_cache_; read() loads from it without blocking.
    void reader_loop();
    std::thread read_thread_;
    std::atomic<bool> read_stop_{false};
    std::atomic<bool> read_hard_fault_{false};      // outage outlived the grace window, surfaced by read()
    utilities::LatestSampleBuffer<ReadSample> read_sample_buffer_;
    std::vector<double> last_decoded_values_; // reader thread only
    uint64_t read_sample_sequence_ = 0;       // reader thread only
    // When each item's current per-item failure streak began; reader thread only.
    std::vector<std::optional<std::chrono::steady_clock::time_point>> item_failure_start_;
    long long read_poll_period_ns_ = 0;             // optional pacing between SUM reads; 0 = unpaced
    // Consecutive failed SUM-read round-trips, for outage and recovery logs. Reader thread only.
    size_t read_consecutive_failures_ = 0;
    std::chrono::steady_clock::time_point read_outage_start_;
    std::optional<std::chrono::steady_clock::time_point> read_recovery_stable_since_;
  };

} // namespace beckhoff_ads_hardware_interface

#endif // beckhoff_ads_hardware_interface__BECKHOFF_SYSTEM_HPP_