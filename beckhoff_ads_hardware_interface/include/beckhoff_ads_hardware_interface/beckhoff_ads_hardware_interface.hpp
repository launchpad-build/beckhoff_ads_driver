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
    // Resolved once at configure so read() never does a string lookup, takes a blocking
    // wait or hits a throwing path on the control loop.
    hardware_interface::StateInterface::SharedPtr state_handle;
  };

  struct WriteInstruction
  {
    size_t write_buffer_offset_data;
    PLCType plc_type;
    std::string command_interface_name;
    std::string fallback_state_interface_name; // The state interface name corresponding to the current command interface name
    CommandFallback fallback = CommandFallback::HOLD_LAST;
    bool is_heartbeat = false; // value comes from the interface's own counter, not a controller
    // Set on the first cycle this interface carried a command. Until then its fallback is
    // the normal case, not a dropout, and it must not count as one.
    bool has_been_commanded = false;
    // Set once this field of the write buffer holds a value something provided: a command,
    // a fallback, or the interface's initial_value at configure. An unseeded field must
    // never reach the PLC; its whole item is left out of the transmitted request.
    bool seeded = false;
    size_t layout_index = 0; // index of the owning layout in ads_item_layouts_write_
    // Resolved once at configure so write() never does a string lookup, takes a blocking
    // wait or hits a throwing path on the control loop. Null for the heartbeat.
    hardware_interface::CommandInterface::SharedPtr command_handle;
    hardware_interface::StateInterface::SharedPtr fallback_state_handle;
  };

  // One whole decoded SUM-read sample, published atomically so a control cycle never
  // mixes values from two different reads. The stamp tells consumers how old it is.
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

    // An ERROR out of read()/write() transitions the component straight here, without
    // on_deactivate or on_shutdown running. Without this override the I/O threads kept
    // running on a component the controller manager had already given up on, and the next
    // on_configure replaced the ADS devices underneath them.
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
     * Interfaces without a declared initial_value are set to 0.0, so a consumer testing
     * against zero never reads an absent symbol as asserted. A declared initial_value is
     * left in place; the framework applied it to the handle at export.
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

    // A comms outage shorter than this is ridden out on the last cached values; only once it
    // outlives the window does read()/write() surface an error and let the controller manager
    // tear the stack down. Overridable via the comms_outage_grace_ms hardware parameter.
    std::chrono::milliseconds comms_outage_grace_{1000};

    // ========= PLC ==============================

    // PLC Type and Size Helpers
    PLCType strToPlcType(const std::string &type_str);
    size_t plcTypeByteSize(PLCType type_enum);

    // ADS Communication objects
    // The reader and writer threads dereference these on every round-trip without holding a
    // lock, so every site that resets or replaces them has to join both threads first
    // (stop_io_threads). Otherwise a thread is left calling a method on a destroyed AdsDevice,
    // whose m_LocalPort is already freed.
    // Each device opens its own local AMS port (AdsPortOpenEx in the AdsDevice constructor)
    // on the shared process-wide local net id, and the route to the PLC is refcounted, so
    // the reader's and writer's round-trips never contend for a port. The AMS identity the
    // PLC sees is one net id with two ports, which is one client, not two.
    std::unique_ptr<AdsDevice> ads_read_device_;  // owned by the reader thread's round-trips
    std::unique_ptr<AdsDevice> ads_write_device_; // owned by the writer thread's round-trips
    bool configure_ads_device();

    // Joins the I/O threads, releases the symbol handles and drops the device, in the only
    // order that is safe. Shared by on_shutdown and on_error.
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

    // Per-layout spans and seeding state for leaving never-provided items out of the
    // transmitted request. All owned by the control loop; the writer thread only sees
    // the PendingWrite handed over under write_mutex_.
    std::vector<utilities::SumWriteItemSpan> write_item_spans_;
    std::vector<size_t> identity_layout_indices_;
    std::vector<uint8_t> write_layout_seeded_;
    std::vector<uint8_t> ads_buffer_sum_write_compact_;
    std::vector<size_t> compact_layout_indices_;

    /**
     * @brief Copies the I/O threads' counters into the introspected mirrors
     *
     * pal_statistics reads plain doubles by address from the publisher thread, so the
     * atomics the I/O threads own are mirrored here, on the control loop, rather than
     * registered directly.
     */
    void refresh_transaction_statistics();

    /**
     * @brief Registers the ADS transaction statistics with the ros2_control introspection
     */
    void register_transaction_statistics();

    // ===== ADS transaction statistics ==========================================
    // Written by the I/O threads, mirrored onto the control loop and published through
    // the controller manager's introspection topic, so a bag holds the link's behaviour
    // alongside the trajectory it was carrying.
    std::atomic<long long> read_rtt_ns_{0};
    std::atomic<long long> write_rtt_ns_{0};
    std::atomic<uint64_t> read_transactions_total_{0};
    std::atomic<uint64_t> write_transactions_total_{0};
    std::atomic<uint64_t> write_coalesced_total_{0};
    std::atomic<uint64_t> read_failures_total_{0};
    std::atomic<uint64_t> write_failures_total_{0};

    // Interface-cycles on which an interface that has carried a command before carried none
    // and its fallback applied. Interfaces nothing has ever commanded are excluded, because
    // write() clears every command interface to NaN after packing and a controller only
    // writes the interfaces its most recent command message named. Counting those made the
    // total grow every cycle whatever the controllers did, which is what it was meant to
    // detect: measured at exactly 4.000 per cycle on the LMCF gantry after an engage,
    // falling to 1.000 once a message named four of the five gpio commands, the remainder
    // being an enable nothing had ever written.
    std::atomic<uint64_t> fallback_activations_{0};

    // The same count over the most recent write() cycle only. Cumulative totals hide when a
    // dropout started and when it stopped; the rate is what a reader wants.
    std::atomic<uint64_t> fallback_activations_cycle_{0};

    // How many command interfaces have never carried a command. A quiet interface lands
    // here once and stays put, so it is legible without inflating the dropout count.
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
     * Failures are warned about and left non-fatal, so a process without the
     * real-time capability still runs with normal scheduling.
     *
     * @param thread The I/O thread to reschedule
     * @param thread_name Human-readable thread name for the log messages
     */
    void apply_io_thread_scheduling(std::thread &thread, const char *thread_name);

    // Scheduling applied to both I/O threads. Defaults to SCHED_FIFO at priority 50, so a
    // wake after the write notify is not at the mercy of time-sharing. Overridable through
    // the io_thread_scheduling_policy, io_thread_priority and io_thread_cpu_affinity
    // hardware parameters; policy inherit restores plain std::thread behaviour.
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
    // Whole-sample hand-over from the reader thread to read(). The reader keeps its own
    // last-good values so a per-item failure holds that item while the rest stay live.
    utilities::LatestSampleBuffer<ReadSample> read_sample_buffer_;
    std::vector<double> last_decoded_values_; // reader thread only
    uint64_t read_sample_sequence_ = 0;       // reader thread only
    long long read_poll_period_ns_ = 0;             // optional pacing between SUM reads; 0 = unpaced
    // Consecutive failed SUM-read round-trips, for outage and recovery logs. Reader thread only.
    size_t read_consecutive_failures_ = 0;
    std::chrono::steady_clock::time_point read_outage_start_;
    std::optional<std::chrono::steady_clock::time_point> read_recovery_stable_since_;
  };

} // namespace beckhoff_ads_hardware_interface

#endif // beckhoff_ads_hardware_interface__BECKHOFF_SYSTEM_HPP_