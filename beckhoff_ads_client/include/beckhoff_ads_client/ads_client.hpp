// Copyright (c) 2025, b-robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

/**
 * @file ads_client.hpp
 * @brief Streams PLC symbols over ADS sum commands on background reader and writer threads.
 */
#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "beckhoff_ads_client/ads_client_utilities.hpp"
#include "beckhoff_ads_client/plc_type.hpp"

namespace beckhoff_ads_client
{

/** @brief Severity of a message the client reports. */
enum class LogSeverity
{
  Debug,
  Info,
  Warning,
  Error,
  Fatal,
};

/** @brief Receives the client's messages; called from the configuring thread and from the I/O threads. */
using LogSink = std::function<void(LogSeverity severity, std::string_view message)>;

/** @brief Where the PLC is and how this host identifies itself to it. */
struct ConnectionSettings
{
  /** @brief IPv4 address of the PLC. */
  std::string plc_ip_address;
  /** @brief AMS net id of the PLC, six dotted bytes. */
  std::string plc_ams_net_id;
  /** @brief AMS port of the PLC runtime, 1 to 65535. */
  std::uint16_t plc_ams_port{851};
  /** @brief AMS net id of this host, six dotted bytes; process-wide in the ADS library. */
  std::string local_ams_net_id;
  /** @brief Bound on one blocking ADS request; zero keeps the library default of 5 s. */
  std::chrono::milliseconds request_timeout{100};
};

/** @brief Whether a hard fault stays until the next start (Latch) or clears on the next good round trip (Resume). */
enum class OutageBehaviour
{
  Latch,
  Resume,
};

/** @brief Connection and behaviour of an AdsClient. */
struct ClientSettings
{
  /** @brief Where the PLC is. */
  ConnectionSettings connection;
  /** @brief How long a direction may keep failing, or a critical element keep failing, before it hard-faults. */
  std::chrono::milliseconds outage_grace{100};
  /** @brief Whether a hard fault latches. */
  OutageBehaviour outage_behaviour{OutageBehaviour::Latch};
  /** @brief Minimum time between the starts of two sum reads; zero reads back to back. */
  std::chrono::nanoseconds read_poll_period{0};
  /** @brief Age beyond which the latest sample counts as stale; zero disables; raised to three poll periods. */
  std::chrono::nanoseconds read_staleness_timeout{std::chrono::milliseconds(100)};
  /** @brief Scheduling of the reader and writer threads. */
  utilities::ThreadSchedulingConfig io_thread_scheduling;
  /** @brief UDINT symbol advanced by one on every submit; empty disables the heartbeat. */
  std::string heartbeat_symbol;
  /** @brief UDINT symbol carrying the submit sequence number; empty disables it. */
  std::string setpoint_sequence_symbol;
  /** @brief LREAL symbol carrying the steady-clock seconds of each submit; empty disables it. */
  std::string setpoint_timestamp_symbol;
  /** @brief Receiver of the client's messages; empty writes them to standard error. */
  LogSink log;
};

/**
 * @brief One element of a PLC symbol, exchanged as one double.
 *
 * Elements naming the same symbol share one sum-request item; the first of them sets its type and
 * element count. A scalar symbol has element_count 1 and element_index 0.
 */
struct SymbolElement
{
  /** @brief Full symbol name, such as "MAIN.axis_position". */
  std::string symbol;
  /** @brief Type of one element of the symbol. */
  PlcType type{PlcType::Lreal};
  /** @brief Number of elements of the symbol; 1 for a scalar. */
  std::size_t element_count{1};
  /** @brief Index of this element within the symbol. */
  std::size_t element_index{0};
  /** @brief Whether a symbol missing on the PLC is dropped with a warning instead of failing configure. */
  bool optional{false};
  /** @brief Read elements only: whether failing beyond the outage grace hard-faults the reading. */
  bool critical{false};
  /** @brief Write elements only: value the symbol carries until the first submit provides one. */
  std::optional<double> initial_value;
  /** @brief Name used for this element in messages; empty uses the symbol name. */
  std::string label;
};

/** @brief Why configure failed. */
enum class ConfigureFailure
{
  None,
  InvalidSettings,
  ConnectionFailed,
  RequiredSymbolMissing,
};

/** @brief Outcome of AdsClient::configure. */
struct ConfigureResult
{
  /** @brief None on success. */
  ConfigureFailure failure{ConfigureFailure::None};
  /** @brief What went wrong, empty on success. */
  std::string error;
  /** @brief Indices of read elements dropped because their optional symbol is not on the PLC. */
  std::vector<std::size_t> dropped_read_elements;
  /** @brief Indices of write elements dropped because their optional symbol is not on the PLC. */
  std::vector<std::size_t> dropped_write_elements;

  /** @brief Whether configure succeeded. */
  bool succeeded() const {return failure == ConfigureFailure::None;}
};

/** @brief Where the client is in its life. */
enum class ClientState
{
  Disconnected,
  Stopped,
  Running,
  Interrupted,
  Faulted,
};

/** @brief Describes the sample latest_sample copied out. */
struct SampleInfo
{
  /** @brief Number of the sample since configure, starting at 1. */
  std::uint64_t sequence{0};
  /** @brief Steady-clock instant the sample was decoded. */
  std::chrono::steady_clock::time_point stamp{};
  /** @brief How old the sample was when it was copied out. */
  std::chrono::nanoseconds age{0};
  /** @brief Whether the age exceeds the staleness timeout. */
  bool stale{false};
};

/** @brief Outcome of waiting for the first sample after start. */
enum class FirstSampleOutcome
{
  Received,
  NothingToRead,
  TimedOut,
  LinkFaulted,
};

/** @brief Counters and latest timings of the ADS transactions. */
struct TransactionStatistics
{
  /** @brief Duration of the latest sum-read round trip. */
  std::chrono::nanoseconds read_round_trip{0};
  /** @brief Duration of the latest sum-write round trip. */
  std::chrono::nanoseconds write_round_trip{0};
  /** @brief Time the latest sent write waited between submit and the start of its send. */
  std::chrono::nanoseconds write_buffer_age{0};
  /** @brief Time between submit and the end of the send of the latest sent write. */
  std::chrono::nanoseconds write_handoff_latency{0};
  /** @brief Sum reads attempted. */
  std::uint64_t read_transactions{0};
  /** @brief Sum writes attempted. */
  std::uint64_t write_transactions{0};
  /** @brief Submitted writes replaced by a newer one before they were sent. */
  std::uint64_t coalesced_writes{0};
  /** @brief Sum reads that failed as a whole. */
  std::uint64_t read_failures{0};
  /** @brief Sum writes that failed as a whole. */
  std::uint64_t write_failures{0};
  /** @brief Samples published since the last start. */
  std::uint64_t samples_published{0};
  /** @brief Latest heartbeat value submitted. */
  std::uint32_t heartbeat{0};
  /** @brief Latest setpoint sequence number submitted. */
  std::uint32_t setpoint_sequence{0};
};

/**
 * @brief Exchanges PLC symbols with one TwinCAT runtime over ADS.
 *
 * configure connects, resolves every symbol to a handle and lays out one sum-read and one sum-write
 * request. start runs a reader thread that reads every read element in one transaction, and a writer
 * thread that sends the newest submitted values in one transaction, each on its own local AMS port.
 * latest_sample and submit never block, allocate or throw, so a control loop calls them every cycle;
 * each is meant for one thread at a time. A value not provided yet is never sent: an item is left
 * out of the write until every element of it has been provided once, after which a NaN holds its
 * last value.
 */
class AdsClient
{
public:
  /** @brief Creates a disconnected client. */
  AdsClient();
  /** @brief Stops the threads and releases the handles and the connection. */
  ~AdsClient();
  AdsClient(const AdsClient &) = delete;
  AdsClient & operator=(const AdsClient &) = delete;

  /**
   * @brief Connects, resolves the symbols and lays out the requests, replacing any earlier configuration.
   *
   * Sample values follow read_elements and submitted values follow write_elements, index for index.
   * On failure the client is left disconnected.
   */
  ConfigureResult configure(
    const ClientSettings & settings,
    const std::vector<SymbolElement> & read_elements,
    const std::vector<SymbolElement> & write_elements);

  /** @brief Starts the reader and writer threads and clears earlier faults; false when not configured. */
  bool start();

  /** @brief Stops the threads and waits for them; the configuration stays. */
  void stop();

  /** @brief Stops the threads, then releases the symbol handles and the connection. */
  void disconnect();

  /** @brief The client's state; a stale latest sample counts as Faulted. */
  ClientState state() const noexcept;

  /** @brief Whether reading has hard-faulted: an outage beyond the grace window, or a failing critical element. */
  bool read_faulted() const noexcept;

  /** @brief Whether writing has hard-faulted: an outage beyond the grace window. */
  bool write_faulted() const noexcept;

  /** @brief Number of symbols in the sum read; zero when there is nothing to read. */
  std::size_t read_item_count() const noexcept;

  /** @brief Number of symbols in the sum write, including heartbeat and setpoint annotations. */
  std::size_t write_item_count() const noexcept;

  /**
   * @brief Copies the newest sample into values, one per read element; a dropped element reads NaN.
   * @return Nothing, leaving values untouched, while no sample has arrived since configure.
   */
  std::optional<SampleInfo> latest_sample(std::span<double> values) noexcept;

  /** @brief Hands over one value per write element for the writer to send; NaN means not provided. */
  void submit(std::span<const double> values) noexcept;

  /** @brief Waits until the first sample since start arrives, reading faults, or timeout passes. */
  FirstSampleOutcome wait_for_first_sample(std::chrono::milliseconds timeout);

  /** @brief Current transaction statistics. */
  TransactionStatistics statistics() const noexcept;

private:
  struct Implementation;
  std::unique_ptr<Implementation> implementation_;
};

}  // namespace beckhoff_ads_client
