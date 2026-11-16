// Copyright (c) 2025, b-robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include "beckhoff_ads_client/ads_client.hpp"

#include <pthread.h>
#include <sched.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <format>
#include <iostream>
#include <limits>
#include <map>
#include <thread>
#include <utility>

#include "ads/AdsDevice.h"
#include "ads/AdsLib.h"

#include "beckhoff_ads_client/latest_sample_buffer.hpp"
#include "outage_tracker.hpp"
#include "sum_requests.hpp"
#include "write_handoff.hpp"

namespace beckhoff_ads_client
{
namespace
{

using SteadyClock = std::chrono::steady_clock;

constexpr std::chrono::milliseconds READ_FAILURE_BACKOFF{10};
constexpr std::chrono::seconds LOG_THROTTLE_PERIOD{1};

std::string_view ads_error_text(long error_code)
{
  std::string_view text = "unrecognised ADS error code";
  switch (error_code) {
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

class LogThrottle
{
public:
  bool allow(SteadyClock::time_point now)
  {
    const bool result = !last_ || now - *last_ >= LOG_THROTTLE_PERIOD;
    if (result) {
      last_ = now;
    }
    return result;
  }

private:
  std::optional<SteadyClock::time_point> last_;
};

struct ReadSample
{
  std::vector<double> values;
  SteadyClock::time_point stamp{};
  std::uint64_t sequence{0};
};

struct ItemPlan
{
  SumItem item;
  bool optional{true};
  std::vector<std::size_t> element_indices;
};

std::string element_label(const SymbolElement & element)
{
  return element.label.empty() ? element.symbol : element.label;
}

std::int64_t to_nanoseconds(SteadyClock::duration duration)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
}

AmsNetId to_ams_net_id(const std::array<std::uint8_t, 6> & bytes)
{
  return AmsNetId(bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5]);
}

}  // namespace

struct AdsClient::Implementation
{
  ClientSettings settings;
  std::atomic<bool> configured{false};
  std::atomic<bool> running{false};
  std::chrono::nanoseconds staleness_timeout{0};

  std::unique_ptr<AdsDevice> read_device;
  std::unique_ptr<AdsDevice> write_device;
  std::vector<AdsHandle> read_handles;
  std::vector<AdsHandle> write_handles;

  SumReadRequest read_request;
  SumWritePacker write_packer;
  WriteHandoff write_handoff;
  std::vector<std::uint8_t> write_response;

  std::vector<double> decoded_values;
  utilities::LatestSampleBuffer<ReadSample> samples;
  std::uint64_t sample_sequence{0};

  OutageTracker read_outage;
  OutageTracker write_outage;

  std::thread reader;
  std::thread writer;
  std::atomic<bool> read_stop{false};
  std::atomic<bool> write_stop{false};

  bool heartbeat_enabled{false};
  std::uint32_t heartbeat{0};
  utilities::SetpointSequenceCounter setpoint_sequence;

  std::atomic<std::int64_t> read_round_trip_ns{0};
  std::atomic<std::int64_t> write_round_trip_ns{0};
  std::atomic<std::int64_t> write_buffer_age_ns{0};
  std::atomic<std::int64_t> write_handoff_latency_ns{0};
  std::atomic<std::uint64_t> read_transactions{0};
  std::atomic<std::uint64_t> write_transactions{0};
  std::atomic<std::uint64_t> coalesced_writes{0};
  std::atomic<std::uint64_t> read_failures{0};
  std::atomic<std::uint64_t> write_failures{0};
  std::atomic<std::uint64_t> samples_published{0};
  std::atomic<std::int64_t> latest_sample_stamp_ns{0};
  std::atomic<std::uint32_t> submitted_heartbeat{0};
  std::atomic<std::uint32_t> submitted_setpoint_sequence{0};

  LogThrottle read_failure_log;
  LogThrottle read_size_log;
  LogThrottle read_item_log;
  LogThrottle critical_item_log;
  LogThrottle write_failure_log;
  LogThrottle write_size_log;
  LogThrottle write_item_log;

  void log(LogSeverity severity, const std::string & message) const
  {
    if (settings.log) {
      settings.log(severity, message);
    } else {
      std::cerr << "[beckhoff_ads_client] " << message << '\n';
    }
  }

  ConfigureResult configure(
    const ClientSettings & new_settings,
    const std::vector<SymbolElement> & read_elements,
    const std::vector<SymbolElement> & write_elements);
  bool validate(const std::vector<SymbolElement> & elements, std::string_view direction, ConfigureResult & result) const;
  bool connect(ConfigureResult & result);
  std::vector<ItemPlan> plan_items(const std::vector<SymbolElement> & elements, std::string_view direction) const;
  bool resolve(
    std::vector<ItemPlan> & plans, AdsDevice & device, std::vector<AdsHandle> & handles,
    std::string_view direction, std::vector<std::size_t> & dropped_elements);
  void build_read_request(std::vector<ItemPlan> plans, const std::vector<SymbolElement> & elements);
  void build_write_packer(std::vector<ItemPlan> plans, const std::vector<SymbolElement> & elements);

  bool start();
  void stop();
  void disconnect();
  void apply_thread_scheduling(std::thread & thread, std::string_view thread_name) const;
  void reader_loop();
  void writer_loop();
  bool latest_sample_stale(SteadyClock::time_point now) const noexcept;
};

bool AdsClient::Implementation::validate(
  const std::vector<SymbolElement> & elements, std::string_view direction, ConfigureResult & result) const
{
  std::map<std::string, std::size_t> element_counts;
  bool valid = true;
  for (const SymbolElement & element : elements) {
    std::string problem;
    if (element.symbol.empty()) {
      problem = "has no symbol name";
    } else if (element.element_count == 0) {
      problem = "declares zero elements";
    } else {
      const std::size_t element_count = element_counts.emplace(element.symbol, element.element_count).first->second;
      if (element.element_index >= element_count) {
        problem = std::format(
          "uses element {} of symbol '{}', which has {} elements", element.element_index, element.symbol,
          element_count);
      }
    }
    if (valid && !problem.empty()) {
      valid = false;
      result.failure = ConfigureFailure::InvalidSettings;
      result.error = std::format("The {} element '{}' {}.", direction, element_label(element), problem);
      log(LogSeverity::Fatal, result.error);
    }
  }
  return valid;
}

bool AdsClient::Implementation::connect(ConfigureResult & result)
{
  const ConnectionSettings & connection = settings.connection;
  const auto remote_net_id = utilities::parseAmsNetId(connection.plc_ams_net_id);
  const auto local_net_id = utilities::parseAmsNetId(connection.local_ams_net_id);
  if (!remote_net_id || !local_net_id || connection.plc_ams_port == 0) {
    result.failure = ConfigureFailure::InvalidSettings;
    if (!remote_net_id) {
      result.error = std::format(
        "Invalid PLC AMS net id '{}'. Expected six dotted bytes, x.x.x.x.x.x.", connection.plc_ams_net_id);
    } else if (!local_net_id) {
      result.error = std::format(
        "Invalid local AMS net id '{}'. Expected six dotted bytes, x.x.x.x.x.x.", connection.local_ams_net_id);
    } else {
      result.error = "Invalid PLC AMS port 0. Expected 1 to 65535.";
    }
    log(LogSeverity::Fatal, "\t" + result.error);
    return false;
  }

  log(LogSeverity::Info, "Configuring ADS device...");
  try {
    bhf::ads::SetLocalAddress(to_ams_net_id(*local_net_id));
    read_device = std::make_unique<AdsDevice>(
      connection.plc_ip_address, to_ams_net_id(*remote_net_id), connection.plc_ams_port);
    write_device = std::make_unique<AdsDevice>(
      connection.plc_ip_address, to_ams_net_id(*remote_net_id), connection.plc_ams_port);
    if (connection.request_timeout.count() > 0) {
      read_device->SetTimeout(static_cast<std::uint32_t>(connection.request_timeout.count()));
      write_device->SetTimeout(static_cast<std::uint32_t>(connection.request_timeout.count()));
    }
    log(LogSeverity::Info, std::format("\tTimeout is: {}", read_device->GetTimeout()));
    log(
      LogSeverity::Info, std::format(
        "\tADS Devices configured for PLC: {}, Port: {}", connection.plc_ip_address, connection.plc_ams_port));
    log(
      LogSeverity::Info, std::format(
        "\tPLC AMS NetID: {}. Reader on local AMS port {}, writer on local AMS port {}.",
        connection.plc_ams_net_id, read_device->GetLocalPort(), write_device->GetLocalPort()));
    log(LogSeverity::Info, "Requesting Device state...");
    const AdsDeviceState device_state = read_device->GetState();
    log(
      LogSeverity::Info, std::format(
        "\tCommunication successful! ADS State: {}, DeviceState: {}", static_cast<int>(device_state.ads),
        static_cast<int>(device_state.device)));
  } catch (const AdsException & exception) {
    result.failure = ConfigureFailure::ConnectionFailed;
    result.error = std::format(
      "ADS Exception during connection to {} (AMS NetId {}): {} (error 0x{:X}: {})", connection.plc_ip_address,
      connection.plc_ams_net_id, exception.what(), exception.errorCode, ads_error_text(exception.errorCode));
  } catch (const std::exception & exception) {
    result.failure = ConfigureFailure::ConnectionFailed;
    result.error = std::format("Error during ADS connection config: {}", exception.what());
  }
  if (!result.succeeded()) {
    log(LogSeverity::Fatal, "\t" + result.error);
  }
  return result.succeeded();
}

std::vector<ItemPlan> AdsClient::Implementation::plan_items(
  const std::vector<SymbolElement> & elements, std::string_view direction) const
{
  std::vector<ItemPlan> plans;
  std::map<std::string, std::size_t> plan_by_symbol;
  for (std::size_t index = 0; index < elements.size(); ++index) {
    const SymbolElement & element = elements[index];
    const auto [position, inserted] = plan_by_symbol.emplace(element.symbol, plans.size());
    if (inserted) {
      ItemPlan plan;
      plan.item = SumItem{element.symbol, element.type, element.element_count, 0};
      plans.push_back(std::move(plan));
    }
    ItemPlan & plan = plans[position->second];
    if (!inserted && (plan.item.type != element.type || plan.item.element_count != element.element_count)) {
      log(
        LogSeverity::Warning, std::format(
          "The {} element '{}' declares symbol '{}' as {}[{}], but an earlier element declared it {}[{}]. "
          "Using the earlier declaration.",
          direction, element_label(element), element.symbol, plc_type_name(element.type), element.element_count,
          plc_type_name(plan.item.type), plan.item.element_count));
    }
    plan.optional = plan.optional && element.optional;
    plan.element_indices.push_back(index);
  }
  return plans;
}

bool AdsClient::Implementation::resolve(
  std::vector<ItemPlan> & plans, AdsDevice & device, std::vector<AdsHandle> & handles,
  std::string_view direction, std::vector<std::size_t> & dropped_elements)
{
  bool required_symbol_missing = false;
  std::vector<ItemPlan> resolved;
  for (ItemPlan & plan : plans) {
    try {
      AdsHandle handle = device.GetHandle(plan.item.symbol);
      plan.item.handle = *handle;
      handles.push_back(std::move(handle));
      resolved.push_back(std::move(plan));
    } catch (const std::exception & exception) {
      if (plan.optional) {
        log(
          LogSeverity::Warning, std::format(
            "\tOptional {} symbol '{}' is not on the PLC ({}). Skipping it; its interfaces are set to their "
            "declared initial_value, or 0.0 without one.",
            direction, plan.item.symbol, exception.what()));
        dropped_elements.insert(dropped_elements.end(), plan.element_indices.begin(), plan.element_indices.end());
      } else {
        log(
          LogSeverity::Fatal, std::format(
            "\tRequired {} symbol '{}' is not on the PLC ({}). Flash the PLC project first, or mark the "
            "interface optional.",
            direction, plan.item.symbol, exception.what()));
        required_symbol_missing = true;
      }
    }
  }
  plans = std::move(resolved);
  std::sort(dropped_elements.begin(), dropped_elements.end());
  return !required_symbol_missing;
}

void AdsClient::Implementation::build_read_request(
  std::vector<ItemPlan> plans, const std::vector<SymbolElement> & elements)
{
  std::vector<SumItem> items;
  std::vector<ReadSlot> slots;
  for (std::size_t item_index = 0; item_index < plans.size(); ++item_index) {
    for (const std::size_t element_index : plans[item_index].element_indices) {
      const SymbolElement & element = elements[element_index];
      slots.push_back(ReadSlot{item_index, element.element_index, element_index, element.critical, element_label(element)});
    }
    items.push_back(std::move(plans[item_index].item));
  }
  read_request = SumReadRequest(std::move(items), std::move(slots));
  decoded_values.assign(elements.size(), std::numeric_limits<double>::quiet_NaN());
  const std::size_t value_count = elements.size();
  samples.initialiseSlots(
    [value_count](ReadSample & sample) {
      sample.values.assign(value_count, std::numeric_limits<double>::quiet_NaN());
      sample.stamp = SteadyClock::time_point{};
      sample.sequence = 0;
    });
  sample_sequence = 0;
  if (read_request.item_count() == 0) {
    log(LogSeverity::Info, "No items to configure for ADS Sum READ.");
  } else {
    log(
      LogSeverity::Info, std::format(
        "ADS Sum READ configured for {} items. Request: {} bytes, Response: {} bytes.", read_request.item_count(),
        read_request.request().size(), read_request.response().size()));
  }
}

void AdsClient::Implementation::build_write_packer(
  std::vector<ItemPlan> plans, const std::vector<SymbolElement> & elements)
{
  std::vector<SumItem> items;
  std::vector<WriteSlot> slots;
  for (std::size_t item_index = 0; item_index < plans.size(); ++item_index) {
    for (const std::size_t element_index : plans[item_index].element_indices) {
      if (element_index < elements.size()) {
        slots.push_back(WriteSlot{item_index, elements[element_index].element_index, WriteSource::Element, element_index});
      } else {
        const std::size_t synthetic_index = element_index - elements.size();
        const std::array<WriteSource, 3> synthetic_sources{
          WriteSource::Heartbeat, WriteSource::SetpointSequence, WriteSource::SetpointTimestamp};
        slots.push_back(WriteSlot{item_index, 0, synthetic_sources[synthetic_index], 0});
      }
    }
    items.push_back(std::move(plans[item_index].item));
  }
  write_packer = SumWritePacker(std::move(items), std::move(slots));
  for (std::size_t element_index = 0; element_index < elements.size(); ++element_index) {
    if (elements[element_index].initial_value) {
      write_packer.seed(element_index, *elements[element_index].initial_value);
    }
  }
  write_response.assign(write_packer.item_count() * SUM_ITEM_ERROR_SIZE, 0);
  write_handoff.reset(write_packer.request_size(), write_packer.item_count());
  if (write_packer.item_count() == 0) {
    log(LogSeverity::Info, "No items to configure for ADS Sum WRITE.");
  } else {
    log(
      LogSeverity::Info, std::format(
        "ADS Sum WRITE configured for {} items. Request: {} bytes, Response: {} bytes.", write_packer.item_count(),
        write_packer.request_size(), write_response.size()));
  }
}

ConfigureResult AdsClient::Implementation::configure(
  const ClientSettings & new_settings,
  const std::vector<SymbolElement> & read_elements,
  const std::vector<SymbolElement> & write_elements)
{
  disconnect();
  settings = new_settings;
  staleness_timeout = settings.read_staleness_timeout;
  if (staleness_timeout.count() > 0 && settings.read_poll_period.count() > 0) {
    staleness_timeout = std::max(staleness_timeout, 3 * settings.read_poll_period);
  }
  heartbeat_enabled = !settings.heartbeat_symbol.empty();

  ConfigureResult result;
  if (!validate(read_elements, "read", result) || !validate(write_elements, "write", result) || !connect(result)) {
    disconnect();
    return result;
  }

  std::vector<SymbolElement> annotated_write_elements = write_elements;
  const std::array<std::pair<std::string, PlcType>, 3> annotations{{
    {settings.heartbeat_symbol, PlcType::Udint},
    {settings.setpoint_sequence_symbol, PlcType::Udint},
    {settings.setpoint_timestamp_symbol, PlcType::Lreal},
  }};
  std::vector<ItemPlan> write_plans = plan_items(write_elements, "write");
  for (std::size_t annotation = 0; annotation < annotations.size(); ++annotation) {
    if (!annotations[annotation].first.empty()) {
      ItemPlan plan;
      plan.item = SumItem{annotations[annotation].first, annotations[annotation].second, 1, 0};
      plan.optional = false;
      plan.element_indices.push_back(write_elements.size() + annotation);
      write_plans.push_back(std::move(plan));
    }
  }
  std::vector<ItemPlan> read_plans = plan_items(read_elements, "read");

  log(LogSeverity::Info, "Fetching ADS handles for configured PLC variables...");
  const bool read_resolved = resolve(read_plans, *read_device, read_handles, "read", result.dropped_read_elements);
  const bool write_resolved =
    resolve(write_plans, *write_device, write_handles, "write", result.dropped_write_elements);
  if (!read_resolved || !write_resolved) {
    result.failure = ConfigureFailure::RequiredSymbolMissing;
    result.error = "A required PLC symbol is not on the PLC.";
    disconnect();
    return result;
  }
  log(LogSeverity::Info, "\tHandles acquired");

  log(LogSeverity::Info, "Building ADS sum READ buffer...");
  build_read_request(std::move(read_plans), read_elements);
  log(LogSeverity::Info, "Building ADS sum WRITE buffer...");
  build_write_packer(std::move(write_plans), write_elements);
  configured = true;
  return result;
}

bool AdsClient::Implementation::start()
{
  if (!configured) {
    return false;
  }
  if (running) {
    return true;
  }
  const std::chrono::nanoseconds grace = settings.outage_grace;
  const bool latches = settings.outage_behaviour == OutageBehaviour::Latch;
  read_outage.reset(grace, latches);
  write_outage.reset(grace, latches);
  samples_published.store(0, std::memory_order_release);

  if (write_packer.item_count() > 0) {
    write_handoff.reset(write_packer.request_size(), write_packer.item_count());
    write_stop.store(false);
    writer = std::thread([this] {writer_loop();});
    apply_thread_scheduling(writer, "ADS writer");
  }
  if (read_request.item_count() > 0) {
    read_stop.store(false);
    reader = std::thread([this] {reader_loop();});
    apply_thread_scheduling(reader, "ADS reader");
  }
  running = true;
  return true;
}

void AdsClient::Implementation::stop()
{
  write_stop.store(true);
  write_handoff.wake();
  if (writer.joinable()) {
    writer.join();
  }
  read_stop.store(true);
  if (reader.joinable()) {
    reader.join();
  }
  running = false;
}

void AdsClient::Implementation::disconnect()
{
  stop();
  read_handles.clear();
  write_handles.clear();
  read_device.reset();
  write_device.reset();
  configured = false;
}

void AdsClient::Implementation::apply_thread_scheduling(std::thread & thread, std::string_view thread_name) const
{
  const utilities::ThreadSchedulingConfig & scheduling = settings.io_thread_scheduling;
  if (scheduling.policy != utilities::SchedulingPolicy::INHERIT) {
    int native_policy = SCHED_OTHER;
    if (scheduling.policy == utilities::SchedulingPolicy::FIFO) {
      native_policy = SCHED_FIFO;
    } else if (scheduling.policy == utilities::SchedulingPolicy::ROUND_ROBIN) {
      native_policy = SCHED_RR;
    }
    sched_param scheduling_parameters{};
    scheduling_parameters.sched_priority = scheduling.priority;
    const int scheduling_error = pthread_setschedparam(thread.native_handle(), native_policy, &scheduling_parameters);
    if (scheduling_error != 0) {
      log(
        LogSeverity::Warning, std::format(
          "Could not set the {} thread scheduling (policy {}, priority {}): {}. The thread keeps normal "
          "scheduling; grant the process CAP_SYS_NICE or an rtprio limit, or set io_thread_scheduling_policy "
          "to inherit.",
          thread_name, native_policy, scheduling.priority, std::strerror(scheduling_error)));
    } else {
      log(
        LogSeverity::Info, std::format(
          "{} thread scheduling set: policy {}, priority {}.", thread_name, native_policy, scheduling.priority));
    }
  }
  if (!scheduling.cpu_affinity.empty()) {
    cpu_set_t cpu_set;
    CPU_ZERO(&cpu_set);
    for (const unsigned int cpu : scheduling.cpu_affinity) {
      CPU_SET(cpu, &cpu_set);
    }
    const int affinity_error = pthread_setaffinity_np(thread.native_handle(), sizeof(cpu_set), &cpu_set);
    if (affinity_error != 0) {
      log(
        LogSeverity::Warning,
        std::format("Could not set the {} thread CPU affinity: {}.", thread_name, std::strerror(affinity_error)));
    }
  }
}

void AdsClient::Implementation::reader_loop()
{
  const ConnectionSettings & connection = settings.connection;
  while (!read_stop.load(std::memory_order_acquire)) {
    const SteadyClock::time_point cycle_start = SteadyClock::now();
    const std::span<const std::uint8_t> request = read_request.request();
    const std::span<std::uint8_t> response = read_request.response();
    std::uint32_t bytes_read = 0;
    const long error = read_device->ReadWriteReqEx2(
      ADSIGRP_SUMUP_READ, static_cast<std::uint32_t>(read_request.item_count()), response.size(), response.data(),
      request.size(), request.data(), &bytes_read);
    const SteadyClock::time_point round_trip_end = SteadyClock::now();
    read_round_trip_ns.store(to_nanoseconds(round_trip_end - cycle_start), std::memory_order_relaxed);
    read_transactions.fetch_add(1, std::memory_order_relaxed);

    if (error != ADSERR_NOERR || bytes_read != response.size()) {
      if (error != ADSERR_NOERR && read_outage.consecutive_failures() == 0) {
        log(
          LogSeverity::Error, std::format(
            "ADS Sum Read failed: 0x{:X} ({}). Target {}, AMS NetId {}, port {}.", error, ads_error_text(error),
            connection.plc_ip_address, connection.plc_ams_net_id, connection.plc_ams_port));
      } else if (error != ADSERR_NOERR && read_failure_log.allow(round_trip_end)) {
        log(LogSeverity::Error, std::format("ADS Sum Read still failing: 0x{:X} ({}).", error, ads_error_text(error)));
      } else if (error == ADSERR_NOERR && read_size_log.allow(round_trip_end)) {
        log(
          LogSeverity::Error,
          std::format("ADS Sum Read size mismatch. Expected {}, Got {}.", response.size(), bytes_read));
      }
      read_failures.fetch_add(1, std::memory_order_relaxed);
      read_outage.record_failure(SteadyClock::now());
      std::this_thread::sleep_for(READ_FAILURE_BACKOFF);
      continue;
    }

    const SteadyClock::time_point decode_time = SteadyClock::now();
    const ReadDecodeResult decoded = read_request.decode(decoded_values, decode_time, settings.outage_grace);
    if (decoded.first_failure && read_item_log.allow(decode_time)) {
      const std::uint32_t item_error = decoded.first_failure->error_code;
      log(
        LogSeverity::Warning, std::format(
          "ADS Sum Read operation corresponding to the state interface '{}' failed: 0x{:X} ({}).",
          read_request.slot(decoded.first_failure->slot_index).label, item_error,
          ads_error_text(static_cast<long>(item_error))));
    }
    if (decoded.critical_fault_slot) {
      if (critical_item_log.allow(decode_time)) {
        log(
          LogSeverity::Error, std::format(
            "Critical state interface '{}' has been failing beyond the grace window. Hard-faulting instead of "
            "freezing its feedback.",
            read_request.slot(*decoded.critical_fault_slot).label));
      }
      read_outage.declare_fault();
    }

    if (decoded.all_failed) {
      read_failures.fetch_add(1, std::memory_order_relaxed);
      read_outage.record_failure(SteadyClock::now());
    } else {
      ReadSample & sample = samples.writeSlot();
      std::copy(decoded_values.begin(), decoded_values.end(), sample.values.begin());
      sample.stamp = SteadyClock::now();
      sample.sequence = ++sample_sequence;
      samples.publish();
      latest_sample_stamp_ns.store(sample.stamp.time_since_epoch().count(), std::memory_order_release);
      samples_published.fetch_add(1, std::memory_order_release);

      const std::optional<OutageRecovery> recovery =
        read_outage.record_success(SteadyClock::now(), decoded.critical_fault_slot.has_value());
      if (recovery) {
        log(
          LogSeverity::Info, std::format(
            "ADS Sum Read recovered after {} failed cycles ({:.1f} s).", recovery->failed_round_trips,
            static_cast<double>(recovery->duration.count()) / 1000.0));
      }
    }

    if (settings.read_poll_period.count() > 0) {
      std::this_thread::sleep_until(cycle_start + settings.read_poll_period);
    }
  }
}

void AdsClient::Implementation::writer_loop()
{
  const ConnectionSettings & connection = settings.connection;
  while (write_handoff.wait_for_write(write_stop)) {
    const PendingWrite & pending = write_handoff.taken();
    const std::size_t item_count = pending.item_indices.size();
    const std::span<std::uint8_t> response = std::span<std::uint8_t>(write_response).first(item_count * SUM_ITEM_ERROR_SIZE);
    const SteadyClock::time_point send_start = SteadyClock::now();
    write_buffer_age_ns.store(to_nanoseconds(send_start - pending.handoff), std::memory_order_relaxed);

    std::uint32_t bytes_returned = 0;
    const long error = write_device->ReadWriteReqEx2(
      ADSIGRP_SUMUP_WRITE, static_cast<std::uint32_t>(item_count), response.size(), response.data(),
      pending.request.size(), pending.request.data(), &bytes_returned);
    const SteadyClock::time_point send_end = SteadyClock::now();
    write_round_trip_ns.store(to_nanoseconds(send_end - send_start), std::memory_order_relaxed);
    write_handoff_latency_ns.store(to_nanoseconds(send_end - pending.handoff), std::memory_order_relaxed);
    write_transactions.fetch_add(1, std::memory_order_relaxed);

    bool failed = false;
    if (error != ADSERR_NOERR) {
      if (write_outage.consecutive_failures() == 0) {
        log(
          LogSeverity::Error, std::format(
            "ADS Sum Write failed: 0x{:X} ({}). Target {}, AMS NetId {}, port {}.", error, ads_error_text(error),
            connection.plc_ip_address, connection.plc_ams_net_id, connection.plc_ams_port));
      } else if (write_failure_log.allow(send_end)) {
        log(LogSeverity::Error, std::format("ADS Sum Write still failing: 0x{:X} ({}).", error, ads_error_text(error)));
      }
      failed = true;
    } else if (bytes_returned != response.size()) {
      if (write_size_log.allow(send_end)) {
        log(
          LogSeverity::Error, std::format(
            "ADS Sum Write response size mismatch (error codes). Expected {}, Got {}.", response.size(),
            bytes_returned));
      }
      failed = true;
    } else {
      std::size_t items_failed = 0;
      for (std::size_t index = 0; index < item_count; ++index) {
        std::uint32_t item_error = 0;
        std::memcpy(&item_error, response.subspan(index * SUM_ITEM_ERROR_SIZE).data(), sizeof(item_error));
        if (item_error != ADSERR_NOERR) {
          ++items_failed;
          if (write_item_log.allow(send_end)) {
            const SumItem & item = write_packer.item(pending.item_indices[index]);
            log(
              LogSeverity::Warning, std::format(
                "ADS Sum Write sub-op for '{}' (handle 0x{:X}) failed: 0x{:X} ({})", item.symbol, item.handle,
                item_error, ads_error_text(static_cast<long>(item_error))));
          }
        }
      }
      failed = item_count > 0 && items_failed == item_count;
    }

    if (failed) {
      write_failures.fetch_add(1, std::memory_order_relaxed);
      write_outage.record_failure(SteadyClock::now());
    } else {
      const std::optional<OutageRecovery> recovery = write_outage.record_success(SteadyClock::now(), false);
      if (recovery) {
        log(
          LogSeverity::Info, std::format(
            "ADS Sum Write recovered after {} failed round-trips ({:.1f} s).", recovery->failed_round_trips,
            static_cast<double>(recovery->duration.count()) / 1000.0));
      }
    }
  }
}

bool AdsClient::Implementation::latest_sample_stale(SteadyClock::time_point now) const noexcept
{
  bool stale = false;
  if (staleness_timeout.count() > 0 && samples_published.load(std::memory_order_acquire) > 0) {
    const SteadyClock::time_point stamp{SteadyClock::duration(latest_sample_stamp_ns.load(std::memory_order_acquire))};
    stale = now - stamp > staleness_timeout;
  }
  return stale;
}

AdsClient::AdsClient()
: implementation_(std::make_unique<Implementation>())
{
}

AdsClient::~AdsClient()
{
  implementation_->disconnect();
}

ConfigureResult AdsClient::configure(
  const ClientSettings & settings,
  const std::vector<SymbolElement> & read_elements,
  const std::vector<SymbolElement> & write_elements)
{
  return implementation_->configure(settings, read_elements, write_elements);
}

bool AdsClient::start()
{
  return implementation_->start();
}

void AdsClient::stop()
{
  implementation_->stop();
}

void AdsClient::disconnect()
{
  implementation_->disconnect();
}

ClientState AdsClient::state() const noexcept
{
  const Implementation & self = *implementation_;
  ClientState result = ClientState::Running;
  if (!self.configured) {
    result = ClientState::Disconnected;
  } else if (!self.running) {
    result = ClientState::Stopped;
  } else if (
    self.read_outage.faulted() || self.write_outage.faulted() || self.latest_sample_stale(SteadyClock::now()))
  {
    result = ClientState::Faulted;
  } else if (self.read_outage.interrupted() || self.write_outage.interrupted()) {
    result = ClientState::Interrupted;
  }
  return result;
}

bool AdsClient::read_faulted() const noexcept
{
  return implementation_->read_outage.faulted();
}

bool AdsClient::write_faulted() const noexcept
{
  return implementation_->write_outage.faulted();
}

std::size_t AdsClient::read_item_count() const noexcept
{
  return implementation_->configured ? implementation_->read_request.item_count() : 0;
}

std::size_t AdsClient::write_item_count() const noexcept
{
  return implementation_->configured ? implementation_->write_packer.item_count() : 0;
}

std::optional<SampleInfo> AdsClient::latest_sample(std::span<double> values) noexcept
{
  Implementation & self = *implementation_;
  std::optional<SampleInfo> result;
  if (self.configured) {
    self.samples.refreshReadSlot();
    const ReadSample & sample = self.samples.readSlot();
    if (sample.sequence > 0) {
      const std::size_t count = std::min(values.size(), sample.values.size());
      std::copy_n(sample.values.begin(), count, values.begin());
      SampleInfo info;
      info.sequence = sample.sequence;
      info.stamp = sample.stamp;
      info.age = std::chrono::duration_cast<std::chrono::nanoseconds>(SteadyClock::now() - sample.stamp);
      info.stale = self.staleness_timeout.count() > 0 && info.age > self.staleness_timeout;
      result = info;
    }
  }
  return result;
}

void AdsClient::submit(std::span<const double> values) noexcept
{
  Implementation & self = *implementation_;
  if (self.configured && self.write_packer.item_count() > 0) {
    const SteadyClock::time_point now = SteadyClock::now();
    SyntheticWriteValues synthetic;
    synthetic.setpoint_sequence = self.setpoint_sequence.next();
    synthetic.setpoint_timestamp = utilities::monotonicSeconds(now);
    if (self.heartbeat_enabled) {
      ++self.heartbeat;
    }
    synthetic.heartbeat = self.heartbeat;
    self.submitted_heartbeat.store(self.heartbeat, std::memory_order_relaxed);
    self.submitted_setpoint_sequence.store(synthetic.setpoint_sequence, std::memory_order_relaxed);

    const std::optional<PackedWrite> packed = self.write_packer.pack(values, synthetic);
    if (packed && self.write_handoff.offer(*packed, SteadyClock::now())) {
      self.coalesced_writes.fetch_add(1, std::memory_order_relaxed);
    }
  }
}

FirstSampleOutcome AdsClient::wait_for_first_sample(std::chrono::milliseconds timeout)
{
  const Implementation & self = *implementation_;
  if (read_item_count() == 0) {
    return FirstSampleOutcome::NothingToRead;
  }
  const SteadyClock::time_point deadline = SteadyClock::now() + timeout;
  FirstSampleOutcome result = FirstSampleOutcome::Received;
  bool waiting = true;
  while (waiting) {
    if (self.samples_published.load(std::memory_order_acquire) > 0) {
      result = FirstSampleOutcome::Received;
      waiting = false;
    } else if (self.read_outage.faulted()) {
      result = FirstSampleOutcome::LinkFaulted;
      waiting = false;
    } else if (SteadyClock::now() >= deadline) {
      result = FirstSampleOutcome::TimedOut;
      waiting = false;
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  return result;
}

TransactionStatistics AdsClient::statistics() const noexcept
{
  const Implementation & self = *implementation_;
  TransactionStatistics result;
  result.read_round_trip = std::chrono::nanoseconds(self.read_round_trip_ns.load(std::memory_order_relaxed));
  result.write_round_trip = std::chrono::nanoseconds(self.write_round_trip_ns.load(std::memory_order_relaxed));
  result.write_buffer_age = std::chrono::nanoseconds(self.write_buffer_age_ns.load(std::memory_order_relaxed));
  result.write_handoff_latency =
    std::chrono::nanoseconds(self.write_handoff_latency_ns.load(std::memory_order_relaxed));
  result.read_transactions = self.read_transactions.load(std::memory_order_relaxed);
  result.write_transactions = self.write_transactions.load(std::memory_order_relaxed);
  result.coalesced_writes = self.coalesced_writes.load(std::memory_order_relaxed);
  result.read_failures = self.read_failures.load(std::memory_order_relaxed);
  result.write_failures = self.write_failures.load(std::memory_order_relaxed);
  result.samples_published = self.samples_published.load(std::memory_order_relaxed);
  result.heartbeat = self.submitted_heartbeat.load(std::memory_order_relaxed);
  result.setpoint_sequence = self.submitted_setpoint_sequence.load(std::memory_order_relaxed);
  return result;
}

}  // namespace beckhoff_ads_client
