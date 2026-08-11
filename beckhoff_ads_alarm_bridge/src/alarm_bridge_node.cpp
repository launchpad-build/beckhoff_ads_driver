#include "beckhoff_ads_alarm_bridge/alarm_bridge_node.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>

#include "ads/AdsVariable.h"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"

namespace beckhoff_ads_alarm_bridge
{

namespace
{
// One process = one AlarmBridgeNode instance. The vendored ADS notification API's hUser is a
// uint32_t (not pointer-width), so it cannot carry `this` on a 64-bit build -- this static
// pointer stands in instead. Set just before the notification is registered, cleared just
// before it is torn down.
AlarmBridgeNode * g_instance = nullptr;

double filetimeToUnixEpoch(uint32_t low, uint32_t high)
{
  const uint64_t filetime = (static_cast<uint64_t>(high) << 32) | low;
  // FILETIME: 100ns ticks since 1601-01-01. Offset to 1970-01-01 in 100ns ticks.
  constexpr uint64_t kEpochDiff100ns = 116444736000000000ULL;
  if (filetime < kEpochDiff100ns) {
    return 0.0;
  }
  return static_cast<double>(filetime - kEpochDiff100ns) / 1.0e7;
}
}  // namespace

AlarmBridgeNode::AlarmBridgeNode(const rclcpp::NodeOptions & options)
: Node("alarm_bridge_node", options), updater_(this)
{
  declare_parameter<std::string>("plc_ip_address", "");
  declare_parameter<std::string>("plc_ams_net_id", "");
  declare_parameter<std::string>("local_ams_net_id", "");
  declare_parameter<int>("plc_ams_port", 851);
  declare_parameter<std::string>("trigger_symbol", "GVL_Alarms.last_entry.count");
  declare_parameter<std::string>("alarm_id_symbol", "GVL_Alarms.last_entry.alarm_id");
  declare_parameter<std::string>("source_symbol", "GVL_Alarms.last_entry.source");
  declare_parameter<std::string>("severity_symbol", "GVL_Alarms.last_entry.severity");
  declare_parameter<std::string>("count_symbol", "GVL_Alarms.last_entry.count");
  declare_parameter<std::string>("timestamp_symbol", "GVL_Alarms.last_entry.timestamp");
  declare_parameter<std::string>("hardware_id", "plc_alarms");
  declare_parameter<double>("alarm_active_window_s", 30.0);

  plc_ip_address_ = get_parameter("plc_ip_address").as_string();
  plc_ams_net_id_str_ = get_parameter("plc_ams_net_id").as_string();
  local_ams_net_id_str_ = get_parameter("local_ams_net_id").as_string();
  plc_ams_port_ = static_cast<uint16_t>(get_parameter("plc_ams_port").as_int());
  trigger_symbol_ = get_parameter("trigger_symbol").as_string();
  alarm_id_symbol_ = get_parameter("alarm_id_symbol").as_string();
  source_symbol_ = get_parameter("source_symbol").as_string();
  severity_symbol_ = get_parameter("severity_symbol").as_string();
  count_symbol_ = get_parameter("count_symbol").as_string();
  timestamp_symbol_ = get_parameter("timestamp_symbol").as_string();
  hardware_id_ = get_parameter("hardware_id").as_string();
  alarm_active_window_s_ = get_parameter("alarm_active_window_s").as_double();

  if (plc_ip_address_.empty() || plc_ams_net_id_str_.empty() || local_ams_net_id_str_.empty()) {
    RCLCPP_FATAL(
      getLogger(),
      "plc_ip_address, plc_ams_net_id and local_ams_net_id are required parameters.");
    rclcpp::shutdown();
    return;
  }

  try {
    connect();
    resolveHandles();
  } catch (const AdsException & ex) {
    RCLCPP_FATAL(
      getLogger(), "ADS exception during startup: %s (error 0x%lX)", ex.what(),
      static_cast<long>(ex.errorCode));
    rclcpp::shutdown();
    return;
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(getLogger(), "Error during startup: %s", ex.what());
    rclcpp::shutdown();
    return;
  }

  g_instance = this;

  const AdsNotificationAttrib attrib{
    sizeof(uint64_t), ADSTRANS_SERVERONCHA, /*nMaxDelay*/ 0, {/*nCycleTime*/ 0}};
  notification_ = std::make_unique<AdsNotification>(
    *ads_device_, trigger_symbol_, attrib, &AlarmBridgeNode::onCountChanged, /*hUser*/ 0);

  worker_thread_ = std::thread(&AlarmBridgeNode::workerLoop, this);

  setupDiagnostics();

  RCLCPP_INFO(
    getLogger(), "AlarmBridgeNode ready. PLC %s (AMS %s:%u), watching %s.",
    plc_ip_address_.c_str(), plc_ams_net_id_str_.c_str(), plc_ams_port_,
    trigger_symbol_.c_str());
}

AlarmBridgeNode::~AlarmBridgeNode()
{
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    worker_stop_ = true;
  }
  worker_cv_.notify_all();
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }

  // Tear the notification down before clearing g_instance -- once cleared, a callback that is
  // somehow still in flight would dereference a null instance. Destroying notification_ here
  // (rather than waiting for implicit member teardown) guarantees no further callback fires
  // before g_instance is cleared and ads_device_ is released.
  notification_.reset();
  g_instance = nullptr;
}

void AlarmBridgeNode::connect()
{
  AmsNetId remote_net_id;
  if (sscanf(
        plc_ams_net_id_str_.c_str(), "%hhu.%hhu.%hhu.%hhu.%hhu.%hhu", &remote_net_id.b[0],
        &remote_net_id.b[1], &remote_net_id.b[2], &remote_net_id.b[3], &remote_net_id.b[4],
        &remote_net_id.b[5]) != 6) {
    throw std::runtime_error("Invalid format for 'plc_ams_net_id'. Expected 'x.x.x.x.x.x'.");
  }

  AmsNetId local_net_id;
  if (sscanf(
        local_ams_net_id_str_.c_str(), "%hhu.%hhu.%hhu.%hhu.%hhu.%hhu", &local_net_id.b[0],
        &local_net_id.b[1], &local_net_id.b[2], &local_net_id.b[3], &local_net_id.b[4],
        &local_net_id.b[5]) != 6) {
    throw std::runtime_error("Invalid format for 'local_ams_net_id'. Expected 'x.x.x.x.x.x'.");
  }

  bhf::ads::SetLocalAddress(local_net_id);
  ads_device_ = std::make_unique<AdsDevice>(plc_ip_address_, remote_net_id, plc_ams_port_);

  const AdsDeviceState state = ads_device_->GetState();
  RCLCPP_INFO(
    getLogger(), "Connected to PLC. ADS state: %d, device state: %d", state.ads, state.device);
}

void AlarmBridgeNode::resolveHandles()
{
  // AdsHandle's deleter (ResourceDeleter) holds a `const std::function<...>` member, which
  // makes AdsHandle -- and therefore std::optional<AdsHandle> -- move-CONSTRUCTIBLE but not
  // move-ASSIGNABLE. emplace() move-constructs into the optional's storage instead of going
  // through the (deleted) move-assignment operator that plain `=` would require.
  handle_alarm_id_.emplace(ads_device_->GetHandle(alarm_id_symbol_));
  handle_source_.emplace(ads_device_->GetHandle(source_symbol_));
  handle_severity_.emplace(ads_device_->GetHandle(severity_symbol_));
  handle_count_.emplace(ads_device_->GetHandle(count_symbol_));
  handle_timestamp_.emplace(ads_device_->GetHandle(timestamp_symbol_));
}

void AlarmBridgeNode::onCountChanged(
  const AmsAddr * /*addr*/, const AdsNotificationHeader * notification, uint32_t /*hUser*/)
{
  if (g_instance != nullptr) {
    g_instance->handleCountChanged(notification);
  }
}

void AlarmBridgeNode::handleCountChanged(const AdsNotificationHeader * notification)
{
  uint64_t new_count = 0;
  std::memcpy(
    &new_count, reinterpret_cast<const uint8_t *>(notification) + sizeof(AdsNotificationHeader),
    std::min<size_t>(sizeof(new_count), notification->cbSampleSize));
  notified_count_.store(new_count, std::memory_order_release);

  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    worker_wake_pending_ = true;
  }
  worker_cv_.notify_one();
}

void AlarmBridgeNode::workerLoop()
{
  while (true) {
    std::unique_lock<std::mutex> lock(worker_mutex_);
    worker_cv_.wait(lock, [this] { return worker_wake_pending_ || worker_stop_; });
    if (worker_stop_) {
      break;
    }
    worker_wake_pending_ = false;
    lock.unlock();

    const uint64_t expected_count = notified_count_.load(std::memory_order_acquire);
    LatestAlarm snapshot;
    if (readAlarmSnapshot(expected_count, snapshot)) {
      snapshot.read_time = this->now();
      std::lock_guard<std::mutex> latest_lock(latest_mutex_);
      latest_ = snapshot;
    }
  }
}

bool AlarmBridgeNode::readAlarmSnapshot(uint64_t expected_count, LatestAlarm & out)
{
  // AdsDevice::ReadReqEx2 (used directly) returns a raw ADS error code rather than throwing --
  // AdsVariable<T>::Read() is the layer that turns that into a thrown AdsException (and also
  // checks the returned byte count matches sizeof(T)), so it is used here instead of raw
  // ReadReqEx2 calls specifically so a failed/short read cannot be silently mistaken for a
  // successful zero-filled one. The (group, offset) constructor just wraps the already-resolved
  // numeric handle -- it performs no further ADS round trip of its own.

  try {
    AdsVariable<uint32_t> var(*ads_device_, ADSIGRP_SYM_VALBYHND, **handle_alarm_id_);
    out.alarm_id = static_cast<uint32_t>(var);
  } catch (const AdsException & ex) {
    RCLCPP_WARN(getLogger(), "Failed reading %s: %s", alarm_id_symbol_.c_str(), ex.what());
    return false;
  }

  try {
    AdsVariable<std::array<char, 65>> var(*ads_device_, ADSIGRP_SYM_VALBYHND, **handle_source_);
    const std::array<char, 65> raw = var;
    const size_t len = strnlen(raw.data(), raw.size());
    out.source.assign(raw.data(), len);
  } catch (const AdsException & ex) {
    RCLCPP_WARN(getLogger(), "Failed reading %s: %s", source_symbol_.c_str(), ex.what());
    return false;
  }

  try {
    // E_AlarmSeverity declares no explicit base type, so TwinCAT's IEC 61131-3 default (INT,
    // 16-bit) applies. Verified against a live read during commissioning, not from a compiled
    // .tmc (none is checked into the PLC project repo).
    AdsVariable<int16_t> var(*ads_device_, ADSIGRP_SYM_VALBYHND, **handle_severity_);
    out.severity = static_cast<int16_t>(var);
  } catch (const AdsException & ex) {
    RCLCPP_WARN(getLogger(), "Failed reading %s: %s", severity_symbol_.c_str(), ex.what());
    return false;
  }

  try {
    // T_FILETIME is a Win32 FILETIME: two consecutive DWORDs, dwLowDateTime then
    // dwHighDateTime.
    AdsVariable<std::array<uint8_t, 8>> var(
      *ads_device_, ADSIGRP_SYM_VALBYHND, **handle_timestamp_);
    const std::array<uint8_t, 8> raw = var;
    uint32_t low = 0, high = 0;
    std::memcpy(&low, raw.data(), sizeof(low));
    std::memcpy(&high, raw.data() + sizeof(low), sizeof(high));
    out.timestamp_utc = filetimeToUnixEpoch(low, high);
  } catch (const AdsException & ex) {
    RCLCPP_WARN(getLogger(), "Failed reading %s: %s", timestamp_symbol_.c_str(), ex.what());
    return false;
  }

  try {
    AdsVariable<uint64_t> var(*ads_device_, ADSIGRP_SYM_VALBYHND, **handle_count_);
    const uint64_t count_after = static_cast<uint64_t>(var);
    out.count = count_after;
    if (count_after != expected_count) {
      RCLCPP_WARN(
        getLogger(),
        "Torn read: %s changed from %lu to %lu while reading this alarm snapshot; discarding.",
        count_symbol_.c_str(), static_cast<unsigned long>(expected_count),
        static_cast<unsigned long>(count_after));
      return false;
    }
  } catch (const AdsException & ex) {
    RCLCPP_WARN(getLogger(), "Failed reading %s: %s", count_symbol_.c_str(), ex.what());
    return false;
  }

  return true;
}

void AlarmBridgeNode::setupDiagnostics()
{
  updater_.setHardwareID(hardware_id_);
  updater_.add(hardware_id_, this, &AlarmBridgeNode::produceDiagnostics);
}

void AlarmBridgeNode::produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::optional<LatestAlarm> alarm;
  {
    std::lock_guard<std::mutex> lock(latest_mutex_);
    alarm = latest_;
  }

  if (!alarm.has_value()) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "No alarms received yet");
    return;
  }

  const double age_s = (this->now() - alarm->read_time).seconds();

  stat.add("alarm_id", alarm->alarm_id);
  stat.add("source", alarm->source);
  stat.add("severity", alarm->severity);
  stat.add("count", alarm->count);
  stat.add("last_alarm_utc", alarm->timestamp_utc);
  stat.add("age_s", age_s);

  if (age_s >= alarm_active_window_s_) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      "No active alarm (last: " + std::to_string(alarm->alarm_id) + " " +
      std::to_string(static_cast<int>(age_s)) + "s ago)");
    return;
  }

  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  if (alarm->severity >= 40) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  } else if (alarm->severity >= 30) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  }

  stat.summary(
    level, "alarm " + std::to_string(alarm->alarm_id) + " from " + alarm->source + " (" +
    std::to_string(static_cast<int>(age_s)) + "s ago)");
}

}  // namespace beckhoff_ads_alarm_bridge
