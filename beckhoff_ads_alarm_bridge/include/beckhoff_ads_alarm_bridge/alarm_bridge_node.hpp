#ifndef BECKHOFF_ADS_ALARM_BRIDGE__ALARM_BRIDGE_NODE_HPP_
#define BECKHOFF_ADS_ALARM_BRIDGE__ALARM_BRIDGE_NODE_HPP_

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"

#include "ads/AdsLib.h"
#include "ads/AdsDevice.h"
#include "ads/AdsNotificationOOI.h"

namespace beckhoff_ads_alarm_bridge
{

struct LatestAlarm
{
  uint32_t alarm_id{0};
  std::string source;
  int16_t severity{0};
  uint64_t count{0};
  double timestamp_utc{0.0};   // seconds since Unix epoch, decoded from the PLC's T_FILETIME
  rclcpp::Time read_time;      // local wall-clock time this snapshot was read, for the decay window
};

/**
 * Bridges a TwinCAT PLC alarm struct (default: GVL_Alarms.last_entry) into a
 * diagnostic_updater DiagnosticStatus over an ADS device notification.
 *
 * Symbol names are parameters, not hardcoded, so this stays reusable across any
 * TwinCAT project exposing a similarly-shaped {alarm_id, source, severity, count}
 * alarm record -- mirrors how beckhoff_ads_hardware_interface takes PLC_symbol as
 * external config rather than hardcoding a project's symbol names.
 */
class AlarmBridgeNode : public rclcpp::Node
{
public:
  explicit AlarmBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~AlarmBridgeNode() override;

private:
  rclcpp::Logger getLogger() { return rclcpp::get_logger("AlarmBridgeNode"); }

  // Connects ads_device_ from the plc_ip_address/plc_ams_net_id/plc_ams_port/local_ams_net_id
  // parameters and confirms reachability via GetState(). Throws on failure; the constructor
  // catches and shuts the node down rather than leaving it half-configured.
  void connect();

  // Resolves every leaf symbol handle once at startup so a typo'd symbol parameter fails
  // loudly at launch instead of silently on the first real alarm.
  void resolveHandles();

  // Static trampoline: hUser in the vendored ADS API is a uint32_t, not pointer-width, so it
  // cannot carry `this` on a 64-bit build. One process runs exactly one instance, so a static
  // pointer (set before the notification is registered, cleared before it is torn down) stands
  // in for hUser instead.
  static void onCountChanged(
    const AmsAddr * addr, const AdsNotificationHeader * notification, uint32_t hUser);

  // Instance-side handler invoked by onCountChanged. Does the minimum possible: stash the new
  // count and wake the worker thread. Must never perform a blocking ADS call -- it runs on the
  // library's shared notification-dispatch thread.
  void handleCountChanged(const AdsNotificationHeader * notification);

  // Worker thread body: waits for handleCountChanged() to wake it, then does the actual
  // synchronous ADS reads and publishes into latest_.
  void workerLoop();

  // Reads the 5 leaf symbols for one alarm snapshot. Returns false (logging why) if any read
  // fails or if count changed again mid-read (a torn read -- a second alarm landed while this
  // one was still being assembled).
  bool readAlarmSnapshot(uint64_t expected_count, LatestAlarm & out);

  void setupDiagnostics();
  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // === Parameters ===
  std::string plc_ip_address_;
  std::string plc_ams_net_id_str_;
  std::string local_ams_net_id_str_;
  uint16_t plc_ams_port_{851};
  std::string trigger_symbol_;
  std::string alarm_id_symbol_;
  std::string source_symbol_;
  std::string severity_symbol_;
  std::string count_symbol_;
  std::string timestamp_symbol_;
  std::string hardware_id_;
  double alarm_active_window_s_{30.0};

  // === ADS connection ===
  // Declared first: AdsHandle/AdsNotification deleters capture a raw pointer back into
  // ads_device_, so those members must be destroyed before it is -- C++ tears down members in
  // reverse declaration order, so ads_device_ must be declared before them.
  std::unique_ptr<AdsDevice> ads_device_;
  std::optional<AdsHandle> handle_alarm_id_;
  std::optional<AdsHandle> handle_source_;
  std::optional<AdsHandle> handle_severity_;
  std::optional<AdsHandle> handle_count_;
  std::optional<AdsHandle> handle_timestamp_;
  std::unique_ptr<AdsNotification> notification_;

  // === Worker thread / notification handoff ===
  std::thread worker_thread_;
  std::mutex worker_mutex_;
  std::condition_variable worker_cv_;
  bool worker_stop_{false};
  bool worker_wake_pending_{false};
  std::atomic<uint64_t> notified_count_{0};

  // === Latest decoded alarm, read by produceDiagnostics() on the ROS executor thread ===
  std::mutex latest_mutex_;
  std::optional<LatestAlarm> latest_;

  diagnostic_updater::Updater updater_;
};

}  // namespace beckhoff_ads_alarm_bridge

#endif  // BECKHOFF_ADS_ALARM_BRIDGE__ALARM_BRIDGE_NODE_HPP_
