// Copyright (c) 2025, b-robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

// Author: Nikola Banovic
// Contributor: Hajar Bartakh

/**
 * @file beckhoff_ads_hardware_interface.hpp
 * @brief ros2_control system interface exchanging its state and command interfaces with a Beckhoff PLC.
 */
#ifndef beckhoff_ads_hardware_interface__BECKHOFF_SYSTEM_HPP_
#define beckhoff_ads_hardware_interface__BECKHOFF_SYSTEM_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "beckhoff_ads_client/ads_client.hpp"

namespace beckhoff_ads_hardware_interface
{

/** @brief What a command interface sends on a cycle no controller commanded it. */
enum class CommandFallback
{
  HOLD_LAST,
  MIRROR_STATE,
  ZERO,
};

/** @brief A state interface fed by one element of the client's latest sample. */
struct StateBinding
{
  /** @brief Full state interface name. */
  std::string interface_name;
  /** @brief Handle resolved at configure, so read() never looks it up by name. */
  hardware_interface::StateInterface::SharedPtr handle;
  /** @brief False when its optional PLC symbol was dropped. */
  bool available{true};
};

/** @brief A command interface feeding one element of the client's submitted values. */
struct CommandBinding
{
  /** @brief Full command interface name. */
  std::string interface_name;
  /** @brief Handle resolved at configure, so write() never looks it up by name. */
  hardware_interface::CommandInterface::SharedPtr handle;
  /** @brief Fallback applied on a cycle without a command. */
  CommandFallback fallback{CommandFallback::HOLD_LAST};
  /** @brief State interface sharing the PLC symbol element, for the mirror_state fallback. */
  hardware_interface::StateInterface::SharedPtr fallback_state_handle;
  /** @brief Whether a controller has ever commanded the interface. */
  bool has_been_commanded{false};
  /** @brief False when its optional PLC symbol was dropped. */
  bool available{true};
};

/**
 * @brief ros2_control adapter over beckhoff_ads_client::AdsClient.
 *
 * Maps every state and command interface carrying PLC_symbol and PLC_type parameters to one PLC
 * symbol element, configures the client from the hardware parameters, copies the latest sample into
 * the state interfaces in read() and submits the command interfaces, after their fallbacks, in write().
 */
class BeckhoffADSHardwareInterface : public hardware_interface::SystemInterface
{
public:
  /** @brief Stops the client if no lifecycle shutdown did. */
  ~BeckhoffADSHardwareInterface() override;

  /** @brief Initialises the base system interface and the log throttle clock. */
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;

  /** @brief Parses the parameters, connects the client and resolves every PLC symbol. */
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  /** @brief Starts the client and waits for the first PLC sample. */
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  /** @brief Stops the client's I/O threads. */
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  /** @brief Disconnects the client. */
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  /** @brief Disconnects the client; entered straight from an ERROR in read() or write(). */
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  /** @brief Copies the latest PLC sample into the state interfaces without blocking. */
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /** @brief Hands the commands, after their fallbacks, to the client without blocking. */
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  rclcpp::Logger getLogger() const {return rclcpp::get_logger("BeckhoffADSHardwareInterface");}

  std::optional<beckhoff_ads_client::ClientSettings> parse_client_settings();
  std::optional<std::string> hardware_parameter(const std::string & name) const;
  void plan_state_elements(std::vector<beckhoff_ads_client::SymbolElement> & elements);
  void plan_command_elements(
    std::vector<beckhoff_ads_client::SymbolElement> & elements, std::vector<CommandFallback> & fallbacks);
  CommandFallback parseCommandFallback(const std::string & policy_str, const std::string & interface_name);
  bool stateInterfaceHasDeclaredInitialValue(const std::string & interface_name) const;
  void warn_if_joint_motion_interface_is_32_bit(
    const std::string & interface_name, const hardware_interface::InterfaceDescription & description,
    bool is_joint, const std::string & plc_type_str);
  void bind_interfaces(
    const std::vector<beckhoff_ads_client::SymbolElement> & state_elements,
    const std::vector<beckhoff_ads_client::SymbolElement> & command_elements,
    const std::vector<CommandFallback> & fallbacks, const beckhoff_ads_client::ConfigureResult & result);
  void teardown_ads_client();
  void refresh_transaction_statistics();
  void register_transaction_statistics();

  std::shared_ptr<rclcpp::Clock> logging_throttle_clock_;
  beckhoff_ads_client::AdsClient ads_client_;
  std::chrono::milliseconds activation_first_sample_timeout_{1000};
  std::chrono::nanoseconds read_staleness_timeout_{std::chrono::milliseconds(100)};

  std::vector<StateBinding> state_bindings_;
  std::vector<CommandBinding> command_bindings_;
  std::vector<double> state_values_;
  std::vector<double> command_values_;

  std::uint64_t fallback_activations_{0};

  double stat_read_rtt_ms_{0.0};
  double stat_write_rtt_ms_{0.0};
  double stat_write_buffer_age_ms_{0.0};
  double stat_write_handoff_latency_ms_{0.0};
  double stat_read_transactions_{0.0};
  double stat_write_transactions_{0.0};
  double stat_write_coalesced_{0.0};
  double stat_read_failures_{0.0};
  double stat_write_failures_{0.0};
  double stat_fallback_activations_{0.0};
  double stat_fallback_activations_per_cycle_{0.0};
  double stat_never_commanded_interfaces_{0.0};
  double stat_heartbeat_{0.0};
  double stat_setpoint_sequence_{0.0};
  double stat_read_sample_age_ms_{0.0};
};

}  // namespace beckhoff_ads_hardware_interface

#endif  // beckhoff_ads_hardware_interface__BECKHOFF_SYSTEM_HPP_
