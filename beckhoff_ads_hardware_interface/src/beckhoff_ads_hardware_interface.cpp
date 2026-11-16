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

#include "beckhoff_ads_hardware_interface/beckhoff_ads_hardware_interface.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <string_view>
#include <tuple>
#include <unordered_map>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace beckhoff_ads_hardware_interface
{
namespace
{

using beckhoff_ads_client::ClientSettings;
using beckhoff_ads_client::LogSeverity;
using beckhoff_ads_client::PlcType;
using beckhoff_ads_client::SymbolElement;

constexpr double NOT_PROVIDED = std::numeric_limits<double>::quiet_NaN();

struct RegisteredSymbol
{
  PlcType type{PlcType::Lreal};
  std::size_t element_count{1};
};

struct InterfacePlcParameters
{
  std::string symbol;
  std::string type_text;
  std::size_t element_count{1};
  std::size_t element_index{0};
  bool optional{false};
};

InterfacePlcParameters read_interface_plc_parameters(const hardware_interface::InterfaceDescription & description)
{
  const auto & parameters = description.interface_info.parameters;
  InterfacePlcParameters result;
  result.symbol = parameters.at("PLC_symbol");
  result.type_text = parameters.at("PLC_type");
  if (parameters.count("n_elements")) {
    result.element_count = std::stoul(parameters.at("n_elements"));
  }
  if (parameters.count("index")) {
    result.element_index = std::stoul(parameters.at("index"));
  }
  result.optional = parameters.count("optional") && parameters.at("optional") == "true";
  return result;
}

bool is_joint_motion_interface(const hardware_interface::InterfaceDescription & description, bool is_joint)
{
  return is_joint && (description.interface_info.name == hardware_interface::HW_IF_POSITION ||
         description.interface_info.name == hardware_interface::HW_IF_VELOCITY);
}

}  // namespace

BeckhoffADSHardwareInterface::~BeckhoffADSHardwareInterface() = default;

hardware_interface::CallbackReturn BeckhoffADSHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  logging_throttle_clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  return CallbackReturn::SUCCESS;
}

std::optional<std::string> BeckhoffADSHardwareInterface::hardware_parameter(const std::string & name) const
{
  const auto found = info_.hardware_parameters.find(name);
  return found != info_.hardware_parameters.end() ? std::optional<std::string>(found->second) : std::nullopt;
}

std::optional<ClientSettings> BeckhoffADSHardwareInterface::parse_client_settings()
{
  ClientSettings settings;
  const rclcpp::Logger logger = getLogger();
  settings.log = [logger](LogSeverity severity, std::string_view message) {
      const int length = static_cast<int>(message.size());
      switch (severity) {
        case LogSeverity::Debug:
          RCLCPP_DEBUG(logger, "%.*s", length, message.data());
          break;
        case LogSeverity::Info:
          RCLCPP_INFO(logger, "%.*s", length, message.data());
          break;
        case LogSeverity::Warning:
          RCLCPP_WARN(logger, "%.*s", length, message.data());
          break;
        case LogSeverity::Error:
          RCLCPP_ERROR(logger, "%.*s", length, message.data());
          break;
        case LogSeverity::Fatal:
          RCLCPP_FATAL(logger, "%.*s", length, message.data());
          break;
      }
    };

  auto number_parameter = [this](const std::string & name, const std::string & fallback_note) {
      std::optional<double> result;
      const std::optional<std::string> text = hardware_parameter(name);
      if (text) {
        try {
          result = std::stod(*text);
        } catch (const std::exception & exception) {
          RCLCPP_WARN(getLogger(), "Invalid %s: %s. %s", name.c_str(), exception.what(), fallback_note.c_str());
        }
      }
      return result;
    };
  auto milliseconds_to_nanoseconds = [](double milliseconds) {
      return std::chrono::nanoseconds(static_cast<long long>(milliseconds * 1e6));
    };

  const std::optional<double> poll_period = number_parameter("read_poll_period_ms", "Running unpaced.");
  if (poll_period && std::isfinite(*poll_period) && *poll_period > 0.0) {
    settings.read_poll_period = milliseconds_to_nanoseconds(*poll_period);
  }

  const std::optional<double> grace = number_parameter("comms_outage_grace_ms", "Using default 100 ms.");
  if (grace && std::isfinite(*grace) && *grace >= 0.0) {
    settings.outage_grace = std::chrono::milliseconds(static_cast<long long>(*grace));
  }

  const std::optional<std::string> outage_behaviour = hardware_parameter("outage_behaviour");
  if (outage_behaviour && *outage_behaviour == "resume") {
    settings.outage_behaviour = beckhoff_ads_client::OutageBehaviour::Resume;
  } else if (outage_behaviour && *outage_behaviour != "latch") {
    RCLCPP_WARN(
      getLogger(), "Unknown outage_behaviour '%s'. Expected latch or resume. Latching.", outage_behaviour->c_str());
  }

  const std::optional<double> staleness = number_parameter("read_staleness_timeout_ms", "Using default 100 ms.");
  if (staleness && std::isfinite(*staleness) && *staleness >= 0.0) {
    settings.read_staleness_timeout = milliseconds_to_nanoseconds(*staleness);
  }
  read_staleness_timeout_ = settings.read_staleness_timeout;
  if (read_staleness_timeout_.count() > 0 && settings.read_poll_period.count() > 0) {
    read_staleness_timeout_ = std::max(read_staleness_timeout_, 3 * settings.read_poll_period);
  }

  activation_first_sample_timeout_ = std::chrono::milliseconds(1000);
  const std::optional<double> activation_timeout =
    number_parameter("activation_first_sample_timeout_ms", "Using default 1000 ms.");
  if (activation_timeout && std::isfinite(*activation_timeout) && *activation_timeout >= 0.0) {
    activation_first_sample_timeout_ = std::chrono::milliseconds(static_cast<long long>(*activation_timeout));
  }

  const beckhoff_ads_client::utilities::ThreadSchedulingParseResult scheduling =
    beckhoff_ads_client::utilities::parseThreadScheduling(
    hardware_parameter("io_thread_scheduling_policy").value_or(""),
    hardware_parameter("io_thread_priority").value_or(""),
    hardware_parameter("io_thread_cpu_affinity").value_or(""));
  if (!scheduling.valid) {
    RCLCPP_WARN(
      getLogger(), "Invalid I/O thread scheduling parameters: %s. Using the defaults.", scheduling.error.c_str());
  }
  settings.io_thread_scheduling = scheduling.config;

  settings.heartbeat_symbol = hardware_parameter("heartbeat_plc_symbol").value_or("");
  if (!settings.heartbeat_symbol.empty()) {
    RCLCPP_INFO(getLogger(), "ADS link heartbeat enabled on PLC symbol '%s'.", settings.heartbeat_symbol.c_str());
  }
  settings.setpoint_sequence_symbol = hardware_parameter("setpoint_sequence_plc_symbol").value_or("");
  if (!settings.setpoint_sequence_symbol.empty()) {
    RCLCPP_INFO(
      getLogger(), "Setpoint sequence enabled on PLC symbol '%s'.", settings.setpoint_sequence_symbol.c_str());
  }
  settings.setpoint_timestamp_symbol = hardware_parameter("setpoint_time_plc_symbol").value_or("");
  if (!settings.setpoint_timestamp_symbol.empty()) {
    RCLCPP_INFO(
      getLogger(), "Setpoint timestamp enabled on PLC symbol '%s'.", settings.setpoint_timestamp_symbol.c_str());
  }

  const std::optional<double> request_timeout = number_parameter("ads_request_timeout_ms", "Using default 100 ms.");
  if (request_timeout) {
    settings.connection.request_timeout = std::chrono::milliseconds(
      std::isfinite(*request_timeout) && *request_timeout >= 0.0 ? static_cast<std::uint32_t>(*request_timeout) : 100);
  }

  const std::array<std::string, 4> required_parameters{
    "plc_ip_address", "plc_ams_net_id", "local_ams_net_id", "plc_ams_port"};
  for (const std::string & name : required_parameters) {
    if (!hardware_parameter(name)) {
      RCLCPP_FATAL(getLogger(), "\tMissing required URDF <hardware> parameter: %s", name.c_str());
      return std::nullopt;
    }
  }
  settings.connection.plc_ip_address = *hardware_parameter("plc_ip_address");
  settings.connection.plc_ams_net_id = *hardware_parameter("plc_ams_net_id");
  settings.connection.local_ams_net_id = *hardware_parameter("local_ams_net_id");
  const std::string port_text = *hardware_parameter("plc_ams_port");
  const beckhoff_ads_client::utilities::AmsPortParseResult port = beckhoff_ads_client::utilities::parseAmsPort(port_text);
  if (!port.valid) {
    RCLCPP_FATAL(getLogger(), "\tInvalid 'plc_ams_port' '%s': %s.", port_text.c_str(), port.error.c_str());
    return std::nullopt;
  }
  settings.connection.plc_ams_port = port.port;
  return settings;
}

void BeckhoffADSHardwareInterface::plan_state_elements(std::vector<SymbolElement> & elements)
{
  std::map<std::string, RegisteredSymbol> registered_symbols;
  auto plan = [&](const std::unordered_map<std::string, hardware_interface::InterfaceDescription> & interfaces,
      bool is_joint) {
      for (const auto & [name, description] : interfaces) {
        InterfacePlcParameters plc;
        try {
          plc = read_interface_plc_parameters(description);
        } catch (const std::exception & exception) {
          RCLCPP_ERROR(
            getLogger(), "Error parsing PLC parameters for state interface '%s': %s. Check URDF.", name.c_str(),
            exception.what());
          continue;
        }
        bool critical = is_joint_motion_interface(description, is_joint);
        if (description.interface_info.parameters.count("critical")) {
          critical = description.interface_info.parameters.at("critical") == "true";
        }
        warn_if_joint_motion_interface_is_32_bit(name, description, is_joint, plc.type_text);

        auto registered = registered_symbols.find(plc.symbol);
        if (registered == registered_symbols.end()) {
          const std::optional<PlcType> type = beckhoff_ads_client::parse_plc_type(plc.type_text);
          if (!type) {
            RCLCPP_ERROR(
              getLogger(), "Skipping state variable '%s' due to UNSUPPORTED or UNKNOWN PLC_type '%s'.",
              plc.symbol.c_str(), plc.type_text.c_str());
            continue;
          }
          registered = registered_symbols.emplace(plc.symbol, RegisteredSymbol{*type, plc.element_count}).first;
        }
        SymbolElement element;
        element.symbol = plc.symbol;
        element.type = registered->second.type;
        element.element_count = registered->second.element_count;
        element.element_index = plc.element_index;
        element.optional = plc.optional;
        element.critical = critical;
        element.label = name;
        elements.push_back(std::move(element));
      }
    };
  plan(joint_state_interfaces_, true);
  plan(gpio_state_interfaces_, false);
  plan(sensor_state_interfaces_, false);
}

void BeckhoffADSHardwareInterface::plan_command_elements(
  std::vector<SymbolElement> & elements, std::vector<CommandFallback> & fallbacks)
{
  std::map<std::string, RegisteredSymbol> registered_symbols;
  auto plan = [&](const std::unordered_map<std::string, hardware_interface::InterfaceDescription> & interfaces,
      bool is_joint) {
      for (const auto & [name, description] : interfaces) {
        InterfacePlcParameters plc;
        try {
          plc = read_interface_plc_parameters(description);
        } catch (const std::exception & exception) {
          RCLCPP_ERROR(
            getLogger(), "Error parsing PLC parameters for command interface '%s': %s. Check URDF.", name.c_str(),
            exception.what());
          continue;
        }
        std::string fallback_text;
        if (description.interface_info.parameters.count("command_fallback")) {
          fallback_text = description.interface_info.parameters.at("command_fallback");
        }
        warn_if_joint_motion_interface_is_32_bit(name, description, is_joint, plc.type_text);

        CommandFallback fallback = CommandFallback::HOLD_LAST;
        if (fallback_text.empty() && is_joint && description.interface_info.name == hardware_interface::HW_IF_VELOCITY) {
          fallback = CommandFallback::ZERO;
          RCLCPP_INFO(
            getLogger(),
            "Joint velocity command interface '%s' defaults to a zero fallback. "
            "Set command_fallback to hold_last or mirror_state to override.",
            name.c_str());
        } else {
          fallback = parseCommandFallback(fallback_text, name);
        }

        auto registered = registered_symbols.find(plc.symbol);
        if (registered == registered_symbols.end()) {
          const std::optional<PlcType> type = beckhoff_ads_client::parse_plc_type(plc.type_text);
          if (!type) {
            RCLCPP_ERROR(
              getLogger(), "Skipping command variable '%s' due to UNSUPPORTED or UNKNOWN PLC_type '%s'.",
              plc.symbol.c_str(), plc.type_text.c_str());
            continue;
          }
          registered = registered_symbols.emplace(plc.symbol, RegisteredSymbol{*type, plc.element_count}).first;
        }
        SymbolElement element;
        element.symbol = plc.symbol;
        element.type = registered->second.type;
        element.element_count = registered->second.element_count;
        element.element_index = plc.element_index;
        element.optional = plc.optional;
        element.label = name;
        double initial_value = NOT_PROVIDED;
        std::ignore = get_command(get_command_interface_handle(name), initial_value, true);
        if (std::isfinite(initial_value)) {
          element.initial_value = initial_value;
        }
        elements.push_back(std::move(element));
        fallbacks.push_back(fallback);
      }
    };
  plan(joint_command_interfaces_, true);
  plan(gpio_command_interfaces_, false);
}

hardware_interface::CallbackReturn BeckhoffADSHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  ads_client_.disconnect();
  state_bindings_.clear();
  command_bindings_.clear();

  const std::optional<ClientSettings> settings = parse_client_settings();
  if (!settings) {
    RCLCPP_FATAL(getLogger(), "Failed to configure ADS device from URDF parameters.");
    return CallbackReturn::ERROR;
  }

  std::vector<SymbolElement> state_elements;
  std::vector<SymbolElement> command_elements;
  std::vector<CommandFallback> fallbacks;
  plan_state_elements(state_elements);
  plan_command_elements(command_elements, fallbacks);

  const beckhoff_ads_client::ConfigureResult result =
    ads_client_.configure(*settings, state_elements, command_elements);
  if (result.failure == beckhoff_ads_client::ConfigureFailure::InvalidSettings ||
    result.failure == beckhoff_ads_client::ConfigureFailure::ConnectionFailed)
  {
    RCLCPP_FATAL(getLogger(), "Failed to configure ADS device from URDF parameters.");
    return CallbackReturn::ERROR;
  }
  if (!result.succeeded()) {
    return CallbackReturn::ERROR;
  }

  bind_interfaces(state_elements, command_elements, fallbacks, result);
  return CallbackReturn::SUCCESS;
}

void BeckhoffADSHardwareInterface::bind_interfaces(
  const std::vector<SymbolElement> & state_elements, const std::vector<SymbolElement> & command_elements,
  const std::vector<CommandFallback> & fallbacks, const beckhoff_ads_client::ConfigureResult & result)
{
  auto dropped = [](const std::vector<std::size_t> & dropped_elements, std::size_t index) {
      return std::binary_search(dropped_elements.begin(), dropped_elements.end(), index);
    };

  for (std::size_t index = 0; index < state_elements.size(); ++index) {
    StateBinding binding;
    binding.interface_name = state_elements[index].label;
    binding.available = !dropped(result.dropped_read_elements, index);
    if (binding.available) {
      binding.handle = get_state_interface_handle(binding.interface_name);
    } else if (!stateInterfaceHasDeclaredInitialValue(binding.interface_name)) {
      set_state(binding.interface_name, 0.0);
    }
    state_bindings_.push_back(std::move(binding));
  }

  RCLCPP_INFO(getLogger(), "Linking command interfaces to state interfaces...");
  for (std::size_t index = 0; index < command_elements.size(); ++index) {
    const SymbolElement & element = command_elements[index];
    CommandBinding binding;
    binding.interface_name = element.label;
    binding.fallback = fallbacks[index];
    binding.available = !dropped(result.dropped_write_elements, index);
    if (binding.available) {
      binding.handle = get_command_interface_handle(binding.interface_name);
      if (binding.fallback == CommandFallback::MIRROR_STATE) {
        const auto mirrored = std::find_if(
          state_elements.begin(), state_elements.end(), [&](const SymbolElement & state) {
            return state.symbol == element.symbol && state.element_index == element.element_index;
          });
        const std::size_t mirrored_index = static_cast<std::size_t>(mirrored - state_elements.begin());
        if (mirrored != state_elements.end() && state_bindings_[mirrored_index].available) {
          binding.fallback_state_handle = state_bindings_[mirrored_index].handle;
        } else {
          RCLCPP_WARN(
            getLogger(),
            "Command interface '%s' asks for a mirror_state fallback but no state interface "
            "shares its PLC symbol. Holding the last value instead.",
            binding.interface_name.c_str());
          binding.fallback = CommandFallback::HOLD_LAST;
        }
      }
    }
    command_bindings_.push_back(std::move(binding));
  }

  state_values_.assign(state_bindings_.size(), NOT_PROVIDED);
  command_values_.assign(command_bindings_.size(), NOT_PROVIDED);
}

bool BeckhoffADSHardwareInterface::stateInterfaceHasDeclaredInitialValue(const std::string & interface_name) const
{
  bool result = false;
  for (const auto & interfaces : {std::cref(joint_state_interfaces_), std::cref(gpio_state_interfaces_),
      std::cref(sensor_state_interfaces_)})
  {
    const auto found = interfaces.get().find(interface_name);
    if (found != interfaces.get().end() && !found->second.interface_info.initial_value.empty()) {
      result = true;
    }
  }
  return result;
}

void BeckhoffADSHardwareInterface::warn_if_joint_motion_interface_is_32_bit(
  const std::string & interface_name, const hardware_interface::InterfaceDescription & description, bool is_joint,
  const std::string & plc_type_str)
{
  if (is_joint_motion_interface(description, is_joint) &&
    beckhoff_ads_client::utilities::toUpperCopy(plc_type_str) == "REAL")
  {
    RCLCPP_WARN(
      getLogger(),
      "Joint interface '%s' is declared as 32-bit REAL. A position or velocity "
      "stream loses precision through a float; declare it LREAL on the PLC and here.",
      interface_name.c_str());
  }
}

CommandFallback BeckhoffADSHardwareInterface::parseCommandFallback(
  const std::string & policy_str, const std::string & interface_name)
{
  CommandFallback result = CommandFallback::HOLD_LAST;
  if (policy_str == "mirror_state") {
    result = CommandFallback::MIRROR_STATE;
  } else if (policy_str == "zero") {
    result = CommandFallback::ZERO;
  } else if (!policy_str.empty() && policy_str != "hold_last") {
    RCLCPP_WARN(
      getLogger(),
      "Unknown command_fallback '%s' on interface '%s'. Expected hold_last, mirror_state or zero. "
      "Holding the last value.",
      policy_str.c_str(), interface_name.c_str());
  }
  return result;
}

hardware_interface::CallbackReturn BeckhoffADSHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  register_transaction_statistics();
  if (!ads_client_.start()) {
    RCLCPP_ERROR(getLogger(), "Activation failed: the ADS client is not configured.");
    return CallbackReturn::FAILURE;
  }

  CallbackReturn result = CallbackReturn::SUCCESS;
  if (ads_client_.read_item_count() > 0 && activation_first_sample_timeout_.count() > 0) {
    const beckhoff_ads_client::FirstSampleOutcome outcome =
      ads_client_.wait_for_first_sample(activation_first_sample_timeout_);
    if (outcome == beckhoff_ads_client::FirstSampleOutcome::LinkFaulted) {
      RCLCPP_ERROR(getLogger(), "Activation failed: the ADS link faulted before the first sample.");
      result = CallbackReturn::FAILURE;
    } else if (outcome == beckhoff_ads_client::FirstSampleOutcome::TimedOut) {
      RCLCPP_ERROR(
        getLogger(), "Activation failed: no PLC sample arrived within %ld ms.",
        static_cast<long>(activation_first_sample_timeout_.count()));
      result = CallbackReturn::FAILURE;
    }
    if (result != CallbackReturn::SUCCESS) {
      ads_client_.stop();
    }
  }
  return result;
}

hardware_interface::CallbackReturn BeckhoffADSHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  ads_client_.stop();
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BeckhoffADSHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  teardown_ads_client();
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BeckhoffADSHardwareInterface::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(getLogger(), "ADS component entering the error state. Dropping the link.");
  teardown_ads_client();
  return CallbackReturn::SUCCESS;
}

void BeckhoffADSHardwareInterface::teardown_ads_client()
{
  RCLCPP_INFO(getLogger(), "Releasing ADS resources...");
  ads_client_.disconnect();
  RCLCPP_INFO(getLogger(), "ADS resources released.");
}

hardware_interface::return_type BeckhoffADSHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (ads_client_.read_item_count() == 0) {
    return hardware_interface::return_type::OK;
  }

  const std::optional<beckhoff_ads_client::SampleInfo> sample = ads_client_.latest_sample(state_values_);
  if (!sample) {
    std::fill(state_values_.begin(), state_values_.end(), NOT_PROVIDED);
  }
  for (std::size_t index = 0; index < state_bindings_.size(); ++index) {
    if (state_bindings_[index].available) {
      std::ignore = set_state(state_bindings_[index].handle, state_values_[index], false);
    }
  }

  stat_read_sample_age_ms_ = sample ? static_cast<double>(sample->age.count()) * 1e-6 : NOT_PROVIDED;
  const bool stale = sample && sample->stale;
  if (stale) {
    RCLCPP_ERROR_THROTTLE(
      getLogger(), *logging_throttle_clock_, 1000,
      "The published PLC state sample is %.1f ms old, over the %.1f ms staleness "
      "timeout. Surfacing an error rather than frozen feedback.",
      stat_read_sample_age_ms_, static_cast<double>(read_staleness_timeout_.count()) * 1e-6);
  }

  return (stale || ads_client_.read_faulted()) ? hardware_interface::return_type::ERROR :
         hardware_interface::return_type::OK;
}

hardware_interface::return_type BeckhoffADSHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (ads_client_.write_item_count() == 0) {
    return hardware_interface::return_type::OK;
  }

  std::uint64_t fallbacks_this_cycle = 0;
  std::uint64_t never_commanded_this_cycle = 0;
  std::optional<std::size_t> first_fallback_binding;

  for (std::size_t index = 0; index < command_bindings_.size(); ++index) {
    CommandBinding & binding = command_bindings_[index];
    double value = NOT_PROVIDED;
    if (binding.available) {
      if (get_command(binding.handle, value, false)) {
        std::ignore = set_command(binding.handle, NOT_PROVIDED, false);
      }
      if (!std::isnan(value)) {
        binding.has_been_commanded = true;
      } else {
        if (binding.has_been_commanded) {
          ++fallbacks_this_cycle;
          ++fallback_activations_;
          if (!first_fallback_binding) {
            first_fallback_binding = index;
          }
        } else {
          ++never_commanded_this_cycle;
        }
        if (binding.fallback == CommandFallback::ZERO) {
          value = 0.0;
        } else if (binding.fallback == CommandFallback::MIRROR_STATE && binding.fallback_state_handle) {
          std::ignore = get_state(binding.fallback_state_handle, value, false);
        }
      }
    }
    command_values_[index] = value;
  }

  if (first_fallback_binding) {
    RCLCPP_WARN_THROTTLE(
      getLogger(), *logging_throttle_clock_, 5000,
      "%lu command interfaces carried no command this cycle after having been "
      "commanded before ('%s' among them). Applying their fallbacks.",
      static_cast<unsigned long>(fallbacks_this_cycle),
      command_bindings_[*first_fallback_binding].interface_name.c_str());
  }

  ads_client_.submit(command_values_);

  stat_fallback_activations_per_cycle_ = static_cast<double>(fallbacks_this_cycle);
  stat_never_commanded_interfaces_ = static_cast<double>(never_commanded_this_cycle);
  refresh_transaction_statistics();

  return ads_client_.write_faulted() ? hardware_interface::return_type::ERROR :
         hardware_interface::return_type::OK;
}

void BeckhoffADSHardwareInterface::refresh_transaction_statistics()
{
  const beckhoff_ads_client::TransactionStatistics statistics = ads_client_.statistics();
  auto milliseconds = [](std::chrono::nanoseconds duration) {return static_cast<double>(duration.count()) * 1e-6;};
  stat_read_rtt_ms_ = milliseconds(statistics.read_round_trip);
  stat_write_rtt_ms_ = milliseconds(statistics.write_round_trip);
  stat_write_buffer_age_ms_ = milliseconds(statistics.write_buffer_age);
  stat_write_handoff_latency_ms_ = milliseconds(statistics.write_handoff_latency);
  stat_read_transactions_ = static_cast<double>(statistics.read_transactions);
  stat_write_transactions_ = static_cast<double>(statistics.write_transactions);
  stat_write_coalesced_ = static_cast<double>(statistics.coalesced_writes);
  stat_read_failures_ = static_cast<double>(statistics.read_failures);
  stat_write_failures_ = static_cast<double>(statistics.write_failures);
  stat_fallback_activations_ = static_cast<double>(fallback_activations_);
  stat_heartbeat_ = static_cast<double>(statistics.heartbeat);
  stat_setpoint_sequence_ = static_cast<double>(statistics.setpoint_sequence);
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

}  // namespace beckhoff_ads_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  beckhoff_ads_hardware_interface::BeckhoffADSHardwareInterface, hardware_interface::SystemInterface)
