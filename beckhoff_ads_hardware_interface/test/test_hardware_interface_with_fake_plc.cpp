// Copyright (c) 2025, b-robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "beckhoff_ads_client/testing/fake_ads_server.hpp"
#include "beckhoff_ads_hardware_interface/beckhoff_ads_hardware_interface.hpp"
#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/system.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace
{

using lifecycle_msgs::msg::State;

std::string make_urdf(const std::string & diagnostic_optional)
{
  return R"(<?xml version="1.0"?>
<robot name="cell">
  <link name="world"/>
  <link name="carriage"/>
  <link name="slide"/>
  <joint name="x" type="prismatic">
    <parent link="world"/>
    <child link="carriage"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1" upper="1" effort="10" velocity="1"/>
  </joint>
  <joint name="y" type="prismatic">
    <parent link="carriage"/>
    <child link="slide"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1" upper="1" effort="10" velocity="1"/>
  </joint>
  <ros2_control name="plc" type="system">
    <hardware>
      <plugin>beckhoff_ads_hardware_interface/BeckhoffADSHardwareInterface</plugin>
      <param name="plc_ip_address">127.0.0.1</param>
      <param name="plc_ams_net_id">127.0.0.1.1.1</param>
      <param name="local_ams_net_id">127.0.0.2.1.1</param>
      <param name="plc_ams_port">851</param>
      <param name="ads_request_timeout_ms">50</param>
      <param name="io_thread_scheduling_policy">inherit</param>
      <param name="heartbeat_plc_symbol">MAIN.heartbeat</param>
    </hardware>
    <joint name="x">
      <command_interface name="position">
        <param name="PLC_symbol">MAIN.targets</param>
        <param name="PLC_type">LREAL</param>
        <param name="n_elements">2</param>
        <param name="index">0</param>
      </command_interface>
      <command_interface name="velocity">
        <param name="PLC_symbol">MAIN.velocity</param>
        <param name="PLC_type">LREAL</param>
      </command_interface>
      <state_interface name="position">
        <param name="PLC_symbol">MAIN.positions</param>
        <param name="PLC_type">LREAL</param>
        <param name="n_elements">2</param>
        <param name="index">0</param>
      </state_interface>
    </joint>
    <joint name="y">
      <command_interface name="position">
        <param name="PLC_symbol">MAIN.targets</param>
        <param name="PLC_type">LREAL</param>
        <param name="n_elements">2</param>
        <param name="index">1</param>
      </command_interface>
      <state_interface name="position">
        <param name="PLC_symbol">MAIN.positions</param>
        <param name="PLC_type">LREAL</param>
        <param name="n_elements">2</param>
        <param name="index">1</param>
      </state_interface>
    </joint>
    <gpio name="io">
      <command_interface name="mode">
        <param name="PLC_symbol">MAIN.mode</param>
        <param name="PLC_type">INT</param>
      </command_interface>
      <state_interface name="enabled">
        <param name="PLC_symbol">MAIN.enabled</param>
        <param name="PLC_type">BOOL</param>
      </state_interface>
      <state_interface name="diagnostic">
        <param name="PLC_symbol">MAIN.not_flashed_yet</param>
        <param name="PLC_type">DINT</param>
        <param name="optional">)" + diagnostic_optional + R"(</param>
      </state_interface>
    </gpio>
  </ros2_control>
</robot>)";
}

bool wait_until(const std::function<bool()> & condition)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  bool satisfied = condition();
  while (!satisfied && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    satisfied = condition();
  }
  return satisfied;
}

class HardwareInterfaceWithFakePlc : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ASSERT_TRUE(server_.listening());
    server_.add_symbol("MAIN.positions", 16);
    server_.add_symbol("MAIN.targets", 16);
    server_.add_symbol("MAIN.velocity", 8);
    server_.add_symbol("MAIN.mode", 2);
    server_.add_symbol("MAIN.enabled", 1);
    server_.add_symbol("MAIN.heartbeat", 4);
    server_.set_value<double>("MAIN.positions", 0, 1.5);
    server_.set_value<double>("MAIN.positions", 8, -2.5);
    server_.set_value<std::uint8_t>("MAIN.enabled", 0, 1);
  }

  const rclcpp_lifecycle::State & initialize(const std::string & diagnostic_optional)
  {
    hardware_interface::HardwareComponentParams params;
    params.hardware_info = hardware_interface::parse_control_resources_from_urdf(make_urdf(diagnostic_optional))[0];
    params.clock = clock_;
    params.logger = rclcpp::get_logger("test_hardware_interface_with_fake_plc");
    const rclcpp_lifecycle::State & state = system_.initialize(params);
    for (const auto & state_interface : system_.export_state_interfaces()) {
      state_interfaces_.push_back(state_interface);
    }
    for (const auto & command_interface : system_.export_command_interfaces()) {
      command_interfaces_.push_back(command_interface);
    }
    return state;
  }

  std::optional<double> state(const std::string & name) const
  {
    std::optional<double> result;
    for (const auto & state_interface : state_interfaces_) {
      if (state_interface->get_name() == name) {
        result = state_interface->get_optional<double>();
      }
    }
    return result;
  }

  void command(const std::string & name, double value)
  {
    for (const auto & command_interface : command_interfaces_) {
      if (command_interface->get_name() == name) {
        ASSERT_TRUE(command_interface->set_value(value));
      }
    }
  }

  hardware_interface::return_type read()
  {
    return system_.read(clock_->now(), rclcpp::Duration(std::chrono::milliseconds(2)));
  }

  hardware_interface::return_type write()
  {
    return system_.write(clock_->now(), rclcpp::Duration(std::chrono::milliseconds(2)));
  }

  beckhoff_ads_client::testing::FakeAdsServer server_;
  rclcpp::Clock::SharedPtr clock_{std::make_shared<rclcpp::Clock>()};
  hardware_interface::System system_{
    std::make_unique<beckhoff_ads_hardware_interface::BeckhoffADSHardwareInterface>()};
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces_;
};

}  // namespace

TEST_F(HardwareInterfaceWithFakePlc, ReadPublishesThePlcStateAndPinsADroppedOptionalStateToZero)
{
  ASSERT_EQ(initialize("true").id(), State::PRIMARY_STATE_UNCONFIGURED);
  ASSERT_EQ(system_.configure().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_DOUBLE_EQ(state("io/diagnostic").value_or(-1.0), 0.0);
  ASSERT_EQ(system_.activate().id(), State::PRIMARY_STATE_ACTIVE);

  ASSERT_EQ(read(), hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(state("x/position").value_or(0.0), 1.5);
  EXPECT_DOUBLE_EQ(state("y/position").value_or(0.0), -2.5);
  EXPECT_DOUBLE_EQ(state("io/enabled").value_or(0.0), 1.0);
  EXPECT_DOUBLE_EQ(state("io/diagnostic").value_or(-1.0), 0.0);

  server_.set_value<double>("MAIN.positions", 0, 3.0);
  EXPECT_TRUE(wait_until([&] {return read() == hardware_interface::return_type::OK && state("x/position") == 3.0;}));

  EXPECT_EQ(system_.deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(system_.shutdown().id(), State::PRIMARY_STATE_FINALIZED);
}

TEST_F(HardwareInterfaceWithFakePlc, ARequiredSymbolMissingOnThePlcFailsConfigure)
{
  ASSERT_EQ(initialize("false").id(), State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_NE(system_.configure().id(), State::PRIMARY_STATE_INACTIVE);
}

TEST_F(HardwareInterfaceWithFakePlc, WriteSendsOnlyCommandedValuesAndFallsBackPerInterface)
{
  ASSERT_EQ(initialize("true").id(), State::PRIMARY_STATE_UNCONFIGURED);
  ASSERT_EQ(system_.configure().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(system_.activate().id(), State::PRIMARY_STATE_ACTIVE);

  ASSERT_EQ(write(), hardware_interface::return_type::OK);
  ASSERT_TRUE(wait_until([&] {return server_.writes_to("MAIN.velocity") > 0;}));
  EXPECT_EQ(server_.writes_to("MAIN.targets"), 0u);
  EXPECT_EQ(server_.writes_to("MAIN.mode"), 0u);
  EXPECT_EQ(server_.value<std::uint32_t>("MAIN.heartbeat", 0), 1u);

  command("x/position", 0.25);
  command("y/position", 0.5);
  command("x/velocity", 0.1);
  command("io/mode", 2.0);
  ASSERT_EQ(write(), hardware_interface::return_type::OK);
  ASSERT_TRUE(wait_until([&] {return server_.writes_to("MAIN.targets") > 0;}));
  EXPECT_DOUBLE_EQ(server_.value<double>("MAIN.targets", 0), 0.25);
  EXPECT_DOUBLE_EQ(server_.value<double>("MAIN.targets", 8), 0.5);
  EXPECT_DOUBLE_EQ(server_.value<double>("MAIN.velocity", 0), 0.1);
  EXPECT_EQ(server_.value<std::int16_t>("MAIN.mode", 0), 2);

  command("x/position", 0.75);
  const std::size_t target_writes = server_.writes_to("MAIN.targets");
  ASSERT_EQ(write(), hardware_interface::return_type::OK);
  ASSERT_TRUE(wait_until([&] {return server_.writes_to("MAIN.targets") > target_writes;}));
  EXPECT_DOUBLE_EQ(server_.value<double>("MAIN.targets", 0), 0.75);
  EXPECT_DOUBLE_EQ(server_.value<double>("MAIN.targets", 8), 0.5);
  EXPECT_DOUBLE_EQ(server_.value<double>("MAIN.velocity", 0), 0.0);
  EXPECT_EQ(server_.value<std::int16_t>("MAIN.mode", 0), 2);
  EXPECT_EQ(server_.value<std::uint32_t>("MAIN.heartbeat", 0), 3u);

  EXPECT_EQ(system_.shutdown().id(), State::PRIMARY_STATE_FINALIZED);
}

TEST_F(HardwareInterfaceWithFakePlc, ALinkOutageSurfacesAsAReadError)
{
  ASSERT_EQ(initialize("true").id(), State::PRIMARY_STATE_UNCONFIGURED);
  ASSERT_EQ(system_.configure().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(system_.activate().id(), State::PRIMARY_STATE_ACTIVE);
  ASSERT_EQ(read(), hardware_interface::return_type::OK);

  server_.set_responding(false);
  EXPECT_TRUE(wait_until([&] {return read() == hardware_interface::return_type::ERROR;}));
  server_.set_responding(true);
}
