// Copyright (c) 2025, b-robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include <cmath>

#include <gtest/gtest.h>

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/handle.hpp"

namespace
{
  constexpr const char *TEST_URDF = R"(<?xml version="1.0"?>
<robot name="r">
  <link name="world"/>
  <ros2_control name="sys" type="system">
    <hardware><plugin>fake/Fake</plugin></hardware>
    <gpio name="g">
      <state_interface name="with_initial">
        <param name="initial_value">7.5</param>
        <param name="PLC_symbol">MAIN.x</param>
      </state_interface>
      <state_interface name="without_initial">
        <param name="PLC_symbol">MAIN.y</param>
      </state_interface>
    </gpio>
  </ros2_control>
</robot>)";
} // namespace

// The driver relies on the framework parsing <param name="initial_value"> into
// the exported state handle: an unresolved optional symbol keeps that value,
// and only interfaces without one need the explicit zero at configure.
TEST(InterfaceInitialValue, FrameworkAppliesDeclaredInitialValueToStateHandles)
{
  const std::vector<hardware_interface::HardwareInfo> infos =
      hardware_interface::parse_control_resources_from_urdf(TEST_URDF);
  ASSERT_EQ(infos.size(), 1u);
  ASSERT_EQ(infos[0].gpios.size(), 1u);

  const hardware_interface::ComponentInfo &gpio = infos[0].gpios[0];
  ASSERT_EQ(gpio.state_interfaces.size(), 2u);
  EXPECT_EQ(gpio.state_interfaces[0].initial_value, "7.5");

  hardware_interface::InterfaceDescription description("g", gpio.state_interfaces[0]);
  hardware_interface::StateInterface handle(description);
  double value = 0.0;
  ASSERT_TRUE(handle.get_value(value, true));
  EXPECT_DOUBLE_EQ(value, 7.5);
}

TEST(InterfaceInitialValue, StateHandleWithoutInitialValueStartsAsNan)
{
  const std::vector<hardware_interface::HardwareInfo> infos =
      hardware_interface::parse_control_resources_from_urdf(TEST_URDF);
  const hardware_interface::ComponentInfo &gpio = infos[0].gpios[0];
  EXPECT_TRUE(gpio.state_interfaces[1].initial_value.empty());

  hardware_interface::InterfaceDescription description("g", gpio.state_interfaces[1]);
  hardware_interface::StateInterface handle(description);
  double value = 0.0;
  ASSERT_TRUE(handle.get_value(value, true));
  EXPECT_TRUE(std::isnan(value));
}
