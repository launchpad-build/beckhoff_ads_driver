// Locks what the interface puts on the wire for configurations made only of plain and
// array symbols. The expected items were recorded from release 1.5.0; a change to them
// breaks every URDF already in the field.

#include <chrono>
#include <optional>
#include <set>
#include <string>

#include <gtest/gtest.h>

#include "ads_test_support.hpp"

namespace
{
  using namespace beckhoff_ads_hardware_interface::test;
  using namespace std::chrono_literals;

  // The V2 gantry before the struct interface: one symbol per value, a heartbeat, a setpoint
  // sequence and timestamp, optional symbols missing from the PLC.
  std::string gantryUrdf(uint16_t port)
  {
    const std::string heartbeat = R"(
      <param name="heartbeat_plc_symbol">MAIN.ros_heartbeat</param>
      <param name="setpoint_sequence_plc_symbol">MAIN.ros_setpoint_sequence</param>
      <param name="setpoint_time_plc_symbol">MAIN.ros_setpoint_time</param>)";
    auto joint = [](const std::string &name, const std::string &axis)
    {
      return "<joint name=\"" + name + "\">" +
             "<command_interface name=\"position\">" + plcParams("MAIN.gantry_" + axis + "_target_position", "LREAL") + "</command_interface>" +
             "<command_interface name=\"velocity\">" + plcParams("MAIN.gantry_" + axis + "_target_velocity", "LREAL") + "</command_interface>" +
             "<state_interface name=\"position\">" + plcParams("MAIN.gantry_" + axis + "_current_position", "LREAL") + "</state_interface>" +
             "<state_interface name=\"velocity\">" + plcParams("MAIN.gantry_" + axis + "_current_velocity", "LREAL") + "</state_interface>" +
             "</joint>";
    };
    const std::string optional = "<param name=\"optional\">true</param>";
    return R"(<?xml version="1.0"?>
<robot name="gantry">
  <link name="world"/>
  <link name="x_link"/>
  <link name="y_link"/>
  <joint name="x_axis_joint" type="prismatic">
    <parent link="world"/><child link="x_link"/><axis xyz="1 0 0"/>
    <limit lower="-0.001" upper="0.33" velocity="0.1" effort="100"/>
  </joint>
  <joint name="y_axis_joint" type="prismatic">
    <parent link="x_link"/><child link="y_link"/><axis xyz="0 1 0"/>
    <limit lower="-0.001" upper="0.53" velocity="0.1" effort="100"/>
  </joint>
  <ros2_control name="gantry" type="system">)" +
           hardwareBlock(port, heartbeat) +
           joint("x_axis_joint", "x") + joint("y_axis_joint", "y") +
           R"(<gpio name="machine">)" +
           "<command_interface name=\"enable\">" + plcParams("MAIN.gantry_enable", "BOOL", "<param name=\"initial_value\">0</param>") + "</command_interface>" +
           "<command_interface name=\"servo_mode\">" + plcParams("MAIN.command_servo_enable_gantry", "BOOL", "<param name=\"initial_value\">0</param>") + "</command_interface>" +
           "<command_interface name=\"gripper_target\">" + plcParams("MAIN.gripper_target_position", "LREAL", optional) + "</command_interface>" +
           "<state_interface name=\"drives_ready\">" + plcParams("MAIN.gantry_drives_ready", "BOOL") + "</state_interface>" +
           "<state_interface name=\"x_alarm_id\">" + plcParams("MAIN.gantry_x_alarm_id", "UDINT") + "</state_interface>" +
           "<state_interface name=\"contract_version\">" + plcParams("MAIN.ros_contract_version", "UDINT", optional) + "</state_interface>" +
           R"(</gpio>
  </ros2_control>
</robot>)";
  }

  // beckhoff_ads_bringup's example robot: arrays addressed by index, mixed types, and a symbol
  // written as USINT but read as UDINT.
  std::string exampleRobotUrdf(uint16_t port)
  {
    auto command = [](const std::string &name, const std::string &symbol, const std::string &type,
                      const std::string &initial, const std::string &extra = "")
    {
      return "<command_interface name=\"" + name + "\">" +
             plcParams(symbol, type, "<param name=\"initial_value\">" + initial + "</param>" + extra) +
             "</command_interface>";
    };
    auto state = [](const std::string &name, const std::string &symbol, const std::string &type,
                    const std::string &extra = "")
    {
      return "<state_interface name=\"" + name + "\">" + plcParams(symbol, type, extra) + "</state_interface>";
    };
    const std::string array0 = "<param name=\"n_elements\">2</param><param name=\"index\">0</param>";
    const std::string array1 = "<param name=\"n_elements\">2</param><param name=\"index\">1</param>";
    return R"(<?xml version="1.0"?>
<robot name="beckhoff_bot">
  <link name="world"/>
  <ros2_control name="beckhoff_bot" type="system">)" +
           hardwareBlock(port) +
           R"(<gpio name="robot_io">)" +
           command("joggingEnabled", "MAIN.joggingEnabled", "BOOL", "1") +
           command("myRobotMode", "MAIN.myRobotMode", "UDINT", "5") +
           command("motionType", "MAIN.motionType", "USINT", "1") +
           command("commandPos_0", "MAIN.commandPos", "BOOL", "1", array0) +
           command("commandPos_1", "MAIN.commandPos", "BOOL", "1", array1) +
           R"(</gpio><sensor name="robot_sensor">)" +
           state("statusError", "MAIN.statusError", "BOOL") +
           state("batteryVoltage", "MAIN.batteryVoltage", "REAL") +
           state("currentPos_0", "MAIN.currentPos", "LREAL", array0) +
           state("currentPos_1", "MAIN.currentPos", "LREAL", array1) +
           state("pickCount", "MAIN.pickCount", "UINT") +
           state("currentTemperature", "MAIN.currentTemperature", "INT") +
           state("ledMap", "MAIN.ledMap", "BYTE") +
           state("joggingEnabled", "MAIN.joggingEnabled", "BOOL") +
           state("myRobotMode", "MAIN.myRobotMode", "UDINT") +
           state("motionType", "MAIN.motionType", "UDINT") +
           state("commandPos_0", "MAIN.commandPos", "BOOL", array0) +
           state("commandPos_1", "MAIN.commandPos", "BOOL", array1) +
           R"(</sensor>
  </ros2_control>
</robot>)";
  }

  class LegacySymbolLayout : public AdsInterfaceTest
  {
  };

  const std::set<std::string> GANTRY_VOLATILE = {
      "MAIN.ros_heartbeat", "MAIN.ros_setpoint_sequence", "MAIN.ros_setpoint_time"};

  const char *GANTRY_SUM_READ =
      "MAIN.gantry_drives_ready 0xf005 1\n"
      "MAIN.gantry_x_alarm_id 0xf005 4\n"
      "MAIN.gantry_x_current_position 0xf005 8\n"
      "MAIN.gantry_x_current_velocity 0xf005 8\n"
      "MAIN.gantry_y_current_position 0xf005 8\n"
      "MAIN.gantry_y_current_velocity 0xf005 8\n";
  const char *GANTRY_FIRST_WRITE =
      "MAIN.command_servo_enable_gantry 0xf005 1 00\n"
      "MAIN.gantry_enable 0xf005 1 00\n"
      "MAIN.gantry_x_target_velocity 0xf005 8 0000000000000000\n"
      "MAIN.gantry_y_target_velocity 0xf005 8 0000000000000000\n"
      "MAIN.ros_heartbeat 0xf005 4 *\n"
      "MAIN.ros_setpoint_sequence 0xf005 4 *\n"
      "MAIN.ros_setpoint_time 0xf005 8 *\n";
  const char *GANTRY_COMMANDED_WRITE =
      "MAIN.command_servo_enable_gantry 0xf005 1 00\n"
      "MAIN.gantry_enable 0xf005 1 01\n"
      "MAIN.gantry_x_target_position 0xf005 8 000000000000d03f\n"
      "MAIN.gantry_x_target_velocity 0xf005 8 9a9999999999a93f\n"
      "MAIN.gantry_y_target_position 0xf005 8 000000000000e0bf\n"
      "MAIN.gantry_y_target_velocity 0xf005 8 0000000000000000\n"
      "MAIN.ros_heartbeat 0xf005 4 *\n"
      "MAIN.ros_setpoint_sequence 0xf005 4 *\n"
      "MAIN.ros_setpoint_time 0xf005 8 *\n";

  TEST_F(LegacySymbolLayout, GantrySymbolsOnTheWire)
  {
    server_.addSymbol("MAIN.gantry_x_target_position", bytesOf(0.0));
    server_.addSymbol("MAIN.gantry_x_target_velocity", bytesOf(0.0));
    server_.addSymbol("MAIN.gantry_x_current_position", bytesOf(0.125));
    server_.addSymbol("MAIN.gantry_x_current_velocity", bytesOf(-0.0625));
    server_.addSymbol("MAIN.gantry_y_target_position", bytesOf(0.0));
    server_.addSymbol("MAIN.gantry_y_target_velocity", bytesOf(0.0));
    server_.addSymbol("MAIN.gantry_y_current_position", bytesOf(0.5));
    server_.addSymbol("MAIN.gantry_y_current_velocity", bytesOf(0.0));
    server_.addSymbol("MAIN.gantry_enable", bytesOf<uint8_t>(0));
    server_.addSymbol("MAIN.command_servo_enable_gantry", bytesOf<uint8_t>(0));
    server_.addSymbol("MAIN.gantry_drives_ready", bytesOf<uint8_t>(1));
    server_.addSymbol("MAIN.gantry_x_alarm_id", bytesOf<uint32_t>(1210));
    server_.addSymbol("MAIN.ros_heartbeat", bytesOf<uint32_t>(0));
    server_.addSymbol("MAIN.ros_setpoint_sequence", bytesOf<uint32_t>(0));
    server_.addSymbol("MAIN.ros_setpoint_time", bytesOf(0.0));

    start(gantryUrdf(server_.port()));
    ASSERT_EQ(componentState("gantry"), "active");
    cycle();
    ASSERT_TRUE(server_.waitForSumWrites(1, 2s));

    EXPECT_DOUBLE_EQ(state("x_axis_joint/position"), 0.125);
    EXPECT_DOUBLE_EQ(state("x_axis_joint/velocity"), -0.0625);
    EXPECT_DOUBLE_EQ(state("y_axis_joint/position"), 0.5);
    EXPECT_DOUBLE_EQ(state("machine/drives_ready"), 1.0);
    EXPECT_DOUBLE_EQ(state("machine/x_alarm_id"), 1210.0);
    EXPECT_DOUBLE_EQ(state("machine/contract_version"), 0.0);
    EXPECT_EQ(describe(server_.lastSumRead()), GANTRY_SUM_READ);
    EXPECT_EQ(server_.symbolInfoRequests(), 0u); // sizes are checked for structures only

    {
      auto x_pos = rm_->claim_command_interface("x_axis_joint/position");
      auto x_vel = rm_->claim_command_interface("x_axis_joint/velocity");
      auto y_pos = rm_->claim_command_interface("y_axis_joint/position");
      auto y_vel = rm_->claim_command_interface("y_axis_joint/velocity");
      auto enable = rm_->claim_command_interface("machine/enable");
      ASSERT_TRUE(x_pos.set_value(0.25));
      ASSERT_TRUE(x_vel.set_value(0.05));
      ASSERT_TRUE(y_pos.set_value(-0.5));
      ASSERT_TRUE(y_vel.set_value(0.0));
      ASSERT_TRUE(enable.set_value(1.0));
      cycle();
      ASSERT_TRUE(server_.waitForSumWrites(2, 2s));
    }

    const auto writes = server_.sumWrites();
    EXPECT_EQ(describe(writes[0], GANTRY_VOLATILE), GANTRY_FIRST_WRITE);
    EXPECT_EQ(describe(writes[1], GANTRY_VOLATILE), GANTRY_COMMANDED_WRITE);

    // The heartbeat and the setpoint sequence advance by one per write cycle; the time is monotonic.
    for (const char *symbol : {"MAIN.ros_heartbeat", "MAIN.ros_setpoint_sequence"})
    {
      const std::optional<SumItem> first = findItem(writes[0], symbol);
      const std::optional<SumItem> second = findItem(writes[1], symbol);
      ASSERT_TRUE(first.has_value()) << symbol;
      ASSERT_TRUE(second.has_value()) << symbol;
      uint32_t a = 0, b = 0;
      std::memcpy(&a, first->data.data(), sizeof(a));
      std::memcpy(&b, second->data.data(), sizeof(b));
      EXPECT_EQ(b, a + 1) << symbol;
    }
    const std::optional<SumItem> t0 = findItem(writes[0], "MAIN.ros_setpoint_time");
    const std::optional<SumItem> t1 = findItem(writes[1], "MAIN.ros_setpoint_time");
    ASSERT_TRUE(t0.has_value());
    ASSERT_TRUE(t1.has_value());
    double s0 = 0.0, s1 = 0.0;
    std::memcpy(&s0, t0->data.data(), sizeof(s0));
    std::memcpy(&s1, t1->data.data(), sizeof(s1));
    EXPECT_GT(s0, 0.0);
    EXPECT_GE(s1, s0);
  }

  const char *EXAMPLE_SUM_READ =
      "MAIN.batteryVoltage 0xf005 4\n"
      "MAIN.commandPos 0xf005 2\n"
      "MAIN.currentPos 0xf005 16\n"
      "MAIN.currentTemperature 0xf005 2\n"
      "MAIN.joggingEnabled 0xf005 1\n"
      "MAIN.ledMap 0xf005 1\n"
      "MAIN.motionType 0xf005 4\n"
      "MAIN.myRobotMode 0xf005 4\n"
      "MAIN.pickCount 0xf005 2\n"
      "MAIN.statusError 0xf005 1\n";
  const char *EXAMPLE_FIRST_WRITE =
      "MAIN.commandPos 0xf005 2 0101\n"
      "MAIN.joggingEnabled 0xf005 1 01\n"
      "MAIN.motionType 0xf005 1 01\n"
      "MAIN.myRobotMode 0xf005 4 05000000\n";
  const char *EXAMPLE_COMMANDED_WRITE =
      "MAIN.commandPos 0xf005 2 0100\n"
      "MAIN.joggingEnabled 0xf005 1 01\n"
      "MAIN.motionType 0xf005 1 01\n"
      "MAIN.myRobotMode 0xf005 4 07000000\n";

  TEST_F(LegacySymbolLayout, ExampleRobotSymbolsOnTheWire)
  {
    server_.addSymbol("MAIN.joggingEnabled", bytesOf<uint8_t>(0));
    server_.addSymbol("MAIN.myRobotMode", bytesOf<uint32_t>(0));
    server_.addSymbol("MAIN.motionType", bytesOf<uint32_t>(0));
    server_.addSymbol("MAIN.commandPos", bytesOfArray<uint8_t>({0, 0}));
    server_.addSymbol("MAIN.statusError", bytesOf<uint8_t>(1));
    server_.addSymbol("MAIN.batteryVoltage", bytesOf(24.5f));
    server_.addSymbol("MAIN.currentPos", bytesOfArray<double>({1.5, -2.25}));
    server_.addSymbol("MAIN.pickCount", bytesOf<uint16_t>(42));
    server_.addSymbol("MAIN.currentTemperature", bytesOf<int16_t>(-7));
    server_.addSymbol("MAIN.ledMap", bytesOf<uint8_t>(0xA5));

    start(exampleRobotUrdf(server_.port()));
    ASSERT_EQ(componentState("beckhoff_bot"), "active");
    cycle();
    ASSERT_TRUE(server_.waitForSumWrites(1, 2s));

    EXPECT_DOUBLE_EQ(state("robot_sensor/statusError"), 1.0);
    EXPECT_DOUBLE_EQ(state("robot_sensor/batteryVoltage"), 24.5);
    EXPECT_DOUBLE_EQ(state("robot_sensor/currentPos_0"), 1.5);
    EXPECT_DOUBLE_EQ(state("robot_sensor/currentPos_1"), -2.25);
    EXPECT_DOUBLE_EQ(state("robot_sensor/pickCount"), 42.0);
    EXPECT_DOUBLE_EQ(state("robot_sensor/currentTemperature"), -7.0);
    EXPECT_DOUBLE_EQ(state("robot_sensor/ledMap"), 165.0);
    EXPECT_EQ(describe(server_.lastSumRead()), EXAMPLE_SUM_READ);
    EXPECT_EQ(server_.symbolInfoRequests(), 0u);

    {
      auto mode = rm_->claim_command_interface("robot_io/myRobotMode");
      auto pos1 = rm_->claim_command_interface("robot_io/commandPos_1");
      ASSERT_TRUE(mode.set_value(7.0));
      ASSERT_TRUE(pos1.set_value(0.0));
      cycle();
      ASSERT_TRUE(server_.waitForSumWrites(2, 2s));
    }

    const auto writes = server_.sumWrites();
    EXPECT_EQ(describe(writes[0]), EXAMPLE_FIRST_WRITE);
    EXPECT_EQ(describe(writes[1]), EXAMPLE_COMMANDED_WRITE);
  }

} // namespace
