// Interfaces mapped onto fields of PLC structures by byte_offset: each structure travels as one
// SUM item, and a layout that cannot fit its structure fails configure.

#include <chrono>
#include <limits>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "ads_test_support.hpp"

namespace
{
  using namespace beckhoff_ads_hardware_interface::test;
  using namespace std::chrono_literals;

  constexpr const char *COMMAND = "GVL_RosIo.stCommand";
  constexpr const char *FEEDBACK = "GVL_RosIo.stFeedback";
  constexpr size_t COMMAND_SIZE = 66;
  constexpr size_t FEEDBACK_SIZE = 48;

  std::string field(const std::string &kind, const std::string &name, const std::string &symbol,
                    const std::string &type, size_t offset, const std::string &extra = "")
  {
    return "<" + kind + "_interface name=\"" + name + "\">" +
           plcParams(symbol, type, param("byte_offset", std::to_string(offset)) + extra) +
           "</" + kind + "_interface>";
  }

  std::string gpioUrdf(uint16_t port, const std::string &interfaces, const std::string &hardware_extra = "")
  {
    return R"(<?xml version="1.0"?>
<robot name="r">
  <link name="world"/>
  <ros2_control name="sys" type="system">)" +
           hardwareBlock(port, hardware_extra) + "<gpio name=\"g\">" + interfaces + R"(</gpio>
  </ros2_control>
</robot>)";
  }

  // The command structure as the PLC lays it out: version, sequence and time (unmapped here),
  // position and velocity per joint, then two flags. The feedback structure mixes types and
  // leaves byte 7 and bytes 44 to 47 unmapped.
  std::string structUrdf(uint16_t port, const std::string &hardware_extra = "", bool seed_positions = false)
  {
    auto joint = [seed_positions](const std::string &name, size_t command_offset, size_t state_offset)
    {
      const std::string position_extra = seed_positions ? param("seed_from_state", name + "/position") : "";
      const std::string velocity_extra = seed_positions ? param("initial_value", "0") : "";
      return "<joint name=\"" + name + "\">" +
             field("command", "position", COMMAND, "LREAL", command_offset, position_extra) +
             field("command", "velocity", COMMAND, "LREAL", command_offset + 8, velocity_extra) +
             field("state", "position", FEEDBACK, "LREAL", state_offset) +
             field("state", "velocity", FEEDBACK, "LREAL", state_offset + 8) +
             "</joint>";
    };
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
           hardwareBlock(port, hardware_extra) +
           joint("x_axis_joint", 16, 8) + joint("y_axis_joint", 32, 24) +
           "<gpio name=\"machine\">" +
           field("command", "contract_version", COMMAND, "UDINT", 0,
                 param("byte_size", std::to_string(COMMAND_SIZE)) + param("initial_value", "1")) +
           field("command", "servo_enable", COMMAND, "BOOL", 64, param("initial_value", "0")) +
           field("command", "stop", COMMAND, "BOOL", 65, param("initial_value", "0")) +
           field("state", "contract_version", FEEDBACK, "UDINT", 0, param("byte_size", std::to_string(FEEDBACK_SIZE))) +
           field("state", "group_state", FEEDBACK, "INT", 4) +
           field("state", "owns_axes", FEEDBACK, "BOOL", 6) +
           field("state", "x_alarm_id", FEEDBACK, "UDINT", 40) +
           R"(</gpio>
  </ros2_control>
</robot>)";
  }

  class StructSymbolLayout : public AdsInterfaceTest
  {
  protected:
    void addStructs()
    {
      std::vector<uint8_t> feedback(FEEDBACK_SIZE, 0xEE); // unmapped bytes carry junk the interface must ignore
      putAt<uint32_t>(feedback, 0, 1);
      putAt<int16_t>(feedback, 4, 20);
      putAt<uint8_t>(feedback, 6, 1);
      putAt<double>(feedback, 8, 0.125);
      putAt<double>(feedback, 16, -0.0625);
      putAt<double>(feedback, 24, 0.5);
      putAt<double>(feedback, 32, 0.0);
      putAt<uint32_t>(feedback, 40, 1210);
      server_.addSymbol(FEEDBACK, feedback);
      server_.addSymbol(COMMAND, std::vector<uint8_t>(COMMAND_SIZE, 0));
    }
  };

  TEST_F(StructSymbolLayout, ReadsEachFieldAtItsOffsetAndType)
  {
    addStructs();
    start(structUrdf(server_.port()));
    ASSERT_EQ(componentState("gantry"), "active");
    cycle();

    EXPECT_DOUBLE_EQ(state("machine/contract_version"), 1.0);
    EXPECT_DOUBLE_EQ(state("machine/group_state"), 20.0);
    EXPECT_DOUBLE_EQ(state("machine/owns_axes"), 1.0);
    EXPECT_DOUBLE_EQ(state("x_axis_joint/position"), 0.125);
    EXPECT_DOUBLE_EQ(state("x_axis_joint/velocity"), -0.0625);
    EXPECT_DOUBLE_EQ(state("y_axis_joint/position"), 0.5);
    EXPECT_DOUBLE_EQ(state("y_axis_joint/velocity"), 0.0);
    EXPECT_DOUBLE_EQ(state("machine/x_alarm_id"), 1210.0);

    EXPECT_EQ(server_.symbolInfoRequests(), 2u); // each structure's size checked once, in its direction

    // One SUM item covers the whole structure.
    EXPECT_EQ(describe(server_.lastSumRead()),
              std::string(FEEDBACK) + " 0xf005 " + std::to_string(FEEDBACK_SIZE) + "\n");
  }

  TEST_F(StructSymbolLayout, WritesTheWholeStructureOnceEveryFieldHasAValue)
  {
    addStructs();
    start(structUrdf(server_.port()));
    ASSERT_EQ(componentState("gantry"), "active");

    // The joint positions have no value yet, so the structure is held back as a whole.
    cycle();
    std::this_thread::sleep_for(50ms);
    for (const auto &write : server_.sumWrites())
    {
      EXPECT_FALSE(findItem(write, COMMAND).has_value());
    }
    const size_t writes_before = server_.sumWrites().size();

    {
      auto x_pos = rm_->claim_command_interface("x_axis_joint/position");
      auto x_vel = rm_->claim_command_interface("x_axis_joint/velocity");
      auto y_pos = rm_->claim_command_interface("y_axis_joint/position");
      auto y_vel = rm_->claim_command_interface("y_axis_joint/velocity");
      auto servo = rm_->claim_command_interface("machine/servo_enable");
      ASSERT_TRUE(x_pos.set_value(0.25));
      ASSERT_TRUE(x_vel.set_value(0.05));
      ASSERT_TRUE(y_pos.set_value(-0.5));
      ASSERT_TRUE(y_vel.set_value(0.0));
      ASSERT_TRUE(servo.set_value(1.0));
      cycle();
      ASSERT_TRUE(server_.waitForSumWrites(writes_before + 1, 2s));
    }

    std::vector<uint8_t> expected(COMMAND_SIZE, 0);
    putAt<uint32_t>(expected, 0, 1);
    putAt<double>(expected, 16, 0.25);
    putAt<double>(expected, 24, 0.05);
    putAt<double>(expected, 32, -0.5);
    putAt<double>(expected, 40, 0.0);
    putAt<uint8_t>(expected, 64, 1);
    putAt<uint8_t>(expected, 65, 0);

    const auto write = server_.sumWrites().back();
    ASSERT_EQ(write.size(), 1u);
    const std::optional<SumItem> command = findItem(write, COMMAND);
    ASSERT_TRUE(command.has_value());
    EXPECT_EQ(command->index_group, 0xF005u);
    EXPECT_EQ(hex(command->data), hex(expected));
    EXPECT_EQ(hex(server_.symbolValue(COMMAND)), hex(expected));
  }

  // ---- driver-generated values ---------------------------------------------------------------

  void commandAllJoints(hardware_interface::ResourceManager &rm)
  {
    for (const char *name : {"x_axis_joint/position", "x_axis_joint/velocity",
                             "y_axis_joint/position", "y_axis_joint/velocity"})
    {
      auto handle = rm.claim_command_interface(name);
      ASSERT_TRUE(handle.set_value(0.25));
    }
  }

  TEST_F(StructSymbolLayout, WritesTheSetpointSequenceAndTimeIntoTheirFields)
  {
    addStructs();
    start(structUrdf(server_.port(),
                     param("setpoint_sequence_plc_symbol", COMMAND) + param("setpoint_sequence_byte_offset", "4") +
                         param("setpoint_time_plc_symbol", COMMAND) + param("setpoint_time_byte_offset", "8")));
    ASSERT_EQ(componentState("gantry"), "active");

    std::vector<std::vector<uint8_t>> sent;
    for (int i = 0; i < 3; ++i)
    {
      commandAllJoints(*rm_);
      const size_t before = server_.sumWrites().size();
      cycle();
      ASSERT_TRUE(server_.waitForSumWrites(before + 1, 2s));
      const std::optional<SumItem> command = findItem(server_.sumWrites().back(), COMMAND);
      ASSERT_TRUE(command.has_value());
      sent.push_back(command->data);
    }

    for (size_t i = 1; i < sent.size(); ++i)
    {
      EXPECT_EQ(getAt<uint32_t>(sent[i], 4), getAt<uint32_t>(sent[i - 1], 4) + 1);
      EXPECT_GE(getAt<double>(sent[i], 8), getAt<double>(sent[i - 1], 8));
    }
    EXPECT_GT(getAt<double>(sent[0], 8), 0.0);

    // Every other field is untouched by the sequence and the time.
    std::vector<uint8_t> expected(COMMAND_SIZE, 0);
    putAt<uint32_t>(expected, 0, 1);
    for (size_t offset : {16u, 24u, 32u, 40u})
    {
      putAt<double>(expected, offset, 0.25);
    }
    std::vector<uint8_t> last = sent.back();
    std::fill(last.begin() + 4, last.begin() + 16, 0);
    EXPECT_EQ(hex(last), hex(expected));
  }

  // ---- seed_from_state -------------------------------------------------------------------------

  std::vector<uint8_t> waitForCommandWrite(FakeAdsServer &server, size_t writes_before)
  {
    EXPECT_TRUE(server.waitForSumWrites(writes_before + 1, 2s));
    const std::optional<SumItem> command = findItem(server.sumWrites().back(), COMMAND);
    return command ? command->data : std::vector<uint8_t>{};
  }

  TEST_F(StructSymbolLayout, SeedsPositionsFromTheAxesOnActivation)
  {
    addStructs();
    start(structUrdf(server_.port(), "", true));
    ASSERT_EQ(componentState("gantry"), "active");

    // No controller has commanded anything, yet the structure goes out carrying the axes' positions.
    cycle();
    const std::vector<uint8_t> sent = waitForCommandWrite(server_, 0);
    ASSERT_EQ(sent.size(), COMMAND_SIZE);
    EXPECT_DOUBLE_EQ(getAt<double>(sent, 16), 0.125);
    EXPECT_DOUBLE_EQ(getAt<double>(sent, 24), 0.0);
    EXPECT_DOUBLE_EQ(getAt<double>(sent, 32), 0.5);
    EXPECT_DOUBLE_EQ(getAt<double>(sent, 40), 0.0);
    EXPECT_EQ(getAt<uint32_t>(sent, 0), 1u);

    // A command replaces the seed as usual.
    {
      auto x_pos = rm_->claim_command_interface("x_axis_joint/position");
      ASSERT_TRUE(x_pos.set_value(0.2));
      const size_t before = server_.sumWrites().size();
      cycle();
      EXPECT_DOUBLE_EQ(getAt<double>(waitForCommandWrite(server_, before), 16), 0.2);
    }
  }

  TEST_F(StructSymbolLayout, ReseedsOnEveryActivation)
  {
    addStructs();
    start(structUrdf(server_.port(), "", true));
    ASSERT_EQ(componentState("gantry"), "active");

    // The axis moves while the hardware is inactive.
    rclcpp_lifecycle::State inactive(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
    rclcpp_lifecycle::State active(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
    ASSERT_EQ(rm_->set_component_state("gantry", inactive), hardware_interface::return_type::OK);
    std::vector<uint8_t> feedback = server_.symbolValue(FEEDBACK);
    putAt<double>(feedback, 8, 0.3);
    server_.addSymbol(FEEDBACK, feedback);
    ASSERT_EQ(rm_->set_component_state("gantry", active), hardware_interface::return_type::OK);

    const size_t before = server_.sumWrites().size();
    cycle();
    EXPECT_DOUBLE_EQ(getAt<double>(waitForCommandWrite(server_, before), 16), 0.3);
  }

  TEST_F(StructSymbolLayout, SeedsAPlainSymbolToo)
  {
    server_.addSymbol("MAIN.target", bytesOf(0.0));
    server_.addSymbol("MAIN.actual", bytesOf(3.5));
    start(gpioUrdf(server_.port(),
                   "<command_interface name=\"target\">" +
                       plcParams("MAIN.target", "LREAL", param("seed_from_state", "g/actual")) +
                       "</command_interface><state_interface name=\"actual\">" + plcParams("MAIN.actual", "LREAL") +
                       "</state_interface>"));
    ASSERT_EQ(componentState("sys"), "active");
    cycle();
    ASSERT_TRUE(server_.waitForSumWrites(1, 2s));
    EXPECT_DOUBLE_EQ(getAt<double>(server_.symbolValue("MAIN.target"), 0), 3.5);
  }

  TEST_F(StructSymbolLayout, FailsActivationWhenTheSeedIsNotANumber)
  {
    server_.addSymbol("MAIN.target", bytesOf(0.0));
    server_.addSymbol("MAIN.actual", bytesOf(std::numeric_limits<double>::quiet_NaN()));
    start(gpioUrdf(server_.port(),
                   "<command_interface name=\"target\">" +
                       plcParams("MAIN.target", "LREAL", param("seed_from_state", "g/actual")) +
                       "</command_interface><state_interface name=\"actual\">" + plcParams("MAIN.actual", "LREAL") +
                       "</state_interface>"));
    EXPECT_NE(componentState("sys"), "active");
  }

  // ---- byte_size against the structure's size on the PLC -----------------------------------

  TEST_F(StructSymbolLayout, FailsConfigureWhenTheCommandStructureGrewOnThePlc)
  {
    addStructs();
    server_.addSymbol(COMMAND, std::vector<uint8_t>(COMMAND_SIZE + 4, 0));
    start(structUrdf(server_.port()));
    EXPECT_NE(componentState("gantry"), "active");
  }

  TEST_F(StructSymbolLayout, FailsConfigureWhenTheCommandStructureShrankOnThePlc)
  {
    addStructs();
    server_.addSymbol(COMMAND, std::vector<uint8_t>(COMMAND_SIZE - 2, 0));
    start(structUrdf(server_.port()));
    EXPECT_NE(componentState("gantry"), "active");
  }

  TEST_F(StructSymbolLayout, FailsConfigureWhenTheFeedbackStructureDiffersOnThePlc)
  {
    addStructs();
    server_.addSymbol(FEEDBACK, std::vector<uint8_t>(FEEDBACK_SIZE + 8, 0));
    start(structUrdf(server_.port()));
    EXPECT_NE(componentState("gantry"), "active");
  }

  TEST_F(StructSymbolLayout, FailsConfigureWhenThePlcWillNotReportTheSize)
  {
    addStructs();
    server_.refuseSymbolInfo();
    start(structUrdf(server_.port()));
    EXPECT_NE(componentState("gantry"), "active");
  }

  // ---- layouts that cannot fit their structure ---------------------------------------------

  struct BadLayout
  {
    const char *name;
    std::string interfaces;
    std::string hardware = "";
  };

  class StructLayoutRejected : public AdsInterfaceTest, public ::testing::WithParamInterface<BadLayout>
  {
  };

  TEST_P(StructLayoutRejected, FailsConfigure)
  {
    server_.addSymbol("GVL.st", std::vector<uint8_t>(16, 0));
    start(gpioUrdf(server_.port(), GetParam().interfaces, GetParam().hardware));
    EXPECT_NE(componentState("sys"), "active");
  }

  const std::string SIZE_16 = param("byte_size", "16");

  class OptionalStructSymbol : public AdsInterfaceTest
  {
  protected:
    void startOptional()
    {
      start(gpioUrdf(server_.port(), field("state", "a", "GVL.st", "UDINT", 0, SIZE_16 + param("optional", "true"))));
    }
  };

  TEST_F(OptionalStructSymbol, MissingFromThePlcIsDropped)
  {
    startOptional();
    EXPECT_EQ(componentState("sys"), "active");
  }

  TEST_F(OptionalStructSymbol, PresentWithTheRightSizeIsUsed)
  {
    server_.addSymbol("GVL.st", bytesOfArray<uint32_t>({7, 0, 0, 0}));
    startOptional();
    ASSERT_EQ(componentState("sys"), "active");
    cycle();
    EXPECT_DOUBLE_EQ(state("g/a"), 7.0);
  }

  TEST_F(OptionalStructSymbol, PresentWithTheWrongSizeFailsConfigure)
  {
    server_.addSymbol("GVL.st", std::vector<uint8_t>(20, 0));
    startOptional();
    EXPECT_NE(componentState("sys"), "active");
  }

  INSTANTIATE_TEST_SUITE_P(
      StructSymbolLayout, StructLayoutRejected,
      ::testing::Values(
          BadLayout{"Overlap",
                    field("state", "a", "GVL.st", "UDINT", 0, SIZE_16) + field("state", "b", "GVL.st", "INT", 2)},
          BadLayout{"PastTheEnd",
                    field("state", "a", "GVL.st", "LREAL", 12, SIZE_16)},
          BadLayout{"NoByteSize",
                    field("state", "a", "GVL.st", "UDINT", 0)},
          BadLayout{"ByteSizesDisagree",
                    field("state", "a", "GVL.st", "UDINT", 0, SIZE_16) +
                        field("state", "b", "GVL.st", "UDINT", 4, param("byte_size", "12"))},
          BadLayout{"SameOffsetTwice",
                    field("state", "a", "GVL.st", "UDINT", 0, SIZE_16) + field("state", "b", "GVL.st", "UDINT", 0)},
          BadLayout{"ByteOffsetWithIndex",
                    field("state", "a", "GVL.st", "UDINT", 0, SIZE_16 + param("index", "1"))},
          BadLayout{"MixedWithIndexedInterface",
                    field("state", "a", "GVL.st", "UDINT", 0, SIZE_16) +
                        "<state_interface name=\"b\">" +
                        plcParams("GVL.st", "UDINT", param("n_elements", "4") + param("index", "1")) +
                        "</state_interface>"},
          BadLayout{"IndexedThenMixed",
                    "<state_interface name=\"b\">" +
                        plcParams("GVL.st", "UDINT", param("n_elements", "4") + param("index", "1")) +
                        "</state_interface>" + field("state", "a", "GVL.st", "UDINT", 0, SIZE_16)},
          BadLayout{"SequenceOffsetOnAPlainSymbol",
                    "<command_interface name=\"a\">" + plcParams("GVL.st", "UDINT") + "</command_interface>",
                    param("setpoint_sequence_plc_symbol", "GVL.st") + param("setpoint_sequence_byte_offset", "4")},
          BadLayout{"SequenceOffsetOverlapsAField",
                    field("command", "a", "GVL.st", "UDINT", 4, SIZE_16),
                    param("setpoint_sequence_plc_symbol", "GVL.st") + param("setpoint_sequence_byte_offset", "6")},
          BadLayout{"SequenceOffsetOnAMappedByte",
                    field("command", "a", "GVL.st", "UDINT", 4, SIZE_16),
                    param("setpoint_sequence_plc_symbol", "GVL.st") + param("setpoint_sequence_byte_offset", "4")},
          BadLayout{"TimeOffsetPastTheEnd",
                    field("command", "a", "GVL.st", "UDINT", 0, SIZE_16),
                    param("setpoint_time_plc_symbol", "GVL.st") + param("setpoint_time_byte_offset", "12")},
          BadLayout{"InvalidSequenceOffset",
                    field("command", "a", "GVL.st", "UDINT", 0, SIZE_16),
                    param("setpoint_sequence_plc_symbol", "GVL.st") + param("setpoint_sequence_byte_offset", "four")},
          BadLayout{"SequenceOffsetWithoutItsSymbol",
                    field("command", "a", "GVL.st", "UDINT", 0, SIZE_16),
                    param("setpoint_sequence_byte_offset", "4")},
          BadLayout{"SeedFromAMissingState",
                    "<command_interface name=\"a\">" + plcParams("GVL.st", "UDINT", param("seed_from_state", "g/nothing")) +
                        "</command_interface>"},
          BadLayout{"SeedAndInitialValue",
                    "<command_interface name=\"a\">" +
                        plcParams("GVL.st", "UDINT", param("seed_from_state", "g/b") + param("initial_value", "1")) +
                        "</command_interface><state_interface name=\"b\">" + plcParams("GVL.st", "UDINT") +
                        "</state_interface>"},
          BadLayout{"NegativeOffset",
                    "<state_interface name=\"a\">" +
                        plcParams("GVL.st", "UDINT", param("byte_offset", "-4") + SIZE_16) + "</state_interface>"}),
      [](const ::testing::TestParamInfo<BadLayout> &param_info)
      { return std::string(param_info.param.name); });

} // namespace
