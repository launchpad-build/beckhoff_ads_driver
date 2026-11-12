#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <thread>
#include <vector>

#include "beckhoff_ads_hardware_interface/beckhoff_ads_hardware_interface.hpp"
#include "gpio_controllers/gpio_command_controller.hpp"
#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

using namespace std::chrono_literals;

// Minimal loopback ADS peer. The driver still parses URDF, configures handles,
// starts its real I/O threads and exchanges complete AMS/TCP frames.
class LoopbackPlc
{
public:
  LoopbackPlc()
  {
    listener_ = socket(AF_INET, SOCK_STREAM, 0);
    const int reuse = 1;
    setsockopt(listener_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = htons(48898);
    if (bind(listener_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) ||
        listen(listener_, 1)) {
      close(listener_);
      throw std::runtime_error("Cannot bind loopback ADS peer");
    }
    thread_ = std::thread([this] { serve(); });
  }

  ~LoopbackPlc()
  {
    stopping_.store(true);
    shutdown(listener_, SHUT_RDWR);
    shutdown(client_.load(), SHUT_RDWR);
    thread_.join();
    close(listener_);
  }

  std::array<std::atomic<uint8_t>, 3> value{};
  std::atomic<uint8_t> scalar{77};
  // Only this peer models PLC event admission. Raw command storage stays ROS-owned.
  std::array<std::atomic<uint32_t>, 4> boot_id{{1, 17, 23, 29}};
  std::array<std::atomic<uint32_t>, 4> boot_echo{};
  std::atomic<bool> scan_enabled{true};
  std::atomic<bool> input_ready{false};
  std::atomic<uint8_t> observed_event{0};
  std::atomic<uint32_t> accepted_events{0};
  std::atomic<unsigned> writes{0};
  std::atomic<bool> hold_writes{false};
  std::atomic<bool> hold_reads{false};
  std::atomic<bool> fail_reads{false};
  std::atomic<bool> drop_read_reply{false};
  std::atomic<unsigned> dropped_read_replies{0};
  std::atomic<bool> drop_write_reply{false};
  std::atomic<unsigned> read_requests{0};
  std::atomic<unsigned> positive_array_writes{0};
  std::array<std::atomic<double>, 3> calibration_xyz{{-0.125, 0.25, 1.5}};
  std::atomic<uint8_t> calibration_valid{1};
  std::atomic<uint32_t> heartbeat{0}, sequence{0};
  std::atomic<double> timestamp{0};
  std::atomic<unsigned> metadata_writes{0};
  std::atomic<uint8_t> metadata_scalar{0};

private:
  void scan()
  {
    if (scanned_boot_ != boot_id[0].load()) {
      scanned_boot_ = boot_id[0].load();
      input_ready.store(false);
      previous_event_ = false;
    }
    if (!scan_enabled.load()) return;
    bool current_boot = true;
    for (size_t i = 0; i < boot_id.size(); ++i) {
      current_boot = current_boot && boot_echo[i].load() == boot_id[i].load();
    }
    const bool event = scalar.load() != 0;
    if (!current_boot) input_ready.store(false);
    else if (!event) input_ready.store(true);
    if (input_ready.load() && event && !previous_event_) ++accepted_events;
    previous_event_ = event;
    observed_event.store(event);
  }

  static uint32_t number(const std::vector<uint8_t> &data, size_t offset)
  {
    uint32_t result;
    std::memcpy(&result, data.data() + offset, sizeof(result));
    return result;
  }

  static void append(std::vector<uint8_t> &data, uint32_t value)
  {
    const auto *bytes = reinterpret_cast<const uint8_t *>(&value);
    data.insert(data.end(), bytes, bytes + sizeof(value));
  }

  void serve()
  {
    while (!stopping_.load()) {
      const int client = accept(listener_, nullptr, nullptr);
      client_.store(client);
      if (client < 0) return;
      if (stopping_.load()) {
        close(client);
        client_.store(-1);
        return;
      }
      std::array<uint8_t, 6> tcp{};
      while (recv(client, tcp.data(), tcp.size(), MSG_WAITALL) == 6) {
        uint32_t length;
        std::memcpy(&length, tcp.data() + 2, sizeof(length));
        if (length < 32 || length > 4096) break;
        std::vector<uint8_t> request(length);
        if (recv(client, request.data(), length, MSG_WAITALL) != static_cast<int>(length)) break;
        const auto command = request[16];
        std::vector<uint8_t> response(32);
        std::copy_n(request.begin() + 8, 8, response.begin());
        std::copy_n(request.begin(), 8, response.begin() + 8);
        response[16] = command;
        response[18] = 5;  // ADS command and response flags.
        std::copy_n(request.begin() + 28, 4, response.begin() + 28);
        append(response, 0);  // ADS result.
        if (command == 4) {
          append(response, 5);  // ADS RUN / device state 0.
        } else if (command == 9) {
          const uint32_t group = number(request, 32);
          if (group == 0xF003) {
            append(response, 4);
            const std::string symbol(reinterpret_cast<const char *>(request.data() + 48), request.size() - 48);
            const uint32_t handle = symbol.find("boot_id") != std::string::npos ? 6 :
              symbol.find("calibration_valid") != std::string::npos ? 12 :
              symbol.find("calibration_entry") != std::string::npos ? 11 :
              symbol.find("boot_echo") != std::string::npos ? 7 :
              symbol.find("observed_event") != std::string::npos ? 8 :
              symbol.find("accepted_events") != std::string::npos ? 9 :
              symbol.find("input_ready") != std::string::npos ? 10 :
              symbol.find("heartbeat") != std::string::npos ? 3 :
              symbol.find("sequence") != std::string::npos ? 4 :
              symbol.find("timestamp") != std::string::npos ? 5 :
              symbol.find("other") != std::string::npos ? 2 : 1;
            append(response, handle);
          } else if (group == 0xF080) {
            ++read_requests;
            while (hold_reads.load() && !stopping_.load()) std::this_thread::sleep_for(1ms);
            scan();
            if (drop_read_reply.exchange(false)) {
              ++dropped_read_replies;
              continue;
            }
            const uint32_t count = number(request, 36);
            uint32_t size = 4 * count;
            for (uint32_t i = 0; i < count; ++i) size += number(request, 56 + 12 * i);
            append(response, size);
            const bool failing = fail_reads.load();
            for (uint32_t i = 0; i < count; ++i) {
              append(response, failing && number(request, 52 + 12 * i) == 1 ? 0x710 : 0);
            }
            for (uint32_t i = 0; i < count; ++i) {
              if (number(request, 52 + 12 * i) == 1) {
                for (size_t k = 0; k < value.size(); ++k) {
                  response.push_back(failing && k == 2 ? 199 : value[k].load());
                }
              } else if (number(request, 52 + 12 * i) == 6) {
                for (const auto &word : boot_id) append(response, word.load());
              } else if (number(request, 52 + 12 * i) == 8) {
                response.push_back(observed_event.load());
              } else if (number(request, 52 + 12 * i) == 9) {
                append(response, accepted_events.load());
              } else if (number(request, 52 + 12 * i) == 10) {
                response.push_back(input_ready.load());
              } else if (number(request, 52 + 12 * i) == 11) {
                for (const auto &coordinate : calibration_xyz) {
                  const double coordinate_value = coordinate.load();
                  const auto *bytes = reinterpret_cast<const uint8_t *>(&coordinate_value);
                  response.insert(response.end(), bytes, bytes + sizeof(coordinate_value));
                }
              } else if (number(request, 52 + 12 * i) == 12) {
                response.push_back(calibration_valid.load());
              } else {
                response.push_back(failing ? 88 : scalar.load());
              }
            }
          } else if (group == 0xF081) {
            const uint32_t count = number(request, 36);
            size_t data = 48 + count * 12;
            unsigned metadata = 0;
            bool scalar_written = false;
            uint8_t scalar_value = 0;
            for (uint32_t i = 0; i < count; ++i) {
              const uint32_t size = number(request, 56 + i * 12);
              const auto handle = number(request, 52 + i * 12);
              if (handle == 1) {
                if (request[data] != 0) ++positive_array_writes;
                for (size_t k = 0; k < value.size(); ++k) value[k].store(request[data + k]);
              } else if (handle == 2) {
                scalar.store(request[data]);
                scalar_written = true;
                scalar_value = request[data];
              } else if (handle == 3) {
                heartbeat.store(number(request, data)); metadata |= 1;
              } else if (handle == 4) {
                sequence.store(number(request, data)); metadata |= 2;
              } else if (handle == 5) {
                double stamp;
                std::memcpy(&stamp, request.data() + data, sizeof(stamp));
                timestamp.store(stamp); metadata |= 4;
              } else if (handle == 7) {
                for (size_t k = 0; k < boot_echo.size(); ++k) {
                  boot_echo[k].store(number(request, data + k * sizeof(uint32_t)));
                }
              }
              data += size;
            }
            if (metadata == 7 && scalar_written) {
              metadata_scalar.store(scalar_value);
              ++metadata_writes;
            }
            ++writes;
            while (hold_writes.load() && !stopping_.load()) std::this_thread::sleep_for(1ms);
            if (drop_write_reply.exchange(false)) continue;
            append(response, count * 4);
            for (uint32_t i = 0; i < count; ++i) append(response, 0);
          } else {
            break;
          }
        } else if (command != 3) {  // Handle release uses ADS Write.
          break;
        }
        const uint32_t dataSize = response.size() - 32;
        std::memcpy(response.data() + 20, &dataSize, sizeof(dataSize));
        const uint32_t frameSize = response.size();
        std::memcpy(tcp.data() + 2, &frameSize, sizeof(frameSize));
        std::vector<uint8_t> frame(tcp.begin(), tcp.end());
        frame.insert(frame.end(), response.begin(), response.end());
        if (send(client, frame.data(), frame.size(), MSG_NOSIGNAL) != static_cast<int>(frame.size())) break;
      }
      close(client);
      client_.store(-1);
    }
  }

  int listener_;
  std::atomic<int> client_{-1};
  std::atomic<bool> stopping_{false};
  std::thread thread_;
  uint32_t scanned_boot_{0};
  bool previous_event_{false};
};

TEST(CommandReadback, StockReplayPreservesRawCommandsAndRequiresPlcEventAdmission)
{
  rclcpp::init(0, nullptr);
  LoopbackPlc plc;
  beckhoff_ads_hardware_interface::BeckhoffADSHardwareInterface driver;
  hardware_interface::HardwareComponentParams params;
  params.clock = std::make_shared<rclcpp::Clock>();
  const std::string description = R"(
    <robot name="readback"><link name="base"/>
      <ros2_control name="loopback" type="system">
        <hardware>
          <plugin>beckhoff_ads_hardware_interface/BeckhoffADSHardwareInterface</plugin>
          <param name="plc_ip_address">127.0.0.1</param>
          <param name="plc_ams_net_id">127.0.0.1.1.1</param>
          <param name="local_ams_net_id">127.0.0.1.1.2</param>
          <param name="plc_ams_port">851</param>
          <param name="read_poll_period_ms">2</param>
        </hardware>
        <gpio name="io">
          <command_interface name="unpaired"><param name="PLC_symbol">MAIN.commands</param><param name="PLC_type">USINT</param><param name="n_elements">3</param><param name="index">0</param><param name="command_fallback">none</param></command_interface>
          <command_interface name="paired"><param name="PLC_symbol">MAIN.commands</param><param name="PLC_type">USINT</param><param name="n_elements">3</param><param name="index">2</param><param name="command_fallback">none</param></command_interface>
          <command_interface name="scalar"><param name="PLC_symbol">MAIN.other</param><param name="PLC_type">USINT</param><param name="command_fallback">none</param></command_interface>
          <command_interface name="boot_echo_0"><param name="PLC_symbol">MAIN.boot_echo</param><param name="PLC_type">UDINT</param><param name="n_elements">4</param><param name="index">0</param><param name="command_fallback">none</param></command_interface>
          <command_interface name="boot_echo_1"><param name="PLC_symbol">MAIN.boot_echo</param><param name="PLC_type">UDINT</param><param name="n_elements">4</param><param name="index">1</param><param name="command_fallback">none</param></command_interface>
          <command_interface name="boot_echo_2"><param name="PLC_symbol">MAIN.boot_echo</param><param name="PLC_type">UDINT</param><param name="n_elements">4</param><param name="index">2</param><param name="command_fallback">none</param></command_interface>
          <command_interface name="boot_echo_3"><param name="PLC_symbol">MAIN.boot_echo</param><param name="PLC_type">UDINT</param><param name="n_elements">4</param><param name="index">3</param><param name="command_fallback">none</param></command_interface>
          <state_interface name="readback"><param name="PLC_symbol">MAIN.commands</param><param name="PLC_type">USINT</param><param name="n_elements">3</param><param name="index">2</param><param name="critical">true</param></state_interface>
          <state_interface name="readback_alias"><param name="PLC_symbol">MAIN.commands</param><param name="PLC_type">USINT</param><param name="n_elements">3</param><param name="index">2</param><param name="critical">true</param></state_interface>
          <state_interface name="other"><param name="PLC_symbol">MAIN.other</param><param name="PLC_type">USINT</param><param name="critical">true</param></state_interface>
          <state_interface name="boot_id_0"><param name="PLC_symbol">MAIN.boot_id</param><param name="PLC_type">UDINT</param><param name="n_elements">4</param><param name="index">0</param><param name="critical">true</param></state_interface>
          <state_interface name="boot_id_1"><param name="PLC_symbol">MAIN.boot_id</param><param name="PLC_type">UDINT</param><param name="n_elements">4</param><param name="index">1</param><param name="critical">true</param></state_interface>
          <state_interface name="boot_id_2"><param name="PLC_symbol">MAIN.boot_id</param><param name="PLC_type">UDINT</param><param name="n_elements">4</param><param name="index">2</param><param name="critical">true</param></state_interface>
          <state_interface name="boot_id_3"><param name="PLC_symbol">MAIN.boot_id</param><param name="PLC_type">UDINT</param><param name="n_elements">4</param><param name="index">3</param><param name="critical">true</param></state_interface>
          <state_interface name="observed_event"><param name="PLC_symbol">MAIN.observed_event</param><param name="PLC_type">USINT</param><param name="critical">true</param></state_interface>
          <state_interface name="accepted_events"><param name="PLC_symbol">MAIN.accepted_events</param><param name="PLC_type">UDINT</param><param name="critical">true</param></state_interface>
          <state_interface name="input_ready"><param name="PLC_symbol">MAIN.input_ready</param><param name="PLC_type">USINT</param><param name="critical">true</param></state_interface>
          <state_interface name="x"><param name="PLC_symbol">MAIN.calibration_entry</param><param name="PLC_type">LREAL</param><param name="n_elements">3</param><param name="index">0</param><param name="critical">true</param></state_interface>
          <state_interface name="y"><param name="PLC_symbol">MAIN.calibration_entry</param><param name="PLC_type">LREAL</param><param name="n_elements">3</param><param name="index">1</param><param name="critical">true</param></state_interface>
          <state_interface name="z"><param name="PLC_symbol">MAIN.calibration_entry</param><param name="PLC_type">LREAL</param><param name="n_elements">3</param><param name="index">2</param><param name="critical">true</param></state_interface>
          <state_interface name="valid"><param name="PLC_symbol">MAIN.calibration_valid</param><param name="PLC_type">BOOL</param><param name="critical">true</param></state_interface>
        </gpio>
      </ros2_control>
    </robot>)";
  params.hardware_info = hardware_interface::parse_control_resources_from_urdf(description).front();
  ASSERT_EQ(driver.init(params), hardware_interface::CallbackReturn::SUCCESS);
  const auto state_interfaces = driver.on_export_state_interfaces();
  const auto command_interfaces = driver.on_export_command_interfaces();
  const rclcpp_lifecycle::State state;
  const rclcpp::Duration period(0, 1'000'000);
  gpio_controllers::GpioCommandController controller;
  controller_interface::ControllerInterfaceParams controller_params;
  controller_params.controller_name = "loopback_controller";
  controller_params.node_namespace = "";
  controller_params.update_rate = 50;
  controller_params.robot_description = description;
  controller_params.node_options.parameter_overrides({
    {"gpios", std::vector<std::string>{"io"}},
    {"command_interfaces.io.interfaces", std::vector<std::string>{
      "unpaired", "paired", "scalar", "boot_echo_0", "boot_echo_1", "boot_echo_2", "boot_echo_3"}},
    {"state_interfaces.io.interfaces", std::vector<std::string>{"readback", "other"}}});
  ASSERT_EQ(controller.init(controller_params), controller_interface::return_type::OK);
  ASSERT_EQ(controller.on_configure(state), controller_interface::CallbackReturn::SUCCESS);
  std::vector<hardware_interface::LoanedCommandInterface> command_loans;
  std::vector<hardware_interface::LoanedStateInterface> state_loans;
  for (const auto &interface : command_interfaces) command_loans.emplace_back(interface, nullptr);
  for (const auto &interface : state_interfaces) state_loans.emplace_back(interface);
  controller.assign_interfaces(std::move(command_loans), std::move(state_loans));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller.get_node()->get_node_base_interface());
  auto source = std::make_shared<rclcpp::Node>("command_source");
  auto publisher = source->create_publisher<gpio_controllers::CmdType>(
    "/loopback_controller/commands", rclcpp::SystemDefaultsQoS());
  const auto issue = [&](std::vector<std::string> names, std::vector<double> values) {
    gpio_controllers::CmdType message;
    message.interface_groups = {"io"};
    control_msgs::msg::InterfaceValue group;
    group.interface_names = names;
    group.values = values;
    message.interface_values = {group};
    publisher->publish(message);
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    do {
      executor.spin_some();
      controller.update(params.clock->now(), period);
      bool received = true;
      for (size_t i = 0; i < names.size(); ++i) {
        received = received && driver.get_command<double>("io/" + names[i]) == values[i];
      }
      if (received) return true;
      std::this_thread::sleep_for(2ms);
    } while (std::chrono::steady_clock::now() < deadline);
    return false;
  };
  const auto tick = [&] {
    driver.read(params.clock->now(), period);
    executor.spin_some();
    EXPECT_EQ(controller.update(params.clock->now(), period), controller_interface::return_type::OK);
    return driver.write(params.clock->now(), period);
  };
  const auto wait = [&](const auto &ready) {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    do {
      tick();
      if (ready()) return true;
      std::this_thread::sleep_for(2ms);
    } while (std::chrono::steady_clock::now() < deadline);
    return false;
  };
  for (int expected : {1, 2}) {
    plc.value[2].store(0);
    plc.value[1].store(19);  // This array element is not exported as a ROS command.
    plc.scalar.store(0);
    const auto initial_writes = plc.writes.load();
    ASSERT_EQ(driver.on_configure(state), hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_EQ(driver.read(params.clock->now(), period), hardware_interface::return_type::OK);
    EXPECT_TRUE(std::isnan(driver.get_state<double>("io/readback")));
    if (expected == 1) {
      driver.set_command("io/scalar", 77.0);
      plc.fail_reads.store(true);
      EXPECT_NE(driver.on_activate(state), hardware_interface::CallbackReturn::SUCCESS);
      plc.fail_reads.store(false);
      ASSERT_EQ(driver.on_configure(state), hardware_interface::CallbackReturn::SUCCESS);
    }
    ASSERT_EQ(driver.on_activate(state), hardware_interface::CallbackReturn::SUCCESS);
    if (expected == 1) EXPECT_EQ(driver.get_command<double>("io/scalar"), 77.0);
    ASSERT_EQ(controller.on_activate(state), controller_interface::CallbackReturn::SUCCESS);
    ASSERT_TRUE(wait([&] { return driver.get_state<double>("io/readback") == 0; }));
    if (expected == 1) {
      EXPECT_DOUBLE_EQ(driver.get_state<double>("io/x"), -0.125);
      EXPECT_DOUBLE_EQ(driver.get_state<double>("io/y"), 0.25);
      EXPECT_DOUBLE_EQ(driver.get_state<double>("io/z"), 1.5);
      EXPECT_EQ(driver.get_state<double>("io/valid"), 1);
      plc.calibration_valid.store(0);
      ASSERT_TRUE(wait([&] { return driver.get_state<double>("io/valid") == 0; }));
      // Activation now waits for a read before the initial command can be sent.
      ASSERT_TRUE(wait([&] {
        return plc.writes.load() == initial_writes + 1 && driver.get_state<double>("io/other") == 77;
      }));
    } else {
      EXPECT_EQ(plc.writes.load(), initial_writes);
    }
    ASSERT_TRUE(wait([&] { return publisher->get_subscription_count() > 0; }));
    if (expected == 1) {
      ASSERT_TRUE(issue({"boot_echo_0", "boot_echo_1", "boot_echo_2", "boot_echo_3", "scalar"},
        {1.0, 17.0, 23.0, 29.0, 0.0}));
      ASSERT_TRUE(wait([&] { return driver.get_state<double>("io/input_ready") == 1; }));
      for (size_t i = 0; i < plc.boot_echo.size(); ++i) {
        EXPECT_EQ(plc.boot_echo[i].load(), plc.boot_id[i].load());
      }
      ASSERT_TRUE(issue({"scalar"}, {1.0}));
      ASSERT_TRUE(wait([&] { return driver.get_state<double>("io/accepted_events") == 1; }));
      const auto held_writes = plc.writes.load();
      ASSERT_TRUE(wait([&] { return plc.writes.load() >= held_writes + 3; }));
      EXPECT_EQ(plc.scalar.load(), 1);  // Native consumption never clears raw input.
      EXPECT_EQ(plc.accepted_events.load(), 1);  // Replayed high is still one event.

      // ADS readback of zero alone does not prove the PLC sampled the low edge.
      plc.scan_enabled.store(false);
      ASSERT_TRUE(issue({"scalar"}, {0.0}));
      ASSERT_TRUE(wait([&] { return driver.get_state<double>("io/other") == 0; }));
      EXPECT_EQ(driver.get_state<double>("io/observed_event"), 1);
      plc.scan_enabled.store(true);
      ASSERT_TRUE(wait([&] { return driver.get_state<double>("io/observed_event") == 0; }));
      ASSERT_TRUE(issue({"scalar"}, {1.0}));
      ASSERT_TRUE(wait([&] { return driver.get_state<double>("io/accepted_events") == 2; }));

      // A new PLC identity rejects the controller's cached hazardous high.
      plc.boot_id[0].store(2);
      ASSERT_TRUE(wait([&] {
        return driver.get_state<double>("io/boot_id_0") == 2 &&
               driver.get_state<double>("io/input_ready") == 0;
      }));
      const auto reset_writes = plc.writes.load();
      ASSERT_TRUE(wait([&] { return plc.writes.load() >= reset_writes + 3; }));
      EXPECT_EQ(plc.scalar.load(), 1);
      EXPECT_EQ(plc.accepted_events.load(), 2);
      ASSERT_TRUE(issue({"scalar"}, {0.0}));
      ASSERT_TRUE(wait([&] {
        return driver.get_state<double>("io/other") == 0 &&
               driver.get_state<double>("io/observed_event") == 0;
      }));
      EXPECT_EQ(driver.get_state<double>("io/input_ready"), 0);  // Old echo cannot rearm.
      ASSERT_TRUE(issue({"boot_echo_0", "boot_echo_1", "boot_echo_2", "boot_echo_3", "scalar"},
        {2.0, 17.0, 23.0, 29.0, 1.0}));
      const auto relabelled_writes = plc.writes.load();
      ASSERT_TRUE(wait([&] { return plc.writes.load() >= relabelled_writes + 3; }));
      EXPECT_FALSE(plc.input_ready.load());
      EXPECT_EQ(plc.accepted_events.load(), 2);
      ASSERT_TRUE(issue({"scalar"}, {0.0}));
      ASSERT_TRUE(wait([&] {
        return driver.get_state<double>("io/input_ready") == 1 &&
               driver.get_state<double>("io/observed_event") == 0;
      }));
      ASSERT_TRUE(issue({"scalar"}, {1.0}));
      ASSERT_TRUE(wait([&] { return driver.get_state<double>("io/accepted_events") == 3; }));
    }
    ASSERT_TRUE(issue({"scalar"}, {0.0}));
    ASSERT_TRUE(wait([&] { return driver.get_state<double>("io/other") == 0; }));
    ASSERT_TRUE(issue({"unpaired", "paired"}, {7.0, static_cast<double>(expected)}));
    const auto before = plc.writes.load();
    plc.hold_writes.store(true);
    tick();
    const bool received = wait([&] { return plc.writes.load() > before; });
    tick();  // An empty tick while the old readback is still zero.
    plc.hold_writes.store(false);
    ASSERT_TRUE(received);
    ASSERT_TRUE(wait([&] { return driver.get_state<double>("io/readback") == expected; }));
    EXPECT_GT(plc.writes.load(), before);
    EXPECT_EQ(plc.value[0].load(), 7);
    EXPECT_EQ(plc.value[1].load(), 19);
    EXPECT_EQ(plc.value[2].load(), expected);
    EXPECT_EQ(driver.get_state<double>("io/readback_alias"), expected);
    EXPECT_EQ(plc.scalar.load(), 0);  // Partial messages preserve the last raw value.

    // A new explicit Stop survives later empty ticks while another write is in flight.
    const auto before_stop = plc.writes.load();
    plc.hold_writes.store(true);
    ASSERT_TRUE(issue({"paired"}, {3.0}));
    tick();
    const bool active = wait([&] { return plc.writes.load() > before_stop; });
    ASSERT_TRUE(issue({"paired"}, {0.0}));
    tick();
    tick();
    plc.hold_writes.store(false);
    ASSERT_TRUE(active);
    ASSERT_TRUE(wait([&] {
      return plc.writes.load() > before_stop + 1 &&
             driver.get_state<double>("io/readback") == 0;
    }));
    EXPECT_EQ(plc.value[2].load(), 0);

    const auto before_unpaired_stream = plc.writes.load();
    ASSERT_TRUE(issue({"unpaired"}, {8.0}));
    ASSERT_TRUE(wait([&] {
      return plc.writes.load() >= before_unpaired_stream + 3 && plc.value[0].load() == 8;
    }));
    ASSERT_EQ(controller.on_deactivate(state), controller_interface::CallbackReturn::SUCCESS);
    const auto drain_reads = plc.read_requests.load();
    ASSERT_TRUE(wait([&] { return plc.read_requests.load() >= drain_reads + 3; }));
    ASSERT_EQ(controller.on_activate(state), controller_interface::CallbackReturn::SUCCESS);
    // This field is now absent from the latest controller message.
    plc.value[2].store(expected);
    ASSERT_TRUE(wait([&] { return driver.get_state<double>("io/readback") == expected; }));
    const auto previous = plc.writes.load();
    const auto reading = plc.read_requests.load();
    plc.hold_reads.store(true);
    const bool awaiting_read = wait([&] { return plc.read_requests.load() > reading; });
    ASSERT_TRUE(issue({"unpaired"}, {9.0}));
    tick();
    plc.hold_reads.store(false);
    ASSERT_TRUE(awaiting_read);
    ASSERT_TRUE(wait([&] { return plc.writes.load() > previous && plc.value[0].load() == 9; }));
    EXPECT_EQ(plc.value[2].load(), expected);
    EXPECT_EQ(plc.value[0].load(), 9);
    EXPECT_EQ(plc.scalar.load(), 0);

    ASSERT_TRUE(issue({"scalar"}, {9.0}));  // Received but not written before deactivation.
    ASSERT_EQ(controller.on_deactivate(state), controller_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(driver.on_deactivate(state), hardware_interface::CallbackReturn::SUCCESS);
    const auto before_pause = plc.writes.load();
    std::this_thread::sleep_for(550ms);
    EXPECT_EQ(driver.read(params.clock->now(), period), hardware_interface::return_type::OK);
    EXPECT_TRUE(std::isnan(driver.get_state<double>("io/readback")));
    ASSERT_TRUE(issue({"scalar"}, {8.0}));  // A value placed in the handle while hardware is inactive.
    ASSERT_EQ(driver.on_activate(state), hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_TRUE(std::isnan(driver.get_command<double>("io/scalar")));
    ASSERT_EQ(controller.on_activate(state), controller_interface::CallbackReturn::SUCCESS);
    ASSERT_TRUE(wait([&] { return driver.get_state<double>("io/readback") == expected; }));
    for (int i = 0; i < 5; ++i) {
      tick();
      std::this_thread::sleep_for(2ms);
    }
    EXPECT_EQ(plc.writes.load(), before_pause);
    if (expected == 1) {
      const auto requests = plc.read_requests.load();
      plc.hold_reads.store(true);
      const bool blocked = wait([&] { return plc.read_requests.load() > requests; });
      std::this_thread::sleep_for(550ms);
      const auto result = driver.read(params.clock->now(), period);
      plc.hold_reads.store(false);
      EXPECT_TRUE(blocked);
      EXPECT_EQ(result, hardware_interface::return_type::ERROR);
    } else {
      plc.fail_reads.store(true);
      ASSERT_TRUE(wait([&] {
        return driver.read(params.clock->now(), period) == hardware_interface::return_type::ERROR;
      }));
      EXPECT_NE(driver.get_state<double>("io/readback"), 199);
      EXPECT_NE(driver.get_state<double>("io/other"), 88);
    }
    const auto stopped_writes = plc.writes.load();
    plc.fail_reads.store(false);
    ASSERT_TRUE(issue({"paired"}, {9.0}));
    EXPECT_EQ(tick(), hardware_interface::return_type::ERROR);
    std::this_thread::sleep_for(10ms);
    EXPECT_EQ(plc.writes.load(), stopped_writes);
    ASSERT_EQ(controller.on_deactivate(state), controller_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(driver.on_deactivate(state), hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_EQ(driver.on_activate(state), hardware_interface::CallbackReturn::ERROR);
  }
  EXPECT_EQ(driver.on_shutdown(state), hardware_interface::CallbackReturn::SUCCESS);

  const auto await = [](const auto &ready) {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (!ready() && std::chrono::steady_clock::now() < deadline) std::this_thread::sleep_for(1ms);
    return ready();
  };

  // A fresh process must preserve a held level until a caller explicitly changes it.
  plc.scalar.store(1);
  const auto before_cold_start = plc.writes.load();
  beckhoff_ads_hardware_interface::BeckhoffADSHardwareInterface cold_driver;
  ASSERT_EQ(cold_driver.init(params), hardware_interface::CallbackReturn::SUCCESS);
  cold_driver.on_export_state_interfaces();
  cold_driver.on_export_command_interfaces();
  ASSERT_EQ(cold_driver.on_configure(state), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(cold_driver.on_activate(state), hardware_interface::CallbackReturn::SUCCESS);
  const auto cold_reads = plc.read_requests.load();
  ASSERT_TRUE(await([&] {
    EXPECT_EQ(cold_driver.write(params.clock->now(), period), hardware_interface::return_type::OK);
    return plc.read_requests.load() >= cold_reads + 3;
  }));
  EXPECT_EQ(plc.writes.load(), before_cold_start);
  EXPECT_EQ(plc.scalar.load(), 1);
  cold_driver.set_command("io/unpaired", 4.0);
  ASSERT_EQ(cold_driver.write(params.clock->now(), period), hardware_interface::return_type::OK);
  ASSERT_TRUE(await([&] { return plc.writes.load() == before_cold_start + 1; }));
  EXPECT_EQ(plc.scalar.load(), 1);
  cold_driver.set_command("io/scalar", 0.0);
  ASSERT_EQ(cold_driver.write(params.clock->now(), period), hardware_interface::return_type::OK);
  ASSERT_TRUE(await([&] { return plc.writes.load() == before_cold_start + 2; }));
  EXPECT_EQ(plc.scalar.load(), 0);
  ASSERT_EQ(cold_driver.on_shutdown(state), hardware_interface::CallbackReturn::SUCCESS);

  // Optional stream metadata must remain together through the new command queue.
  auto &settings = params.hardware_info.hardware_parameters;
  settings["heartbeat_plc_symbol"] = "BENCH.heartbeat";
  settings["setpoint_sequence_plc_symbol"] = "BENCH.sequence";
  settings["setpoint_time_plc_symbol"] = "BENCH.timestamp";
  settings["ads_request_timeout_ms"] = "1000";
  settings["read_poll_period_ms"] = "20";
  beckhoff_ads_hardware_interface::BeckhoffADSHardwareInterface metadata_driver;
  ASSERT_EQ(metadata_driver.init(params), hardware_interface::CallbackReturn::SUCCESS);
  metadata_driver.on_export_state_interfaces();
  metadata_driver.on_export_command_interfaces();
  ASSERT_EQ(metadata_driver.on_configure(state), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(metadata_driver.on_activate(state), hardware_interface::CallbackReturn::SUCCESS);
  for (unsigned update = 1; update <= 2; ++update) {
    metadata_driver.set_command("io/scalar", static_cast<double>(update));
    ASSERT_EQ(metadata_driver.write(params.clock->now(), period), hardware_interface::return_type::OK);
    ASSERT_TRUE(await([&] { return plc.metadata_writes.load() == update; }));
    EXPECT_EQ(plc.heartbeat.load(), update);
    EXPECT_EQ(plc.sequence.load(), update);
    EXPECT_GT(plc.timestamp.load(), 0);
    EXPECT_EQ(plc.metadata_scalar.load(), update);
  }
  // Fast scalar writes retain the configured polling cadence.
  const auto reads_before = plc.read_requests.load();
  const auto stream_start = std::chrono::steady_clock::now();
  for (int i = 0; i < 60; ++i) {
    metadata_driver.set_command("io/scalar", 3.0);
    ASSERT_EQ(metadata_driver.write(params.clock->now(), period), hardware_interface::return_type::OK);
    std::this_thread::sleep_for(1ms);
  }
  const auto stream_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - stream_start).count();
  EXPECT_LE(plc.read_requests.load() - reads_before, static_cast<unsigned>(stream_ms / 20 + 2));
  metadata_driver.set_command("io/scalar", std::numeric_limits<double>::infinity());
  EXPECT_EQ(metadata_driver.write(params.clock->now(), period), hardware_interface::return_type::ERROR);
  EXPECT_EQ(metadata_driver.on_activate(state), hardware_interface::CallbackReturn::ERROR);
  ASSERT_EQ(metadata_driver.on_deactivate(state), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(metadata_driver.on_configure(state), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(metadata_driver.on_activate(state), hardware_interface::CallbackReturn::SUCCESS);
  // A delayed response cannot release a queued command even when read() is paused.
  const auto read_count = plc.read_requests.load();
  plc.hold_reads.store(true);
  ASSERT_TRUE(await([&] { return plc.read_requests.load() > read_count; }));
  const auto write_count = plc.writes.load();
  metadata_driver.set_command("io/scalar", 42.0);
  ASSERT_EQ(metadata_driver.write(params.clock->now(), period), hardware_interface::return_type::OK);
  std::this_thread::sleep_for(150ms);
  plc.hold_reads.store(false);
  ASSERT_TRUE(await([&] { return metadata_driver.write(params.clock->now(), period) == hardware_interface::return_type::ERROR; }));
  EXPECT_EQ(plc.writes.load(), write_count);
  EXPECT_EQ(metadata_driver.on_shutdown(state), hardware_interface::CallbackReturn::SUCCESS);

  // Stop during a forced partial-array read must revoke the subsequent write.
  settings["read_poll_period_ms"] = "10000";
  settings["activation_first_sample_timeout_ms"] = "0";
  settings["outage_behaviour"] = "resume";
  beckhoff_ads_hardware_interface::BeckhoffADSHardwareInterface forced_driver;
  ASSERT_EQ(forced_driver.init(params), hardware_interface::CallbackReturn::SUCCESS);
  forced_driver.on_export_state_interfaces();
  forced_driver.on_export_command_interfaces();
  ASSERT_EQ(forced_driver.on_configure(state), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(forced_driver.on_activate(state), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(await([&] {
    return forced_driver.read(params.clock->now(), period) == hardware_interface::return_type::OK &&
           !std::isnan(forced_driver.get_state<double>("io/readback"));
  }));
  const auto before_forced_read = plc.read_requests.load();
  const auto before_forced_write = plc.writes.load();
  plc.hold_reads.store(true);
  forced_driver.set_command("io/unpaired", 42.0);
  ASSERT_EQ(forced_driver.write(params.clock->now(), period), hardware_interface::return_type::OK);
  const bool read_pending = await([&] { return plc.read_requests.load() > before_forced_read; });
  auto deactivated = hardware_interface::CallbackReturn::ERROR;
  std::thread stopping([&] { deactivated = forced_driver.on_deactivate(state); });
  const bool stop_observed = await([&] {
    forced_driver.read(params.clock->now(), period);
    return std::isnan(forced_driver.get_state<double>("io/readback"));
  });
  plc.hold_reads.store(false);
  stopping.join();
  EXPECT_TRUE(read_pending);
  EXPECT_TRUE(stop_observed);
  EXPECT_EQ(deactivated, hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(plc.writes.load(), before_forced_write);

  // Reactivation without waiting for the first sample must expose no previous-session state.
  plc.hold_reads.store(true);
  const auto before_reactivation = plc.read_requests.load();
  ASSERT_EQ(forced_driver.on_activate(state), hardware_interface::CallbackReturn::SUCCESS);
  const bool new_read_pending = await([&] { return plc.read_requests.load() > before_reactivation; });
  EXPECT_EQ(forced_driver.read(params.clock->now(), period), hardware_interface::return_type::OK);
  EXPECT_TRUE(std::isnan(forced_driver.get_state<double>("io/readback")));
  plc.hold_reads.store(false);
  EXPECT_TRUE(new_read_pending);
  ASSERT_EQ(forced_driver.on_deactivate(state), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(forced_driver.on_configure(state), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(forced_driver.on_activate(state), hardware_interface::CallbackReturn::SUCCESS);
  const auto before_uncertain_write = plc.writes.load();
  plc.drop_write_reply.store(true);
  forced_driver.set_command("io/scalar", 1.0);
  ASSERT_EQ(forced_driver.write(params.clock->now(), period), hardware_interface::return_type::OK);
  ASSERT_TRUE(await([&] { return forced_driver.write(params.clock->now(), period) == hardware_interface::return_type::ERROR; }));
  std::this_thread::sleep_for(20ms);
  EXPECT_EQ(plc.writes.load(), before_uncertain_write + 1);
  EXPECT_EQ(forced_driver.on_activate(state), hardware_interface::CallbackReturn::ERROR);
  EXPECT_EQ(forced_driver.on_shutdown(state), hardware_interface::CallbackReturn::SUCCESS);

  // Resume permits streamed commands to recover after a failed write, unlike one-shot commands.
  settings["read_poll_period_ms"] = "2";
  settings["activation_first_sample_timeout_ms"] = "1000";
  settings["ads_request_timeout_ms"] = "50";
  settings["comms_outage_grace_ms"] = "0";
  for (auto &command : params.hardware_info.gpios.front().command_interfaces) {
    if (command.name == "scalar") command.parameters["command_fallback"] = "hold_last";
  }
  beckhoff_ads_hardware_interface::BeckhoffADSHardwareInterface resume_driver;
  ASSERT_EQ(resume_driver.init(params), hardware_interface::CallbackReturn::SUCCESS);
  resume_driver.on_export_state_interfaces();
  resume_driver.on_export_command_interfaces();
  ASSERT_EQ(resume_driver.on_configure(state), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(resume_driver.on_activate(state), hardware_interface::CallbackReturn::SUCCESS);
  const auto before_resume = plc.writes.load();
  plc.drop_write_reply.store(true);
  resume_driver.set_command("io/scalar", 2.0);
  ASSERT_EQ(resume_driver.write(params.clock->now(), period), hardware_interface::return_type::OK);
  ASSERT_TRUE(await([&] { return resume_driver.read(params.clock->now(), period) == hardware_interface::return_type::ERROR; }));
  resume_driver.set_command("io/scalar", 3.0);
  resume_driver.write(params.clock->now(), period);
  ASSERT_TRUE(await([&] {
    return resume_driver.read(params.clock->now(), period) == hardware_interface::return_type::OK &&
           resume_driver.get_state<double>("io/other") == 3.0;
  }));
  EXPECT_EQ(plc.writes.load(), before_resume + 2);
  EXPECT_EQ(resume_driver.write(params.clock->now(), period), hardware_interface::return_type::OK);
  EXPECT_EQ(resume_driver.on_shutdown(state), hardware_interface::CallbackReturn::SUCCESS);

  // L3H tolerates isolated read loss while keeping its write-failure policy strict.
  settings.erase("heartbeat_plc_symbol");
  settings.erase("setpoint_sequence_plc_symbol");
  settings.erase("setpoint_time_plc_symbol");
  settings["outage_behaviour"] = "latch";
  settings["ads_request_timeout_ms"] = "100";
  settings["read_outage_grace_ms"] = "100";
  settings["read_staleness_timeout_ms"] = "500";
  beckhoff_ads_hardware_interface::BeckhoffADSHardwareInterface grace_driver;
  ASSERT_EQ(grace_driver.init(params), hardware_interface::CallbackReturn::SUCCESS);
  grace_driver.on_export_state_interfaces();
  grace_driver.on_export_command_interfaces();
  ASSERT_EQ(grace_driver.on_configure(state), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(grace_driver.on_activate(state), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(grace_driver.read(params.clock->now(), period), hardware_interface::return_type::OK);
  for (const double next_x : {0.125, 0.375}) {
    const auto previous_x = grace_driver.get_state<double>("io/x");
    const auto dropped = plc.dropped_read_replies.load();
    plc.drop_read_reply.store(true);
    ASSERT_TRUE(await([&] { return plc.dropped_read_replies.load() > dropped; }));
    plc.calibration_xyz[0].store(next_x);
    EXPECT_EQ(grace_driver.read(params.clock->now(), period), hardware_interface::return_type::OK);
    EXPECT_EQ(grace_driver.get_state<double>("io/x"), previous_x);
    ASSERT_TRUE(await([&] {
      const auto result = grace_driver.read(params.clock->now(), period);
      return result != hardware_interface::return_type::OK ||
             grace_driver.get_state<double>("io/x") == next_x;
    }));
    ASSERT_EQ(grace_driver.read(params.clock->now(), period), hardware_interface::return_type::OK);
    EXPECT_EQ(grace_driver.get_state<double>("io/x"), next_x);
    // The second loss is a new outage, before the one-second recovery log settles.
    std::this_thread::sleep_for(150ms);
  }
  // Consecutive transport timeouts still latch a fault once the read grace expires.
  const auto before_outage = plc.dropped_read_replies.load();
  const auto before_fault_write = plc.writes.load();
  plc.drop_read_reply.store(true);
  ASSERT_TRUE(await([&] { return plc.dropped_read_replies.load() > before_outage; }));
  plc.drop_read_reply.store(true);
  grace_driver.set_command("io/unpaired", 55.0);
  ASSERT_EQ(grace_driver.write(params.clock->now(), period), hardware_interface::return_type::OK);
  ASSERT_TRUE(await([&] {
    return grace_driver.read(params.clock->now(), period) == hardware_interface::return_type::ERROR;
  }));
  EXPECT_EQ(plc.dropped_read_replies.load(), before_outage + 2);
  std::this_thread::sleep_for(150ms);
  EXPECT_EQ(plc.writes.load(), before_fault_write);
  EXPECT_EQ(grace_driver.on_shutdown(state), hardware_interface::CallbackReturn::SUCCESS);

  // A failed prerequisite read retains unsent data, with a newer Stop taking priority.
  settings["read_poll_period_ms"] = "10000";
  beckhoff_ads_hardware_interface::BeckhoffADSHardwareInterface retry_driver;
  ASSERT_EQ(retry_driver.init(params), hardware_interface::CallbackReturn::SUCCESS);
  retry_driver.on_export_state_interfaces();
  retry_driver.on_export_command_interfaces();
  ASSERT_EQ(retry_driver.on_configure(state), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(retry_driver.on_activate(state), hardware_interface::CallbackReturn::SUCCESS);
  const auto before_retry_read = plc.read_requests.load();
  const auto before_retry_write = plc.writes.load();
  const auto before_positive_write = plc.positive_array_writes.load();
  plc.hold_reads.store(true);
  plc.drop_read_reply.store(true);
  retry_driver.set_command("io/unpaired", 42.0);
  ASSERT_EQ(retry_driver.write(params.clock->now(), period), hardware_interface::return_type::OK);
  ASSERT_TRUE(await([&] { return plc.read_requests.load() > before_retry_read; }));
  retry_driver.set_command("io/unpaired", 0.0);
  ASSERT_EQ(retry_driver.write(params.clock->now(), period), hardware_interface::return_type::OK);
  EXPECT_EQ(plc.writes.load(), before_retry_write);
  plc.hold_reads.store(false);
  ASSERT_TRUE(await([&] { return plc.writes.load() > before_retry_write; }));
  EXPECT_EQ(retry_driver.read(params.clock->now(), period), hardware_interface::return_type::OK);
  EXPECT_EQ(plc.value[0].load(), 0);
  EXPECT_EQ(plc.positive_array_writes.load(), before_positive_write);

  // Without a newer command, the unsent value must survive the same failed read exactly once.
  const auto before_retained_write = plc.writes.load();
  const auto before_retained_drop = plc.dropped_read_replies.load();
  plc.drop_read_reply.store(true);
  retry_driver.set_command("io/unpaired", 41.0);
  ASSERT_EQ(retry_driver.write(params.clock->now(), period), hardware_interface::return_type::OK);
  ASSERT_TRUE(await([&] { return plc.dropped_read_replies.load() > before_retained_drop; }));
  EXPECT_EQ(plc.writes.load(), before_retained_write);
  ASSERT_TRUE(await([&] { return plc.writes.load() > before_retained_write; }));
  EXPECT_EQ(plc.value[0].load(), 41);
  std::this_thread::sleep_for(20ms);
  EXPECT_EQ(plc.writes.load(), before_retained_write + 1);

  // Shutdown during the retry revokes the retained request before it can reach the PLC.
  const auto before_shutdown_read = plc.read_requests.load();
  const auto before_shutdown_write = plc.writes.load();
  const auto before_shutdown_drop = plc.dropped_read_replies.load();
  plc.drop_read_reply.store(true);
  retry_driver.set_command("io/unpaired", 43.0);
  ASSERT_EQ(retry_driver.write(params.clock->now(), period), hardware_interface::return_type::OK);
  ASSERT_TRUE(await([&] { return plc.dropped_read_replies.load() > before_shutdown_drop; }));
  plc.hold_reads.store(true);
  ASSERT_TRUE(await([&] { return plc.read_requests.load() >= before_shutdown_read + 2; }));
  auto shutdown_result = hardware_interface::CallbackReturn::ERROR;
  std::thread shutdown_retry([&] { shutdown_result = retry_driver.on_shutdown(state); });
  const bool retry_revoked = await([&] {
    retry_driver.read(params.clock->now(), period);
    return std::isnan(retry_driver.get_state<double>("io/readback"));
  });
  plc.hold_reads.store(false);
  shutdown_retry.join();
  EXPECT_TRUE(retry_revoked);
  EXPECT_EQ(shutdown_result, hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(plc.writes.load(), before_shutdown_write);

  // Read grace must not relax a lost write reply, including hold-last commands.
  settings["read_poll_period_ms"] = "2";
  beckhoff_ads_hardware_interface::BeckhoffADSHardwareInterface strict_write_driver;
  ASSERT_EQ(strict_write_driver.init(params), hardware_interface::CallbackReturn::SUCCESS);
  strict_write_driver.on_export_state_interfaces();
  strict_write_driver.on_export_command_interfaces();
  ASSERT_EQ(strict_write_driver.on_configure(state), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(strict_write_driver.on_activate(state), hardware_interface::CallbackReturn::SUCCESS);
  const auto before_strict_write = plc.writes.load();
  plc.drop_write_reply.store(true);
  strict_write_driver.set_command("io/scalar", 4.0);
  ASSERT_EQ(strict_write_driver.write(params.clock->now(), period), hardware_interface::return_type::OK);
  ASSERT_TRUE(await([&] {
    return strict_write_driver.read(params.clock->now(), period) == hardware_interface::return_type::ERROR;
  }));
  strict_write_driver.set_command("io/scalar", 5.0);
  EXPECT_EQ(strict_write_driver.write(params.clock->now(), period), hardware_interface::return_type::ERROR);
  EXPECT_EQ(plc.writes.load(), before_strict_write + 1);
  EXPECT_EQ(strict_write_driver.on_shutdown(state), hardware_interface::CallbackReturn::SUCCESS);
  rclcpp::shutdown();
}
