#pragma once

#include <algorithm>
#include <chrono>
#include <cstring>
#include <initializer_list>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "hardware_interface/resource_manager.hpp"
#include "rclcpp/rclcpp.hpp"

#include "fake_ads_server.hpp"

namespace beckhoff_ads_hardware_interface
{
namespace test
{

  template <typename T>
  std::vector<uint8_t> bytesOf(T value)
  {
    std::vector<uint8_t> out(sizeof(T));
    std::memcpy(out.data(), &value, sizeof(T));
    return out;
  }

  template <typename T>
  std::vector<uint8_t> bytesOfArray(std::initializer_list<T> values)
  {
    std::vector<uint8_t> out;
    for (T value : values)
    {
      const auto bytes = bytesOf(value);
      out.insert(out.end(), bytes.begin(), bytes.end());
    }
    return out;
  }

  template <typename T>
  void putAt(std::vector<uint8_t> &buffer, size_t offset, T value)
  {
    std::memcpy(buffer.data() + offset, &value, sizeof(T));
  }

  template <typename T>
  T getAt(const std::vector<uint8_t> &buffer, size_t offset)
  {
    T value{};
    std::memcpy(&value, buffer.data() + offset, sizeof(T));
    return value;
  }

  inline std::string hex(const std::vector<uint8_t> &bytes)
  {
    std::ostringstream out;
    for (uint8_t b : bytes)
    {
      out << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b);
    }
    return out.str();
  }

  // One line per item, sorted by symbol: the PLC sees the same values whatever the item order.
  inline std::string describe(const std::vector<SumItem> &items, const std::set<std::string> &volatile_symbols = {})
  {
    std::vector<std::string> lines;
    for (const auto &item : items)
    {
      std::ostringstream line;
      line << item.symbol << " 0x" << std::hex << item.index_group << std::dec << " " << item.length;
      if (!item.data.empty())
      {
        line << " " << (volatile_symbols.count(item.symbol) ? std::string("*") : hex(item.data));
      }
      lines.push_back(line.str());
    }
    std::sort(lines.begin(), lines.end());
    std::string out;
    for (const auto &line : lines)
    {
      out += line + "\n";
    }
    return out;
  }

  inline std::optional<SumItem> findItem(const std::vector<SumItem> &items, const std::string &symbol)
  {
    const auto it = std::find_if(items.begin(), items.end(), [&symbol](const SumItem &item)
                                 { return item.symbol == symbol; });
    return it == items.end() ? std::nullopt : std::optional<SumItem>(*it);
  }

  inline std::string hardwareBlock(uint16_t port, const std::string &extra = "")
  {
    return R"(
    <hardware>
      <plugin>beckhoff_ads_hardware_interface/BeckhoffADSHardwareInterface</plugin>
      <param name="plc_ip_address">127.0.0.1:)" +
           std::to_string(port) + R"(</param>
      <param name="plc_ams_net_id">10.0.0.1.1.1</param>
      <param name="plc_ams_port">851</param>
      <param name="local_ams_net_id">10.0.0.2.1.1</param>
      <param name="io_thread_scheduling_policy">inherit</param>
      <param name="read_poll_period_ms">2</param>)" +
           extra + R"(
    </hardware>)";
  }

  inline std::string plcParams(const std::string &symbol, const std::string &type, const std::string &extra = "")
  {
    return "<param name=\"PLC_symbol\">" + symbol + "</param><param name=\"PLC_type\">" + type + "</param>" + extra;
  }

  inline std::string param(const std::string &name, const std::string &value)
  {
    return "<param name=\"" + name + "\">" + value + "</param>";
  }

  // Runs the hardware interface through a ResourceManager against a FakeAdsServer.
  class AdsInterfaceTest : public ::testing::Test
  {
  protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void start(const std::string &urdf)
    {
      rm_ = std::make_unique<hardware_interface::ResourceManager>(
          urdf, std::make_shared<rclcpp::Clock>(), rclcpp::get_logger("ads_interface_test"), true, 100);
    }

    std::string componentState(const std::string &component)
    {
      return rm_->get_components_status().at(component).state.label();
    }

    void cycle()
    {
      using namespace std::chrono_literals;
      rm_->read(rclcpp::Time(0), rclcpp::Duration(10ms));
      rm_->write(rclcpp::Time(0), rclcpp::Duration(10ms));
    }

    double state(const std::string &name)
    {
      auto handle = rm_->claim_state_interface(name);
      return handle.get_optional().value_or(std::numeric_limits<double>::quiet_NaN());
    }

    FakeAdsServer server_; // outlives the resource manager, which joins the I/O threads
    std::unique_ptr<hardware_interface::ResourceManager> rm_;
  };

} // namespace test
} // namespace beckhoff_ads_hardware_interface
