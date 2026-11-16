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
#include <limits>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "beckhoff_ads_client/ads_client.hpp"
#include "beckhoff_ads_client/testing/fake_ads_server.hpp"

using beckhoff_ads_client::AdsClient;
using beckhoff_ads_client::ClientSettings;
using beckhoff_ads_client::ClientState;
using beckhoff_ads_client::ConfigureFailure;
using beckhoff_ads_client::FirstSampleOutcome;
using beckhoff_ads_client::LogSeverity;
using beckhoff_ads_client::OutageBehaviour;
using beckhoff_ads_client::PlcType;
using beckhoff_ads_client::SymbolElement;

namespace
{

constexpr double NOT_PROVIDED = std::numeric_limits<double>::quiet_NaN();

bool wait_until(const std::function<bool()> & condition, std::chrono::milliseconds timeout = std::chrono::seconds(3))
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  bool satisfied = condition();
  while (!satisfied && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    satisfied = condition();
  }
  return satisfied;
}

SymbolElement element(
  const std::string & symbol, PlcType type, std::size_t count, std::size_t index, const std::string & label)
{
  SymbolElement result;
  result.symbol = symbol;
  result.type = type;
  result.element_count = count;
  result.element_index = index;
  result.label = label;
  return result;
}

class AdsClientWithFakePlc : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ASSERT_TRUE(server_.listening());
    server_.add_symbol("MAIN.positions", 24);
    server_.add_symbol("MAIN.enabled", 1);
    server_.add_symbol("MAIN.targets", 16);
    server_.add_symbol("MAIN.mode", 2);
    server_.add_symbol("MAIN.heartbeat", 4);
    server_.add_symbol("MAIN.sequence", 4);
    server_.add_symbol("MAIN.stamp", 8);
    server_.set_value<double>("MAIN.positions", 0, 1.5);
    server_.set_value<double>("MAIN.positions", 16, 2.5);
    server_.set_value<std::uint8_t>("MAIN.enabled", 0, 1);
  }

  ClientSettings settings()
  {
    ClientSettings result;
    result.connection.plc_ip_address = "127.0.0.1";
    result.connection.plc_ams_net_id = "127.0.0.1.1.1";
    result.connection.plc_ams_port = 851;
    result.connection.local_ams_net_id = "127.0.0.2.1.1";
    result.connection.request_timeout = std::chrono::milliseconds(50);
    result.io_thread_scheduling.policy = beckhoff_ads_client::utilities::SchedulingPolicy::INHERIT;
    result.log = [this](LogSeverity severity, std::string_view message) {
        const std::lock_guard<std::mutex> lock(log_mutex_);
        if (severity >= LogSeverity::Warning) {
          problems_.emplace_back(message);
        }
      };
    return result;
  }

  static std::vector<SymbolElement> read_elements()
  {
    std::vector<SymbolElement> result{
      element("MAIN.positions", PlcType::Lreal, 3, 0, "x/position"),
      element("MAIN.positions", PlcType::Lreal, 3, 2, "z/position"),
      element("MAIN.enabled", PlcType::Bool, 1, 0, "enabled"),
    };
    result[0].critical = true;
    return result;
  }

  static std::vector<SymbolElement> write_elements()
  {
    return {
      element("MAIN.targets", PlcType::Lreal, 2, 0, "x/target"),
      element("MAIN.targets", PlcType::Lreal, 2, 1, "y/target"),
      element("MAIN.mode", PlcType::Int, 1, 0, "mode"),
    };
  }

  beckhoff_ads_client::testing::FakeAdsServer server_;
  std::mutex log_mutex_;
  std::vector<std::string> problems_;
};

}  // namespace

TEST_F(AdsClientWithFakePlc, StreamsWholeSamplesOfTheReadElements)
{
  AdsClient client;
  const auto result = client.configure(settings(), read_elements(), write_elements());
  ASSERT_TRUE(result.succeeded()) << result.error;
  EXPECT_EQ(client.state(), ClientState::Stopped);
  EXPECT_EQ(client.read_item_count(), 2u);
  EXPECT_EQ(client.write_item_count(), 2u);

  ASSERT_TRUE(client.start());
  ASSERT_EQ(client.wait_for_first_sample(std::chrono::seconds(2)), FirstSampleOutcome::Received);
  std::vector<double> values(3, NOT_PROVIDED);
  const auto sample = client.latest_sample(values);
  ASSERT_TRUE(sample.has_value());
  EXPECT_GT(sample->sequence, 0u);
  EXPECT_FALSE(sample->stale);
  EXPECT_DOUBLE_EQ(values[0], 1.5);
  EXPECT_DOUBLE_EQ(values[1], 2.5);
  EXPECT_DOUBLE_EQ(values[2], 1.0);
  EXPECT_EQ(client.state(), ClientState::Running);

  server_.set_value<double>("MAIN.positions", 0, -4.0);
  EXPECT_TRUE(wait_until([&] {client.latest_sample(values); return values[0] == -4.0;}));
  EXPECT_GT(client.statistics().read_transactions, 0u);
  client.stop();
  EXPECT_EQ(client.state(), ClientState::Stopped);
}

TEST_F(AdsClientWithFakePlc, AMissingOptionalSymbolIsDroppedAndAMissingRequiredOneFails)
{
  std::vector<SymbolElement> reads = read_elements();
  reads.push_back(element("MAIN.not_flashed_yet", PlcType::Dint, 1, 0, "diagnostic"));
  reads.back().optional = true;
  AdsClient client;
  const auto result = client.configure(settings(), reads, write_elements());
  ASSERT_TRUE(result.succeeded()) << result.error;
  EXPECT_EQ(result.dropped_read_elements, (std::vector<std::size_t>{3}));
  ASSERT_TRUE(client.start());
  ASSERT_EQ(client.wait_for_first_sample(std::chrono::seconds(2)), FirstSampleOutcome::Received);
  std::vector<double> values(4, 0.0);
  ASSERT_TRUE(client.latest_sample(values).has_value());
  EXPECT_TRUE(std::isnan(values[3]));
  EXPECT_DOUBLE_EQ(values[0], 1.5);

  reads.back().optional = false;
  const auto failed = client.configure(settings(), reads, write_elements());
  EXPECT_EQ(failed.failure, ConfigureFailure::RequiredSymbolMissing);
  EXPECT_EQ(client.state(), ClientState::Disconnected);
}

TEST_F(AdsClientWithFakePlc, AValueIsNeverSentBeforeEveryElementOfItsSymbolIsProvided)
{
  AdsClient client;
  ASSERT_TRUE(client.configure(settings(), read_elements(), write_elements()).succeeded());
  ASSERT_TRUE(client.start());

  const std::vector<double> only_mode{NOT_PROVIDED, NOT_PROVIDED, 3.0};
  client.submit(only_mode);
  ASSERT_TRUE(wait_until([&] {return server_.writes_to("MAIN.mode") > 0;}));
  EXPECT_EQ(server_.value<std::int16_t>("MAIN.mode", 0), 3);

  const std::vector<double> half_target{7.0, NOT_PROVIDED, NOT_PROVIDED};
  client.submit(half_target);
  ASSERT_TRUE(wait_until([&] {return server_.writes_to("MAIN.mode") > 1;}));
  EXPECT_EQ(server_.writes_to("MAIN.targets"), 0u);

  const std::vector<double> other_half{NOT_PROVIDED, 8.0, NOT_PROVIDED};
  client.submit(other_half);
  ASSERT_TRUE(wait_until([&] {return server_.writes_to("MAIN.targets") > 0;}));
  EXPECT_DOUBLE_EQ(server_.value<double>("MAIN.targets", 0), 7.0);
  EXPECT_DOUBLE_EQ(server_.value<double>("MAIN.targets", 8), 8.0);
  EXPECT_EQ(server_.value<std::int16_t>("MAIN.mode", 0), 3);
}

TEST_F(AdsClientWithFakePlc, InitialValuesAreSentBeforeTheFirstProvidedValue)
{
  std::vector<SymbolElement> writes = write_elements();
  writes[2].initial_value = 5.0;
  AdsClient client;
  ASSERT_TRUE(client.configure(settings(), read_elements(), writes).succeeded());
  ASSERT_TRUE(client.start());
  const std::vector<double> nothing(3, NOT_PROVIDED);
  client.submit(nothing);
  ASSERT_TRUE(wait_until([&] {return server_.writes_to("MAIN.mode") > 0;}));
  EXPECT_EQ(server_.value<std::int16_t>("MAIN.mode", 0), 5);
  EXPECT_EQ(server_.writes_to("MAIN.targets"), 0u);
}

TEST_F(AdsClientWithFakePlc, HeartbeatSequenceAndTimestampTravelWithEverySubmit)
{
  ClientSettings annotated = settings();
  annotated.heartbeat_symbol = "MAIN.heartbeat";
  annotated.setpoint_sequence_symbol = "MAIN.sequence";
  annotated.setpoint_timestamp_symbol = "MAIN.stamp";
  AdsClient client;
  ASSERT_TRUE(client.configure(annotated, read_elements(), write_elements()).succeeded());
  EXPECT_EQ(client.write_item_count(), 5u);
  ASSERT_TRUE(client.start());

  const std::vector<double> nothing(3, NOT_PROVIDED);
  for (int submit = 0; submit < 5; ++submit) {
    client.submit(nothing);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_TRUE(wait_until([&] {return server_.value<std::uint32_t>("MAIN.heartbeat", 0) == 5u;}));
  EXPECT_EQ(server_.value<std::uint32_t>("MAIN.sequence", 0), 5u);
  EXPECT_GT(server_.value<double>("MAIN.stamp", 0), 0.0);
  EXPECT_EQ(client.statistics().heartbeat, 5u);
  EXPECT_EQ(client.statistics().setpoint_sequence, 5u);
  EXPECT_EQ(server_.writes_to("MAIN.targets"), 0u);
}

TEST_F(AdsClientWithFakePlc, SubmitsFasterThanTheLinkAreCoalesced)
{
  AdsClient client;
  ASSERT_TRUE(client.configure(settings(), read_elements(), write_elements()).succeeded());
  ASSERT_TRUE(client.start());
  constexpr int SUBMISSIONS = 5000;
  for (int submit = 0; submit < SUBMISSIONS; ++submit) {
    const std::vector<double> values{static_cast<double>(submit), 0.0, 1.0};
    client.submit(values);
  }
  ASSERT_TRUE(wait_until([&] {return server_.value<double>("MAIN.targets", 0) == SUBMISSIONS - 1;}));
  const auto statistics = client.statistics();
  EXPECT_GT(statistics.coalesced_writes, 0u);
  EXPECT_LE(statistics.write_transactions + statistics.coalesced_writes, static_cast<std::uint64_t>(SUBMISSIONS));
  EXPECT_EQ(statistics.write_failures, 0u);
}

TEST_F(AdsClientWithFakePlc, AnOutageOutlivingTheGraceWindowLatchesAFaultUntilRestart)
{
  AdsClient client;
  ASSERT_TRUE(client.configure(settings(), read_elements(), write_elements()).succeeded());
  ASSERT_TRUE(client.start());
  ASSERT_EQ(client.wait_for_first_sample(std::chrono::seconds(2)), FirstSampleOutcome::Received);

  server_.set_responding(false);
  ASSERT_TRUE(wait_until([&] {return client.read_faulted();}));
  EXPECT_EQ(client.state(), ClientState::Faulted);
  EXPECT_GT(client.statistics().read_failures, 0u);

  server_.set_responding(true);
  const auto read_transactions = client.statistics().read_transactions;
  ASSERT_TRUE(wait_until([&] {return client.statistics().samples_published > 0 &&
      client.statistics().read_transactions > read_transactions + 20;}));
  EXPECT_TRUE(client.read_faulted());

  client.stop();
  ASSERT_TRUE(client.start());
  ASSERT_EQ(client.wait_for_first_sample(std::chrono::seconds(2)), FirstSampleOutcome::Received);
  EXPECT_FALSE(client.read_faulted());
}

TEST_F(AdsClientWithFakePlc, AResumingClientClearsTheFaultWhenTheLinkRecovers)
{
  ClientSettings resuming = settings();
  resuming.outage_behaviour = OutageBehaviour::Resume;
  AdsClient client;
  ASSERT_TRUE(client.configure(resuming, read_elements(), write_elements()).succeeded());
  ASSERT_TRUE(client.start());
  ASSERT_EQ(client.wait_for_first_sample(std::chrono::seconds(2)), FirstSampleOutcome::Received);

  server_.set_responding(false);
  ASSERT_TRUE(wait_until([&] {return client.read_faulted();}));
  server_.set_responding(true);
  EXPECT_TRUE(wait_until([&] {return !client.read_faulted();}));
}

TEST_F(AdsClientWithFakePlc, OnlyACriticalElementThatKeepsFailingFaultsReading)
{
  AdsClient client;
  ASSERT_TRUE(client.configure(settings(), read_elements(), write_elements()).succeeded());
  ASSERT_TRUE(client.start());
  ASSERT_EQ(client.wait_for_first_sample(std::chrono::seconds(2)), FirstSampleOutcome::Received);

  server_.fail_symbol("MAIN.enabled", beckhoff_ads_client::testing::DEVICE_INVALID_ACCESS);
  server_.set_value<std::uint8_t>("MAIN.enabled", 0, 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_FALSE(client.read_faulted());
  std::vector<double> values(3, NOT_PROVIDED);
  ASSERT_TRUE(client.latest_sample(values).has_value());
  EXPECT_DOUBLE_EQ(values[2], 1.0);

  server_.fail_symbol("MAIN.positions", beckhoff_ads_client::testing::DEVICE_INVALID_ACCESS);
  EXPECT_TRUE(wait_until([&] {return client.read_faulted();}));
}

TEST_F(AdsClientWithFakePlc, ASampleOlderThanTheStalenessTimeoutIsReportedStale)
{
  ClientSettings patient = settings();
  patient.connection.request_timeout = std::chrono::milliseconds(1000);
  patient.outage_grace = std::chrono::milliseconds(10000);
  patient.read_staleness_timeout = std::chrono::milliseconds(100);
  AdsClient client;
  ASSERT_TRUE(client.configure(patient, read_elements(), {}).succeeded());
  ASSERT_TRUE(client.start());
  ASSERT_EQ(client.wait_for_first_sample(std::chrono::seconds(2)), FirstSampleOutcome::Received);

  server_.set_responding(false);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  std::vector<double> values(3, NOT_PROVIDED);
  const auto sample = client.latest_sample(values);
  ASSERT_TRUE(sample.has_value());
  EXPECT_TRUE(sample->stale);
  EXPECT_EQ(client.state(), ClientState::Faulted);
  EXPECT_FALSE(client.read_faulted());
  server_.set_responding(true);
}
