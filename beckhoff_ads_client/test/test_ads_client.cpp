// Copyright (c) 2025, b-robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

#include <gtest/gtest.h>

#include "beckhoff_ads_client/ads_client.hpp"

using beckhoff_ads_client::AdsClient;
using beckhoff_ads_client::ClientSettings;
using beckhoff_ads_client::ClientState;
using beckhoff_ads_client::ConfigureFailure;
using beckhoff_ads_client::FirstSampleOutcome;
using beckhoff_ads_client::LogSeverity;
using beckhoff_ads_client::PlcType;
using beckhoff_ads_client::SymbolElement;

namespace
{

struct CapturedMessages
{
  std::vector<std::string> fatal;
};

ClientSettings unreachable_settings(CapturedMessages & messages)
{
  ClientSettings settings;
  settings.connection.plc_ip_address = "127.0.0.1";
  settings.connection.plc_ams_net_id = "127.0.0.1.1.1";
  settings.connection.plc_ams_port = 851;
  settings.connection.local_ams_net_id = "127.0.0.2.1.1";
  settings.connection.request_timeout = std::chrono::milliseconds(50);
  settings.log = [&messages](LogSeverity severity, std::string_view message) {
      if (severity == LogSeverity::Fatal) {
        messages.fatal.emplace_back(message);
      }
    };
  return settings;
}

std::vector<SymbolElement> one_position()
{
  SymbolElement element;
  element.symbol = "MAIN.position";
  element.type = PlcType::Lreal;
  element.label = "x/position";
  return {element};
}

}  // namespace

TEST(AdsClient, AnUnconfiguredClientIsDisconnectedAndInert)
{
  AdsClient client;
  EXPECT_EQ(client.state(), ClientState::Disconnected);
  EXPECT_FALSE(client.start());
  EXPECT_EQ(client.read_item_count(), 0u);
  EXPECT_EQ(client.write_item_count(), 0u);
  std::vector<double> values{4.0};
  EXPECT_FALSE(client.latest_sample(values).has_value());
  EXPECT_DOUBLE_EQ(values[0], 4.0);
  const std::vector<double> commands{1.0};
  client.submit(commands);
  EXPECT_EQ(client.statistics().setpoint_sequence, 0u);
  EXPECT_EQ(client.wait_for_first_sample(std::chrono::milliseconds(10)), FirstSampleOutcome::NothingToRead);
  client.stop();
  client.disconnect();
  EXPECT_EQ(client.state(), ClientState::Disconnected);
}

TEST(AdsClient, ConfigureRejectsAMalformedNetIdBeforeConnecting)
{
  CapturedMessages messages;
  ClientSettings settings = unreachable_settings(messages);
  settings.connection.plc_ams_net_id = "127.0.0.1.1";
  AdsClient client;
  const auto result = client.configure(settings, one_position(), {});
  EXPECT_EQ(result.failure, ConfigureFailure::InvalidSettings);
  EXPECT_NE(result.error.find("127.0.0.1.1"), std::string::npos);
  EXPECT_EQ(client.state(), ClientState::Disconnected);
  EXPECT_FALSE(messages.fatal.empty());
}

TEST(AdsClient, ConfigureRejectsAPortOfZero)
{
  CapturedMessages messages;
  ClientSettings settings = unreachable_settings(messages);
  settings.connection.plc_ams_port = 0;
  AdsClient client;
  EXPECT_EQ(client.configure(settings, one_position(), {}).failure, ConfigureFailure::InvalidSettings);
}

TEST(AdsClient, ConfigureRejectsAnElementOutsideItsSymbol)
{
  CapturedMessages messages;
  std::vector<SymbolElement> elements = one_position();
  elements[0].element_count = 2;
  elements[0].element_index = 2;
  AdsClient client;
  const auto result = client.configure(unreachable_settings(messages), elements, {});
  EXPECT_EQ(result.failure, ConfigureFailure::InvalidSettings);
  EXPECT_NE(result.error.find("x/position"), std::string::npos);
}

TEST(AdsClient, ConfigureRejectsAnElementWithoutASymbol)
{
  CapturedMessages messages;
  std::vector<SymbolElement> elements = one_position();
  elements[0].symbol.clear();
  AdsClient client;
  EXPECT_EQ(client.configure(unreachable_settings(messages), {}, elements).failure, ConfigureFailure::InvalidSettings);
}

TEST(AdsClient, ConfigureFailsCleanlyAndQuicklyAgainstAnUnreachablePlc)
{
  CapturedMessages messages;
  AdsClient client;
  const auto started = std::chrono::steady_clock::now();
  const auto result = client.configure(unreachable_settings(messages), one_position(), one_position());
  const auto elapsed = std::chrono::steady_clock::now() - started;

  EXPECT_EQ(result.failure, ConfigureFailure::ConnectionFailed);
  EXPECT_FALSE(result.error.empty());
  EXPECT_FALSE(messages.fatal.empty());
  EXPECT_LT(elapsed, std::chrono::seconds(3));
  EXPECT_EQ(client.state(), ClientState::Disconnected);
  EXPECT_FALSE(client.start());
  EXPECT_EQ(client.read_item_count(), 0u);
  std::vector<double> values{std::numeric_limits<double>::quiet_NaN()};
  EXPECT_FALSE(client.latest_sample(values).has_value());
}

TEST(AdsClient, AFailedConfigureCanBeRetried)
{
  CapturedMessages messages;
  AdsClient client;
  EXPECT_FALSE(client.configure(unreachable_settings(messages), one_position(), {}).succeeded());
  EXPECT_FALSE(client.configure(unreachable_settings(messages), one_position(), {}).succeeded());
  EXPECT_EQ(client.state(), ClientState::Disconnected);
}
