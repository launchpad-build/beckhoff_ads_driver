// Copyright (c) 2025, b-robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include <chrono>

#include <gtest/gtest.h>

#include "outage_tracker.hpp"

using beckhoff_ads_client::OutageTracker;

namespace
{

const std::chrono::steady_clock::time_point START{std::chrono::seconds(50)};
constexpr std::chrono::milliseconds GRACE{100};

}  // namespace

TEST(OutageTracker, AFailureInsideTheGraceWindowOnlyInterrupts)
{
  OutageTracker tracker;
  tracker.reset(GRACE, true);
  tracker.record_failure(START);
  tracker.record_failure(START + GRACE / 2);
  EXPECT_TRUE(tracker.interrupted());
  EXPECT_FALSE(tracker.faulted());
  EXPECT_EQ(tracker.consecutive_failures(), 2u);
}

TEST(OutageTracker, AnOutageOutlivingTheGraceWindowFaults)
{
  OutageTracker tracker;
  tracker.reset(GRACE, true);
  tracker.record_failure(START);
  tracker.record_failure(START + GRACE);
  EXPECT_TRUE(tracker.faulted());
}

TEST(OutageTracker, AZeroGraceFaultsOnTheFirstFailure)
{
  OutageTracker tracker;
  tracker.reset(std::chrono::milliseconds(0), true);
  tracker.record_failure(START);
  EXPECT_TRUE(tracker.faulted());
}

TEST(OutageTracker, ALatchedFaultSurvivesRecoveryUntilReset)
{
  OutageTracker tracker;
  tracker.reset(GRACE, true);
  tracker.record_failure(START);
  tracker.record_failure(START + GRACE);
  tracker.record_success(START + 2 * GRACE, false);
  EXPECT_TRUE(tracker.faulted());
  tracker.reset(GRACE, true);
  EXPECT_FALSE(tracker.faulted());
  EXPECT_FALSE(tracker.interrupted());
}

TEST(OutageTracker, AResumingFaultClearsOnTheNextGoodRoundTrip)
{
  OutageTracker tracker;
  tracker.reset(GRACE, false);
  tracker.record_failure(START);
  tracker.record_failure(START + GRACE);
  tracker.record_success(START + 2 * GRACE, false);
  EXPECT_FALSE(tracker.faulted());
}

TEST(OutageTracker, KeepFaultHoldsADeclaredFaultThroughASuccess)
{
  OutageTracker tracker;
  tracker.reset(GRACE, false);
  tracker.declare_fault();
  tracker.record_success(START, true);
  EXPECT_TRUE(tracker.faulted());
  tracker.record_success(START, false);
  EXPECT_FALSE(tracker.faulted());
}

TEST(OutageTracker, RecoveryIsReportedOnceTheLinkStaysGoodForTheStablePeriod)
{
  OutageTracker tracker;
  tracker.reset(GRACE, true);
  tracker.record_failure(START);
  tracker.record_failure(START + std::chrono::milliseconds(20));
  const auto good_since = START + std::chrono::milliseconds(50);
  EXPECT_FALSE(tracker.record_success(good_since, false).has_value());
  EXPECT_FALSE(tracker.record_success(good_since + std::chrono::milliseconds(500), false).has_value());
  EXPECT_TRUE(tracker.interrupted());
  const auto recovery = tracker.record_success(good_since + OutageTracker::RECOVERY_STABLE_PERIOD, false);
  ASSERT_TRUE(recovery.has_value());
  EXPECT_EQ(recovery->failed_round_trips, 2u);
  EXPECT_EQ(recovery->duration, std::chrono::milliseconds(50));
  EXPECT_FALSE(tracker.interrupted());
  EXPECT_EQ(tracker.consecutive_failures(), 0u);
}

TEST(OutageTracker, AFailureDuringTheStablePeriodRestartsIt)
{
  OutageTracker tracker;
  tracker.reset(std::chrono::seconds(10), true);
  tracker.record_failure(START);
  tracker.record_success(START + std::chrono::milliseconds(10), false);
  tracker.record_failure(START + std::chrono::milliseconds(500));
  tracker.record_success(START + std::chrono::milliseconds(600), false);
  EXPECT_FALSE(tracker.record_success(START + std::chrono::milliseconds(1200), false).has_value());
  EXPECT_TRUE(tracker.record_success(START + std::chrono::milliseconds(1600), false).has_value());
}
