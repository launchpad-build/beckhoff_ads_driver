// Copyright (c) 2025, b-robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include <chrono>
#include <limits>

#include <gtest/gtest.h>

#include "beckhoff_ads_hardware_interface/ads_interface_utilities.hpp"

namespace utilities = beckhoff_ads_hardware_interface::utilities;

TEST(ParseAmsPort, AcceptsTheFullValidRange)
{
  const utilities::AmsPortParseResult low = utilities::parseAmsPort("1");
  EXPECT_TRUE(low.valid);
  EXPECT_EQ(low.port, 1);

  const utilities::AmsPortParseResult typical = utilities::parseAmsPort("851");
  EXPECT_TRUE(typical.valid);
  EXPECT_EQ(typical.port, 851);

  const utilities::AmsPortParseResult high = utilities::parseAmsPort("65535");
  EXPECT_TRUE(high.valid);
  EXPECT_EQ(high.port, 65535);
}

TEST(ParseAmsPort, RejectsValuesAboveTheUint16Range)
{
  EXPECT_FALSE(utilities::parseAmsPort("65536").valid);
  EXPECT_FALSE(utilities::parseAmsPort("100000").valid);
}

TEST(ParseAmsPort, RejectsZeroAndMalformedInput)
{
  EXPECT_FALSE(utilities::parseAmsPort("0").valid);
  EXPECT_FALSE(utilities::parseAmsPort("").valid);
  EXPECT_FALSE(utilities::parseAmsPort("abc").valid);
  EXPECT_FALSE(utilities::parseAmsPort("851x").valid);
  EXPECT_FALSE(utilities::parseAmsPort("-1").valid);
}

TEST(ParseThreadScheduling, EmptyInputsYieldFifoAtPriorityFifty)
{
  const utilities::ThreadSchedulingParseResult result = utilities::parseThreadScheduling("", "", "");
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(result.config.policy, utilities::SchedulingPolicy::FIFO);
  EXPECT_EQ(result.config.priority, 50);
  EXPECT_TRUE(result.config.cpu_affinity.empty());
}

TEST(ParseThreadScheduling, AcceptsEveryPolicyName)
{
  EXPECT_EQ(utilities::parseThreadScheduling("fifo", "", "").config.policy,
            utilities::SchedulingPolicy::FIFO);
  EXPECT_EQ(utilities::parseThreadScheduling("rr", "", "").config.policy,
            utilities::SchedulingPolicy::ROUND_ROBIN);
  EXPECT_EQ(utilities::parseThreadScheduling("other", "", "").config.policy,
            utilities::SchedulingPolicy::OTHER);
  EXPECT_EQ(utilities::parseThreadScheduling("inherit", "", "").config.policy,
            utilities::SchedulingPolicy::INHERIT);
  EXPECT_FALSE(utilities::parseThreadScheduling("realtime", "", "").valid);
}

TEST(ParseThreadScheduling, ValidatesTheRealTimePriorityRange)
{
  EXPECT_EQ(utilities::parseThreadScheduling("fifo", "80", "").config.priority, 80);
  EXPECT_FALSE(utilities::parseThreadScheduling("fifo", "0", "").valid);
  EXPECT_FALSE(utilities::parseThreadScheduling("fifo", "100", "").valid);
  EXPECT_FALSE(utilities::parseThreadScheduling("fifo", "high", "").valid);
  EXPECT_EQ(utilities::parseThreadScheduling("other", "", "").config.priority, 0);
  EXPECT_EQ(utilities::parseThreadScheduling("inherit", "", "").config.priority, 0);
}

TEST(ParseThreadScheduling, ParsesTheCpuAffinityList)
{
  const utilities::ThreadSchedulingParseResult result =
      utilities::parseThreadScheduling("fifo", "50", "0,2,3");
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(result.config.cpu_affinity, (std::vector<unsigned int>{0, 2, 3}));
  EXPECT_FALSE(utilities::parseThreadScheduling("fifo", "50", "0,two").valid);
  EXPECT_FALSE(utilities::parseThreadScheduling("fifo", "50", ",").valid);
}

TEST(ToUpperCopy, UpperCasesPlainAscii)
{
  EXPECT_EQ(utilities::toUpperCopy("lreal"), "LREAL");
  EXPECT_EQ(utilities::toUpperCopy("Bool"), "BOOL");
}

namespace
{
  std::vector<uint8_t> makeFullRequest()
  {
    // Three items: headers of 3 bytes each, then data blocks of 2, 3 and 1 bytes.
    return {0xA1, 0xA2, 0xA3, 0xB1, 0xB2, 0xB3, 0xC1, 0xC2, 0xC3,
            0x10, 0x11, 0x20, 0x21, 0x22, 0x30};
  }

  std::vector<beckhoff_ads_hardware_interface::utilities::SumWriteItemSpan> makeSpans()
  {
    return {{0, 3, 9, 2}, {3, 3, 11, 3}, {6, 3, 14, 1}};
  }
} // namespace

TEST(CompactSumWriteRequest, AllItemsIncludedReproducesTheFullRequest)
{
  std::vector<uint8_t> compact;
  std::vector<size_t> indices;
  utilities::compactSumWriteRequest(makeFullRequest(), makeSpans(), {1, 1, 1}, compact, indices);
  EXPECT_EQ(compact, makeFullRequest());
  EXPECT_EQ(indices, (std::vector<size_t>{0, 1, 2}));
}

TEST(CompactSumWriteRequest, ExcludedItemsAreAbsentFromHeadersAndData)
{
  std::vector<uint8_t> compact;
  std::vector<size_t> indices;
  utilities::compactSumWriteRequest(makeFullRequest(), makeSpans(), {1, 0, 1}, compact, indices);
  const std::vector<uint8_t> expected = {0xA1, 0xA2, 0xA3, 0xC1, 0xC2, 0xC3, 0x10, 0x11, 0x30};
  EXPECT_EQ(compact, expected);
  EXPECT_EQ(indices, (std::vector<size_t>{0, 2}));
}

TEST(CompactSumWriteRequest, NoItemsIncludedYieldsAnEmptyRequest)
{
  std::vector<uint8_t> compact = {0xFF};
  std::vector<size_t> indices = {9};
  utilities::compactSumWriteRequest(makeFullRequest(), makeSpans(), {0, 0, 0}, compact, indices);
  EXPECT_TRUE(compact.empty());
  EXPECT_TRUE(indices.empty());
}

TEST(SetpointSequenceCounter, AdvancesByOnePerCall)
{
  utilities::SetpointSequenceCounter counter;
  EXPECT_EQ(counter.next(), 1u);
  EXPECT_EQ(counter.next(), 2u);
  EXPECT_EQ(counter.current(), 2u);
}

TEST(SetpointSequenceCounter, WrapsAtTheThirtyTwoBitBoundary)
{
  utilities::SetpointSequenceCounter counter(std::numeric_limits<uint32_t>::max() - 1u);
  EXPECT_EQ(counter.next(), std::numeric_limits<uint32_t>::max());
  EXPECT_EQ(counter.next(), 0u);
  EXPECT_EQ(counter.next(), 1u);
}

TEST(MonotonicSeconds, ConvertsAKnownInstantToSeconds)
{
  const std::chrono::steady_clock::time_point instant(std::chrono::milliseconds(1500));
  EXPECT_DOUBLE_EQ(utilities::monotonicSeconds(instant), 1.5);
}

TEST(MonotonicSeconds, NeverDecreasesAcrossConsecutiveClockReads)
{
  const double first = utilities::monotonicSeconds(std::chrono::steady_clock::now());
  const double second = utilities::monotonicSeconds(std::chrono::steady_clock::now());
  EXPECT_GE(second, first);
}

TEST(ToUpperCopy, ToleratesBytesOutsideTheAsciiRange)
{
  const std::string with_high_bytes = "lre\xC3\xA9l";
  const std::string result = utilities::toUpperCopy(with_high_bytes);
  EXPECT_EQ(result.substr(0, 3), "LRE");
  EXPECT_EQ(result.back(), 'L');
}
