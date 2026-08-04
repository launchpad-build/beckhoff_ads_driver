// Copyright (c) 2025, b-robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

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

TEST(ToUpperCopy, UpperCasesPlainAscii)
{
  EXPECT_EQ(utilities::toUpperCopy("lreal"), "LREAL");
  EXPECT_EQ(utilities::toUpperCopy("Bool"), "BOOL");
}

TEST(ToUpperCopy, ToleratesBytesOutsideTheAsciiRange)
{
  const std::string with_high_bytes = "lre\xC3\xA9l";
  const std::string result = utilities::toUpperCopy(with_high_bytes);
  EXPECT_EQ(result.substr(0, 3), "LRE");
  EXPECT_EQ(result.back(), 'L');
}
