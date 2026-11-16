// Copyright (c) 2025, b-robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

#include <gtest/gtest.h>

#include "beckhoff_ads_client/plc_type.hpp"

using beckhoff_ads_client::PlcType;
using beckhoff_ads_client::decode_plc_value;
using beckhoff_ads_client::encode_plc_value;
using beckhoff_ads_client::parse_plc_type;
using beckhoff_ads_client::plc_type_name;
using beckhoff_ads_client::plc_type_size;

namespace
{

constexpr std::array<PlcType, 10> ALL_TYPES{
  PlcType::Bool, PlcType::Byte, PlcType::Sint, PlcType::Usint, PlcType::Int,
  PlcType::Uint, PlcType::Dint, PlcType::Udint, PlcType::Real, PlcType::Lreal};

double round_trip(PlcType type, double value)
{
  std::array<std::uint8_t, 8> bytes{};
  encode_plc_value(type, value, bytes);
  return decode_plc_value(type, bytes);
}

}  // namespace

TEST(PlcType, ParsesEveryTypeNameInAnyCase)
{
  for (const PlcType type : ALL_TYPES) {
    EXPECT_EQ(parse_plc_type(plc_type_name(type)), type);
  }
  EXPECT_EQ(parse_plc_type("lreal"), PlcType::Lreal);
  EXPECT_EQ(parse_plc_type("Udint"), PlcType::Udint);
}

TEST(PlcType, RejectsUnsupportedTypeNames)
{
  EXPECT_FALSE(parse_plc_type("STRING").has_value());
  EXPECT_FALSE(parse_plc_type("WORD").has_value());
  EXPECT_FALSE(parse_plc_type("").has_value());
}

TEST(PlcType, ReportsTheTwinCatElementSizes)
{
  EXPECT_EQ(plc_type_size(PlcType::Bool), 1u);
  EXPECT_EQ(plc_type_size(PlcType::Byte), 1u);
  EXPECT_EQ(plc_type_size(PlcType::Sint), 1u);
  EXPECT_EQ(plc_type_size(PlcType::Usint), 1u);
  EXPECT_EQ(plc_type_size(PlcType::Int), 2u);
  EXPECT_EQ(plc_type_size(PlcType::Uint), 2u);
  EXPECT_EQ(plc_type_size(PlcType::Dint), 4u);
  EXPECT_EQ(plc_type_size(PlcType::Udint), 4u);
  EXPECT_EQ(plc_type_size(PlcType::Real), 4u);
  EXPECT_EQ(plc_type_size(PlcType::Lreal), 8u);
}

TEST(PlcType, RoundTripsRepresentableValues)
{
  EXPECT_DOUBLE_EQ(round_trip(PlcType::Lreal, 0.123456789012345), 0.123456789012345);
  EXPECT_FLOAT_EQ(static_cast<float>(round_trip(PlcType::Real, 1.25)), 1.25f);
  EXPECT_DOUBLE_EQ(round_trip(PlcType::Sint, -100.0), -100.0);
  EXPECT_DOUBLE_EQ(round_trip(PlcType::Usint, 200.0), 200.0);
  EXPECT_DOUBLE_EQ(round_trip(PlcType::Byte, 255.0), 255.0);
  EXPECT_DOUBLE_EQ(round_trip(PlcType::Int, -30000.0), -30000.0);
  EXPECT_DOUBLE_EQ(round_trip(PlcType::Uint, 60000.0), 60000.0);
  EXPECT_DOUBLE_EQ(round_trip(PlcType::Dint, -2000000000.0), -2000000000.0);
  EXPECT_DOUBLE_EQ(round_trip(PlcType::Udint, 4000000000.0), 4000000000.0);
}

TEST(PlcType, RoundsIntegersToTheNearestWholeNumber)
{
  EXPECT_DOUBLE_EQ(round_trip(PlcType::Dint, 2.6), 3.0);
  EXPECT_DOUBLE_EQ(round_trip(PlcType::Dint, -2.6), -3.0);
  EXPECT_DOUBLE_EQ(round_trip(PlcType::Uint, 7.4), 7.0);
}

TEST(PlcType, StoresBoolAsWhetherTheValueIsNonZero)
{
  EXPECT_DOUBLE_EQ(round_trip(PlcType::Bool, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(round_trip(PlcType::Bool, 0.3), 1.0);
  EXPECT_DOUBLE_EQ(round_trip(PlcType::Bool, -5.0), 1.0);
  const std::array<std::uint8_t, 1> stored_true{7};
  EXPECT_DOUBLE_EQ(decode_plc_value(PlcType::Bool, stored_true), 1.0);
}

TEST(PlcType, EncodesOnlyTheElementBytes)
{
  std::vector<std::uint8_t> bytes(8, 0xEE);
  encode_plc_value(PlcType::Int, 0x0102, bytes);
  EXPECT_EQ(bytes[2], 0xEE);
  std::int16_t stored = 0;
  std::memcpy(&stored, bytes.data(), sizeof(stored));
  EXPECT_EQ(stored, 0x0102);
}
