// Copyright (c) 2025, b-robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "sum_requests.hpp"

using beckhoff_ads_client::PackedWrite;
using beckhoff_ads_client::PlcType;
using beckhoff_ads_client::ReadSlot;
using beckhoff_ads_client::SumItem;
using beckhoff_ads_client::SumReadRequest;
using beckhoff_ads_client::SumWritePacker;
using beckhoff_ads_client::SyntheticWriteValues;
using beckhoff_ads_client::WriteSlot;
using beckhoff_ads_client::WriteSource;

namespace
{

constexpr double NOT_PROVIDED = std::numeric_limits<double>::quiet_NaN();
const std::chrono::steady_clock::time_point START{std::chrono::seconds(100)};
constexpr std::chrono::milliseconds GRACE{100};

std::uint32_t word_at(std::span<const std::uint8_t> bytes, std::size_t offset)
{
  std::uint32_t word = 0;
  std::memcpy(&word, bytes.subspan(offset, sizeof(word)).data(), sizeof(word));
  return word;
}

void put_word(std::span<std::uint8_t> bytes, std::size_t offset, std::uint32_t word)
{
  std::memcpy(bytes.subspan(offset, sizeof(word)).data(), &word, sizeof(word));
}

template<typename Stored>
Stored value_at(std::span<const std::uint8_t> bytes, std::size_t offset)
{
  Stored stored{};
  std::memcpy(&stored, bytes.subspan(offset, sizeof(stored)).data(), sizeof(stored));
  return stored;
}

template<typename Stored>
void put_value(std::span<std::uint8_t> bytes, std::size_t offset, Stored stored)
{
  std::memcpy(bytes.subspan(offset, sizeof(stored)).data(), &stored, sizeof(stored));
}

SumReadRequest make_read_request()
{
  std::vector<SumItem> items{
    SumItem{"MAIN.positions", PlcType::Lreal, 3, 0x11},
    SumItem{"MAIN.enabled", PlcType::Bool, 1, 0x22},
  };
  std::vector<ReadSlot> slots{
    ReadSlot{0, 2, 0, true, "x/position"},
    ReadSlot{0, 0, 1, false, "y/position"},
    ReadSlot{1, 0, 2, false, "enabled"},
  };
  return SumReadRequest(std::move(items), std::move(slots));
}

SumWritePacker make_write_packer()
{
  std::vector<SumItem> items{
    SumItem{"MAIN.targets", PlcType::Lreal, 2, 0x31},
    SumItem{"MAIN.mode", PlcType::Int, 1, 0x32},
  };
  std::vector<WriteSlot> slots{
    WriteSlot{0, 0, WriteSource::Element, 0},
    WriteSlot{0, 1, WriteSource::Element, 1},
    WriteSlot{1, 0, WriteSource::Element, 2},
  };
  return SumWritePacker(std::move(items), std::move(slots));
}

constexpr std::size_t HEADERS = 2 * 12;
constexpr std::size_t TARGETS_DATA = HEADERS;
constexpr std::size_t MODE_DATA = HEADERS + 16;

}  // namespace

TEST(SumReadRequest, LaysOutOneHeaderPerSymbolAndAnErrorThenDataResponse)
{
  SumReadRequest request = make_read_request();
  ASSERT_EQ(request.item_count(), 2u);
  const std::span<const std::uint8_t> headers = request.request();
  ASSERT_EQ(headers.size(), 24u);
  EXPECT_EQ(word_at(headers, 0), 0xF005u);
  EXPECT_EQ(word_at(headers, 4), 0x11u);
  EXPECT_EQ(word_at(headers, 8), 24u);
  EXPECT_EQ(word_at(headers, 12), 0xF005u);
  EXPECT_EQ(word_at(headers, 16), 0x22u);
  EXPECT_EQ(word_at(headers, 20), 1u);
  EXPECT_EQ(request.response().size(), 2 * 4 + 24 + 1u);
}

TEST(SumReadRequest, DecodesEachSlotFromItsElement)
{
  SumReadRequest request = make_read_request();
  const std::span<std::uint8_t> response = request.response();
  put_value<double>(response, 8, 1.5);
  put_value<double>(response, 8 + 16, 3.5);
  response[8 + 24] = 1;
  std::vector<double> values(3, NOT_PROVIDED);

  const auto result = request.decode(values, START, GRACE);

  EXPECT_EQ(result.failed_slots, 0u);
  EXPECT_FALSE(result.all_failed);
  EXPECT_DOUBLE_EQ(values[0], 3.5);
  EXPECT_DOUBLE_EQ(values[1], 1.5);
  EXPECT_DOUBLE_EQ(values[2], 1.0);
}

TEST(SumReadRequest, AFailedItemHoldsItsLastValueAndIsReported)
{
  SumReadRequest request = make_read_request();
  const std::span<std::uint8_t> response = request.response();
  std::vector<double> values{7.0, 8.0, 0.0};
  put_word(response, 0, 0x710);
  response[8 + 24] = 1;

  const auto result = request.decode(values, START, GRACE);

  EXPECT_EQ(result.failed_slots, 2u);
  EXPECT_FALSE(result.all_failed);
  ASSERT_TRUE(result.first_failure.has_value());
  EXPECT_EQ(result.first_failure->slot_index, 0u);
  EXPECT_EQ(result.first_failure->error_code, 0x710u);
  EXPECT_DOUBLE_EQ(values[0], 7.0);
  EXPECT_DOUBLE_EQ(values[1], 8.0);
  EXPECT_DOUBLE_EQ(values[2], 1.0);
  EXPECT_FALSE(result.critical_fault_slot.has_value());
}

TEST(SumReadRequest, EveryItemFailingCountsAsAllFailed)
{
  SumReadRequest request = make_read_request();
  put_word(request.response(), 0, 1);
  put_word(request.response(), 4, 1);
  std::vector<double> values(3, 0.0);
  EXPECT_TRUE(request.decode(values, START, GRACE).all_failed);
}

TEST(SumReadRequest, ACriticalSlotFaultsOnlyOnceItKeepsFailingBeyondTheGrace)
{
  SumReadRequest request = make_read_request();
  put_word(request.response(), 0, 1);
  std::vector<double> values(3, 0.0);

  EXPECT_FALSE(request.decode(values, START, GRACE).critical_fault_slot.has_value());
  EXPECT_FALSE(request.decode(values, START + GRACE / 2, GRACE).critical_fault_slot.has_value());
  const auto beyond = request.decode(values, START + GRACE, GRACE);
  ASSERT_TRUE(beyond.critical_fault_slot.has_value());
  EXPECT_EQ(*beyond.critical_fault_slot, 0u);
}

TEST(SumReadRequest, ASuccessfulReadRestartsTheCriticalFailureStreak)
{
  SumReadRequest request = make_read_request();
  std::vector<double> values(3, 0.0);
  put_word(request.response(), 0, 1);
  request.decode(values, START, GRACE);
  put_word(request.response(), 0, 0);
  request.decode(values, START + GRACE / 2, GRACE);
  put_word(request.response(), 0, 1);
  EXPECT_FALSE(request.decode(values, START + GRACE, GRACE).critical_fault_slot.has_value());
}

TEST(SumWritePacker, LaysOutHeadersThenData)
{
  SumWritePacker packer = make_write_packer();
  EXPECT_EQ(packer.item_count(), 2u);
  EXPECT_EQ(packer.request_size(), HEADERS + 16 + 2);
  const std::vector<double> values{1.0, 2.0, 3.0};
  const auto packed = packer.pack(values, SyntheticWriteValues{});
  ASSERT_TRUE(packed.has_value());
  EXPECT_EQ(word_at(packed->request, 0), 0xF005u);
  EXPECT_EQ(word_at(packed->request, 4), 0x31u);
  EXPECT_EQ(word_at(packed->request, 8), 16u);
  EXPECT_EQ(word_at(packed->request, 16), 0x32u);
  EXPECT_EQ(word_at(packed->request, 20), 2u);
  EXPECT_DOUBLE_EQ(value_at<double>(packed->request, TARGETS_DATA), 1.0);
  EXPECT_DOUBLE_EQ(value_at<double>(packed->request, TARGETS_DATA + 8), 2.0);
  EXPECT_EQ(value_at<std::int16_t>(packed->request, MODE_DATA), 3);
  EXPECT_EQ(std::vector<std::size_t>(packed->item_indices.begin(), packed->item_indices.end()),
    (std::vector<std::size_t>{0, 1}));
}

TEST(SumWritePacker, NothingIsSentBeforeAnyValueIsProvided)
{
  SumWritePacker packer = make_write_packer();
  const std::vector<double> values(3, NOT_PROVIDED);
  EXPECT_FALSE(packer.pack(values, SyntheticWriteValues{}).has_value());
}

TEST(SumWritePacker, AnItemIsLeftOutUntilEveryElementOfItHasBeenProvided)
{
  SumWritePacker packer = make_write_packer();
  const std::vector<double> only_first_target{4.0, NOT_PROVIDED, NOT_PROVIDED};
  EXPECT_FALSE(packer.pack(only_first_target, SyntheticWriteValues{}).has_value());

  const std::vector<double> only_mode{NOT_PROVIDED, NOT_PROVIDED, 9.0};
  const auto packed = packer.pack(only_mode, SyntheticWriteValues{});
  ASSERT_TRUE(packed.has_value());
  EXPECT_EQ(std::vector<std::size_t>(packed->item_indices.begin(), packed->item_indices.end()),
    (std::vector<std::size_t>{1}));
  ASSERT_EQ(packed->request.size(), 12u + 2u);
  EXPECT_EQ(word_at(packed->request, 4), 0x32u);
  EXPECT_EQ(value_at<std::int16_t>(packed->request, 12), 9);

  const std::vector<double> second_target{NOT_PROVIDED, 5.0, NOT_PROVIDED};
  const auto complete = packer.pack(second_target, SyntheticWriteValues{});
  ASSERT_TRUE(complete.has_value());
  EXPECT_EQ(complete->item_indices.size(), 2u);
  EXPECT_DOUBLE_EQ(value_at<double>(complete->request, TARGETS_DATA), 4.0);
  EXPECT_DOUBLE_EQ(value_at<double>(complete->request, TARGETS_DATA + 8), 5.0);
  EXPECT_EQ(value_at<std::int16_t>(complete->request, MODE_DATA), 9);
}

TEST(SumWritePacker, ANotProvidedValueHoldsTheLastProvidedOne)
{
  SumWritePacker packer = make_write_packer();
  const std::vector<double> first{1.0, 2.0, 3.0};
  ASSERT_TRUE(packer.pack(first, SyntheticWriteValues{}).has_value());
  const std::vector<double> partial{NOT_PROVIDED, 6.0, NOT_PROVIDED};
  const auto packed = packer.pack(partial, SyntheticWriteValues{});
  ASSERT_TRUE(packed.has_value());
  EXPECT_DOUBLE_EQ(value_at<double>(packed->request, TARGETS_DATA), 1.0);
  EXPECT_DOUBLE_EQ(value_at<double>(packed->request, TARGETS_DATA + 8), 6.0);
  EXPECT_EQ(value_at<std::int16_t>(packed->request, MODE_DATA), 3);
}

TEST(SumWritePacker, ASeededInitialValueCountsAsProvided)
{
  SumWritePacker packer = make_write_packer();
  packer.seed(2, 4.0);
  packer.seed(0, NOT_PROVIDED);
  const std::vector<double> values(3, NOT_PROVIDED);
  const auto packed = packer.pack(values, SyntheticWriteValues{});
  ASSERT_TRUE(packed.has_value());
  EXPECT_EQ(std::vector<std::size_t>(packed->item_indices.begin(), packed->item_indices.end()),
    (std::vector<std::size_t>{1}));
  EXPECT_EQ(value_at<std::int16_t>(packed->request, 12), 4);
}

TEST(SumWritePacker, UnmappedArrayElementsAreSentAsZeroOnceTheItemIsIncluded)
{
  std::vector<SumItem> items{SumItem{"MAIN.targets", PlcType::Dint, 3, 0x41}};
  std::vector<WriteSlot> slots{WriteSlot{0, 1, WriteSource::Element, 0}};
  SumWritePacker packer(std::move(items), std::move(slots));
  const std::vector<double> values{12.0};
  const auto packed = packer.pack(values, SyntheticWriteValues{});
  ASSERT_TRUE(packed.has_value());
  EXPECT_EQ(value_at<std::int32_t>(packed->request, 12), 0);
  EXPECT_EQ(value_at<std::int32_t>(packed->request, 16), 12);
  EXPECT_EQ(value_at<std::int32_t>(packed->request, 20), 0);
}

TEST(SumWritePacker, AnnotationsAreAlwaysSentWithTheirSyntheticValues)
{
  std::vector<SumItem> items{
    SumItem{"MAIN.target", PlcType::Lreal, 1, 0x51},
    SumItem{"MAIN.heartbeat", PlcType::Udint, 1, 0x52},
    SumItem{"MAIN.sequence", PlcType::Udint, 1, 0x53},
    SumItem{"MAIN.stamp", PlcType::Lreal, 1, 0x54},
  };
  std::vector<WriteSlot> slots{
    WriteSlot{0, 0, WriteSource::Element, 0},
    WriteSlot{1, 0, WriteSource::Heartbeat, 0},
    WriteSlot{2, 0, WriteSource::SetpointSequence, 0},
    WriteSlot{3, 0, WriteSource::SetpointTimestamp, 0},
  };
  SumWritePacker packer(std::move(items), std::move(slots));
  const std::vector<double> values{NOT_PROVIDED};
  const auto packed = packer.pack(values, SyntheticWriteValues{7, 4294967295u, 12.5});
  ASSERT_TRUE(packed.has_value());
  ASSERT_EQ(packed->item_indices.size(), 3u);
  EXPECT_EQ(packed->item_indices[0], 1u);
  const std::size_t data = 3 * 12;
  EXPECT_EQ(value_at<std::uint32_t>(packed->request, data), 7u);
  EXPECT_EQ(value_at<std::uint32_t>(packed->request, data + 4), 4294967295u);
  EXPECT_DOUBLE_EQ(value_at<double>(packed->request, data + 8), 12.5);
}

TEST(SumWritePacker, MissingTrailingValuesCountAsNotProvided)
{
  SumWritePacker packer = make_write_packer();
  const std::vector<double> values{1.0, 2.0};
  const auto packed = packer.pack(values, SyntheticWriteValues{});
  ASSERT_TRUE(packed.has_value());
  EXPECT_EQ(std::vector<std::size_t>(packed->item_indices.begin(), packed->item_indices.end()),
    (std::vector<std::size_t>{0}));
}
