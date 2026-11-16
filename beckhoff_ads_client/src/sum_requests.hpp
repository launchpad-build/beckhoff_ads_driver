// Copyright (c) 2025, b-robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "beckhoff_ads_client/ads_client_utilities.hpp"
#include "beckhoff_ads_client/plc_type.hpp"

namespace beckhoff_ads_client
{

inline constexpr std::uint32_t VALUE_BY_HANDLE_INDEX_GROUP = 0xF005;
inline constexpr std::size_t SUM_ITEM_HEADER_SIZE = 3 * sizeof(std::uint32_t);
inline constexpr std::size_t SUM_ITEM_ERROR_SIZE = sizeof(std::uint32_t);

struct SumItem
{
  std::string symbol;
  PlcType type{PlcType::Lreal};
  std::size_t element_count{1};
  std::uint32_t handle{0};

  std::size_t byte_size() const;
};

struct ReadSlot
{
  std::size_t item_index{0};
  std::size_t element_index{0};
  std::size_t value_index{0};
  bool critical{false};
  std::string label;
};

struct ReadItemFailure
{
  std::size_t slot_index{0};
  std::uint32_t error_code{0};
};

struct ReadDecodeResult
{
  std::size_t failed_slots{0};
  bool all_failed{false};
  std::optional<ReadItemFailure> first_failure;
  std::optional<std::size_t> critical_fault_slot;
};

class SumReadRequest
{
public:
  SumReadRequest() = default;
  SumReadRequest(std::vector<SumItem> items, std::vector<ReadSlot> slots);

  std::size_t item_count() const;
  const SumItem & item(std::size_t index) const;
  const ReadSlot & slot(std::size_t index) const;
  std::span<const std::uint8_t> request() const;
  std::span<std::uint8_t> response();

  ReadDecodeResult decode(
    std::span<double> values, std::chrono::steady_clock::time_point now,
    std::chrono::nanoseconds critical_grace);

private:
  std::vector<SumItem> items_;
  std::vector<ReadSlot> slots_;
  std::vector<std::uint8_t> request_;
  std::vector<std::uint8_t> response_;
  std::vector<std::size_t> slot_error_offsets_;
  std::vector<std::size_t> slot_data_offsets_;
  std::vector<std::optional<std::chrono::steady_clock::time_point>> slot_failing_since_;
};

enum class WriteSource
{
  Element,
  Heartbeat,
  SetpointSequence,
  SetpointTimestamp,
};

struct WriteSlot
{
  std::size_t item_index{0};
  std::size_t element_index{0};
  WriteSource source{WriteSource::Element};
  std::size_t value_index{0};
};

struct SyntheticWriteValues
{
  std::uint32_t heartbeat{0};
  std::uint32_t setpoint_sequence{0};
  double setpoint_timestamp{0.0};
};

struct PackedWrite
{
  std::span<const std::uint8_t> request;
  std::span<const std::size_t> item_indices;
};

class SumWritePacker
{
public:
  SumWritePacker() = default;
  SumWritePacker(std::vector<SumItem> items, std::vector<WriteSlot> slots);

  std::size_t item_count() const;
  const SumItem & item(std::size_t index) const;
  std::size_t request_size() const;

  void seed(std::size_t value_index, double value);

  std::optional<PackedWrite> pack(
    std::span<const double> values, const SyntheticWriteValues & synthetic) noexcept;

private:
  std::vector<SumItem> items_;
  std::vector<WriteSlot> slots_;
  std::vector<std::size_t> slot_data_offsets_;
  std::vector<std::uint8_t> slot_provided_;
  std::vector<std::uint8_t> request_;
  std::vector<utilities::SumWriteItemSpan> item_spans_;
  std::vector<std::size_t> all_item_indices_;
  std::vector<std::uint8_t> item_transmittable_;
  std::vector<std::uint8_t> compact_request_;
  std::vector<std::size_t> compact_item_indices_;
};

}  // namespace beckhoff_ads_client
