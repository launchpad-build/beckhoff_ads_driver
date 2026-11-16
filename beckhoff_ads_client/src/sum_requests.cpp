// Copyright (c) 2025, b-robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include "sum_requests.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <utility>

namespace beckhoff_ads_client
{
namespace
{

void store_word(std::span<std::uint8_t> destination, std::size_t offset, std::uint32_t word)
{
  std::memcpy(destination.subspan(offset, sizeof(word)).data(), &word, sizeof(word));
}

std::uint32_t load_word(std::span<const std::uint8_t> source, std::size_t offset)
{
  std::uint32_t word = 0;
  std::memcpy(&word, source.subspan(offset, sizeof(word)).data(), sizeof(word));
  return word;
}

void store_item_header(std::span<std::uint8_t> destination, std::size_t offset, const SumItem & item)
{
  store_word(destination, offset, VALUE_BY_HANDLE_INDEX_GROUP);
  store_word(destination, offset + sizeof(std::uint32_t), item.handle);
  store_word(destination, offset + 2 * sizeof(std::uint32_t), static_cast<std::uint32_t>(item.byte_size()));
}

}  // namespace

std::size_t SumItem::byte_size() const
{
  return plc_type_size(type) * element_count;
}

SumReadRequest::SumReadRequest(std::vector<SumItem> items, std::vector<ReadSlot> slots)
: items_(std::move(items)), slots_(std::move(slots))
{
  const std::size_t error_block_size = items_.size() * SUM_ITEM_ERROR_SIZE;
  std::size_t data_block_size = 0;
  for (const SumItem & item : items_) {
    data_block_size += item.byte_size();
  }
  request_.assign(items_.size() * SUM_ITEM_HEADER_SIZE, 0);
  response_.assign(error_block_size + data_block_size, 0);

  std::vector<std::size_t> item_data_offsets(items_.size(), 0);
  std::size_t data_offset = error_block_size;
  for (std::size_t index = 0; index < items_.size(); ++index) {
    store_item_header(request_, index * SUM_ITEM_HEADER_SIZE, items_[index]);
    item_data_offsets[index] = data_offset;
    data_offset += items_[index].byte_size();
  }

  for (const ReadSlot & slot : slots_) {
    const SumItem & item = items_[slot.item_index];
    slot_error_offsets_.push_back(slot.item_index * SUM_ITEM_ERROR_SIZE);
    slot_data_offsets_.push_back(item_data_offsets[slot.item_index] + slot.element_index * plc_type_size(item.type));
  }
  slot_failing_since_.assign(slots_.size(), std::nullopt);
}

std::size_t SumReadRequest::item_count() const
{
  return items_.size();
}

const SumItem & SumReadRequest::item(std::size_t index) const
{
  return items_[index];
}

const ReadSlot & SumReadRequest::slot(std::size_t index) const
{
  return slots_[index];
}

std::span<const std::uint8_t> SumReadRequest::request() const
{
  return request_;
}

std::span<std::uint8_t> SumReadRequest::response()
{
  return response_;
}

ReadDecodeResult SumReadRequest::decode(
  std::span<double> values, std::chrono::steady_clock::time_point now,
  std::chrono::nanoseconds critical_grace)
{
  ReadDecodeResult result;
  const std::span<const std::uint8_t> response(response_);
  for (std::size_t index = 0; index < slots_.size(); ++index) {
    const ReadSlot & slot = slots_[index];
    const std::uint32_t error_code = load_word(response, slot_error_offsets_[index]);
    if (error_code != 0) {
      ++result.failed_slots;
      if (!result.first_failure) {
        result.first_failure = ReadItemFailure{index, error_code};
      }
      if (!slot_failing_since_[index]) {
        slot_failing_since_[index] = now;
      }
      if (slot.critical && now - *slot_failing_since_[index] >= critical_grace && !result.critical_fault_slot) {
        result.critical_fault_slot = index;
      }
    } else {
      slot_failing_since_[index].reset();
      if (slot.value_index < values.size()) {
        values[slot.value_index] =
          decode_plc_value(items_[slot.item_index].type, response.subspan(slot_data_offsets_[index]));
      }
    }
  }
  result.all_failed = !slots_.empty() && result.failed_slots == slots_.size();
  return result;
}

SumWritePacker::SumWritePacker(std::vector<SumItem> items, std::vector<WriteSlot> slots)
: items_(std::move(items)), slots_(std::move(slots))
{
  const std::size_t header_block_size = items_.size() * SUM_ITEM_HEADER_SIZE;
  std::size_t data_block_size = 0;
  for (const SumItem & item : items_) {
    data_block_size += item.byte_size();
  }
  request_.assign(header_block_size + data_block_size, 0);

  std::vector<std::size_t> item_data_offsets(items_.size(), 0);
  std::size_t data_offset = header_block_size;
  for (std::size_t index = 0; index < items_.size(); ++index) {
    store_item_header(request_, index * SUM_ITEM_HEADER_SIZE, items_[index]);
    item_data_offsets[index] = data_offset;
    item_spans_.push_back(
      utilities::SumWriteItemSpan{index * SUM_ITEM_HEADER_SIZE, SUM_ITEM_HEADER_SIZE, data_offset, items_[index].byte_size()});
    all_item_indices_.push_back(index);
    data_offset += items_[index].byte_size();
  }

  for (const WriteSlot & slot : slots_) {
    const SumItem & item = items_[slot.item_index];
    slot_data_offsets_.push_back(item_data_offsets[slot.item_index] + slot.element_index * plc_type_size(item.type));
    slot_provided_.push_back(slot.source == WriteSource::Element ? 0 : 1);
  }
  item_transmittable_.assign(items_.size(), 1);
  compact_request_.reserve(request_.size());
  compact_item_indices_.reserve(items_.size());
}

std::size_t SumWritePacker::item_count() const
{
  return items_.size();
}

const SumItem & SumWritePacker::item(std::size_t index) const
{
  return items_[index];
}

std::size_t SumWritePacker::request_size() const
{
  return request_.size();
}

void SumWritePacker::seed(std::size_t value_index, double value)
{
  if (std::isfinite(value)) {
    for (std::size_t index = 0; index < slots_.size(); ++index) {
      const WriteSlot & slot = slots_[index];
      if (slot.source == WriteSource::Element && slot.value_index == value_index) {
        encode_plc_value(
          items_[slot.item_index].type, value, std::span<std::uint8_t>(request_).subspan(slot_data_offsets_[index]));
        slot_provided_[index] = 1;
      }
    }
  }
}

std::optional<PackedWrite> SumWritePacker::pack(
  std::span<const double> values, const SyntheticWriteValues & synthetic) noexcept
{
  std::fill(item_transmittable_.begin(), item_transmittable_.end(), 1);
  const std::span<std::uint8_t> request(request_);
  for (std::size_t index = 0; index < slots_.size(); ++index) {
    const WriteSlot & slot = slots_[index];
    const PlcType type = items_[slot.item_index].type;
    const std::span<std::uint8_t> destination = request.subspan(slot_data_offsets_[index]);
    switch (slot.source) {
      case WriteSource::Heartbeat:
        encode_plc_value(type, static_cast<double>(synthetic.heartbeat), destination);
        break;
      case WriteSource::SetpointSequence:
        encode_plc_value(type, static_cast<double>(synthetic.setpoint_sequence), destination);
        break;
      case WriteSource::SetpointTimestamp:
        encode_plc_value(type, synthetic.setpoint_timestamp, destination);
        break;
      case WriteSource::Element: {
        const double value = slot.value_index < values.size() ?
          values[slot.value_index] :
          std::numeric_limits<double>::quiet_NaN();
        if (std::isnan(value)) {
          if (slot_provided_[index] == 0) {
            item_transmittable_[slot.item_index] = 0;
          }
        } else {
          encode_plc_value(type, value, destination);
          slot_provided_[index] = 1;
        }
        break;
      }
    }
  }

  std::optional<PackedWrite> result;
  const bool every_item_transmittable = std::all_of(
    item_transmittable_.begin(), item_transmittable_.end(), [](std::uint8_t transmittable) {return transmittable != 0;});
  if (every_item_transmittable) {
    if (!items_.empty()) {
      result = PackedWrite{request_, all_item_indices_};
    }
  } else {
    utilities::compactSumWriteRequest(
      request_, item_spans_, item_transmittable_, compact_request_, compact_item_indices_);
    if (!compact_item_indices_.empty()) {
      result = PackedWrite{compact_request_, compact_item_indices_};
    }
  }
  return result;
}

}  // namespace beckhoff_ads_client
