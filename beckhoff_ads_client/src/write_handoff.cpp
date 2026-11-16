// Copyright (c) 2025, b-robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include "write_handoff.hpp"

namespace beckhoff_ads_client
{

void WriteHandoff::reset(std::size_t request_capacity, std::size_t item_capacity)
{
  pending_.initialiseSlots(
    [request_capacity, item_capacity](PendingWrite & slot) {
      slot.request.clear();
      slot.request.reserve(request_capacity);
      slot.item_indices.clear();
      slot.item_indices.reserve(item_capacity);
      slot.handoff = std::chrono::steady_clock::time_point{};
    });
}

bool WriteHandoff::offer(const PackedWrite & packed, std::chrono::steady_clock::time_point handoff) noexcept
{
  PendingWrite & slot = pending_.writeSlot();
  slot.request.assign(packed.request.begin(), packed.request.end());
  slot.item_indices.assign(packed.item_indices.begin(), packed.item_indices.end());
  slot.handoff = handoff;
  const bool replaced_unsent_write = pending_.publish();
  generation_.fetch_add(1);
  generation_.notify_one();
  return replaced_unsent_write;
}

bool WriteHandoff::wait_for_write(const std::atomic<bool> & stop)
{
  bool result = false;
  bool waiting = true;
  while (waiting) {
    const std::uint64_t observed = generation_.load();
    if (stop.load()) {
      waiting = false;
    } else if (pending_.refreshReadSlot()) {
      result = true;
      waiting = false;
    } else {
      generation_.wait(observed);
    }
  }
  return result;
}

const PendingWrite & WriteHandoff::taken() const
{
  return pending_.readSlot();
}

void WriteHandoff::wake()
{
  generation_.fetch_add(1);
  generation_.notify_all();
}

}  // namespace beckhoff_ads_client
