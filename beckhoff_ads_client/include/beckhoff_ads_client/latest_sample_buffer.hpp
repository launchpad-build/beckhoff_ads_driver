// Copyright (c) 2025, b-robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

/**
 * @file latest_sample_buffer.hpp
 * @brief Wait-free triple buffer handing the newest whole sample from one writer to one reader.
 */
#pragma once

#include <array>
#include <atomic>
#include <cstdint>

namespace beckhoff_ads_client
{
namespace utilities
{

/** @brief Wait-free latest-value hand-over between one writer thread and one reader thread. */
template<typename SampleType>
class LatestSampleBuffer
{
public:
  /** @brief The slot the writer fills next; writer thread only. */
  SampleType & writeSlot()
  {
    return slots_[write_index_];
  }

  /** @brief Publishes the write slot as the latest sample; true when it replaced one the reader never took. */
  bool publish()
  {
    const uint32_t previous = control_.exchange(write_index_ | FRESH_FLAG, std::memory_order_acq_rel);
    write_index_ = previous & INDEX_MASK;
    return (previous & FRESH_FLAG) != 0;
  }

  /** @brief Takes the latest published sample as the read slot; true when one arrived since the last take. */
  bool refreshReadSlot()
  {
    bool result = false;
    if ((control_.load(std::memory_order_acquire) & FRESH_FLAG) != 0) {
      read_index_ = control_.exchange(read_index_, std::memory_order_acq_rel) & INDEX_MASK;
      result = true;
    }
    return result;
  }

  /** @brief The most recently taken sample; reader thread only. */
  const SampleType & readSlot() const
  {
    return slots_[read_index_];
  }

  /** @brief Applies initialise to every slot and forgets any published sample; only while neither side runs. */
  template<typename InitFunction>
  void initialiseSlots(InitFunction initialise)
  {
    for (SampleType & slot : slots_) {
      initialise(slot);
    }
    control_.store(1, std::memory_order_release);
    write_index_ = 0;
    read_index_ = 2;
  }

private:
  static constexpr uint32_t FRESH_FLAG = 4;
  static constexpr uint32_t INDEX_MASK = 3;

  std::array<SampleType, 3> slots_{};
  std::atomic<uint32_t> control_{1};
  uint32_t write_index_{0};
  uint32_t read_index_{2};
};

}  // namespace utilities
}  // namespace beckhoff_ads_client
