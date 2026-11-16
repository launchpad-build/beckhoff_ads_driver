// Copyright (c) 2025, b-robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "beckhoff_ads_client/latest_sample_buffer.hpp"
#include "sum_requests.hpp"

namespace beckhoff_ads_client
{

struct PendingWrite
{
  std::vector<std::uint8_t> request;
  std::vector<std::size_t> item_indices;
  std::chrono::steady_clock::time_point handoff{};
};

class WriteHandoff
{
public:
  void reset(std::size_t request_capacity, std::size_t item_capacity);

  bool offer(const PackedWrite & packed, std::chrono::steady_clock::time_point handoff) noexcept;

  bool wait_for_write(const std::atomic<bool> & stop);

  const PendingWrite & taken() const;

  void wake();

private:
  utilities::LatestSampleBuffer<PendingWrite> pending_;
  std::atomic<std::uint64_t> generation_{0};
};

}  // namespace beckhoff_ads_client
