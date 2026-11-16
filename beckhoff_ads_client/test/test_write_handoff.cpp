// Copyright (c) 2025, b-robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "write_handoff.hpp"

using beckhoff_ads_client::PackedWrite;
using beckhoff_ads_client::WriteHandoff;

namespace
{

const std::chrono::steady_clock::time_point HANDOFF{std::chrono::seconds(5)};

struct Request
{
  std::vector<std::uint8_t> bytes;
  std::vector<std::size_t> items;

  PackedWrite packed() const
  {
    return PackedWrite{bytes, items};
  }
};

}  // namespace

TEST(WriteHandoff, TheWriterTakesTheOfferedRequest)
{
  WriteHandoff handoff;
  handoff.reset(8, 2);
  const Request request{{1, 2, 3}, {0, 1}};
  EXPECT_FALSE(handoff.offer(request.packed(), HANDOFF));
  const std::atomic<bool> stop{false};
  ASSERT_TRUE(handoff.wait_for_write(stop));
  EXPECT_EQ(handoff.taken().request, request.bytes);
  EXPECT_EQ(handoff.taken().item_indices, request.items);
  EXPECT_EQ(handoff.taken().handoff, HANDOFF);
}

TEST(WriteHandoff, ANewerOfferCoalescesAnUnsentOne)
{
  WriteHandoff handoff;
  handoff.reset(8, 2);
  const Request older{{1}, {0}};
  const Request newer{{2, 2}, {0, 1}};
  EXPECT_FALSE(handoff.offer(older.packed(), HANDOFF));
  EXPECT_TRUE(handoff.offer(newer.packed(), HANDOFF));
  const std::atomic<bool> stop{false};
  ASSERT_TRUE(handoff.wait_for_write(stop));
  EXPECT_EQ(handoff.taken().request, newer.bytes);
  EXPECT_FALSE(handoff.offer(older.packed(), HANDOFF));
}

TEST(WriteHandoff, OfferingWithinTheReservedCapacityDoesNotReallocate)
{
  WriteHandoff handoff;
  handoff.reset(16, 4);
  const Request request{std::vector<std::uint8_t>(16, 7), {0, 1, 2, 3}};
  const std::atomic<bool> stop{false};
  for (int round = 0; round < 6; ++round) {
    handoff.offer(request.packed(), HANDOFF);
    ASSERT_TRUE(handoff.wait_for_write(stop));
    EXPECT_GE(handoff.taken().request.capacity(), 16u);
    EXPECT_EQ(handoff.taken().request.size(), 16u);
  }
}

TEST(WriteHandoff, AStopRequestEndsTheWaitEvenWithAWritePending)
{
  WriteHandoff handoff;
  handoff.reset(8, 2);
  const Request request{{1}, {0}};
  handoff.offer(request.packed(), HANDOFF);
  const std::atomic<bool> stop{true};
  EXPECT_FALSE(handoff.wait_for_write(stop));
}

TEST(WriteHandoff, AWaitingWriterWakesForAnOfferAndForAStop)
{
  WriteHandoff handoff;
  handoff.reset(8, 2);
  std::atomic<bool> stop{false};
  std::atomic<int> writes_taken{0};
  std::thread writer(
    [&] {
      while (handoff.wait_for_write(stop)) {
        writes_taken.fetch_add(1);
      }
    });

  const Request request{{9}, {0}};
  handoff.offer(request.packed(), HANDOFF);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (writes_taken.load() == 0 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::yield();
  }
  EXPECT_EQ(writes_taken.load(), 1);

  stop.store(true);
  handoff.wake();
  writer.join();
}
