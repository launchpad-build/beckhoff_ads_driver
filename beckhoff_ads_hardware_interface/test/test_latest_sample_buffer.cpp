// Copyright (c) 2025, b-robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include <atomic>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "beckhoff_ads_hardware_interface/ads_interface_utilities.hpp"

namespace utilities = beckhoff_ads_hardware_interface::utilities;

namespace
{
  struct PairedSample
  {
    uint64_t first = 0;
    uint64_t second = 0;
  };
} // namespace

TEST(LatestSampleBuffer, NothingToTakeBeforeTheFirstPublish)
{
  utilities::LatestSampleBuffer<int> buffer;
  EXPECT_FALSE(buffer.refreshReadSlot());
  EXPECT_EQ(buffer.readSlot(), 0);
}

TEST(LatestSampleBuffer, ReaderTakesThePublishedSample)
{
  utilities::LatestSampleBuffer<int> buffer;
  buffer.writeSlot() = 42;
  buffer.publish();
  EXPECT_TRUE(buffer.refreshReadSlot());
  EXPECT_EQ(buffer.readSlot(), 42);
  EXPECT_FALSE(buffer.refreshReadSlot());
  EXPECT_EQ(buffer.readSlot(), 42);
}

TEST(LatestSampleBuffer, ReaderTakesOnlyTheNewestOfSeveralPublishes)
{
  utilities::LatestSampleBuffer<int> buffer;
  buffer.writeSlot() = 1;
  buffer.publish();
  buffer.writeSlot() = 2;
  buffer.publish();
  buffer.writeSlot() = 3;
  buffer.publish();
  EXPECT_TRUE(buffer.refreshReadSlot());
  EXPECT_EQ(buffer.readSlot(), 3);
}

TEST(LatestSampleBuffer, InitialiseSlotsAppliesToAllThreeSlots)
{
  utilities::LatestSampleBuffer<std::vector<double>> buffer;
  buffer.initialiseSlots([](std::vector<double> &slot)
                         { slot.assign(4, -1.0); });
  EXPECT_EQ(buffer.writeSlot().size(), 4u);
  EXPECT_EQ(buffer.readSlot().size(), 4u);
  buffer.publish();
  EXPECT_TRUE(buffer.refreshReadSlot());
  EXPECT_EQ(buffer.readSlot().size(), 4u);
}

// The reason the buffer exists: the reader must never observe a sample mixing
// fields from two different publishes, and must never see time run backwards.
TEST(LatestSampleBuffer, ConcurrentReaderSeesWholeMonotonicSamples)
{
  utilities::LatestSampleBuffer<PairedSample> buffer;
  std::atomic<bool> stop{false};

  std::thread writer(
      [&buffer, &stop]
      {
        uint64_t counter = 0;
        while (!stop.load(std::memory_order_relaxed))
        {
          ++counter;
          PairedSample &slot = buffer.writeSlot();
          slot.first = counter;
          slot.second = counter;
          buffer.publish();
        }
      });

  uint64_t last_seen = 0;
  for (int i = 0; i < 200000; ++i)
  {
    buffer.refreshReadSlot();
    const PairedSample &sample = buffer.readSlot();
    ASSERT_EQ(sample.first, sample.second);
    ASSERT_GE(sample.first, last_seen);
    last_seen = sample.first;
  }

  stop.store(true, std::memory_order_relaxed);
  writer.join();
}
