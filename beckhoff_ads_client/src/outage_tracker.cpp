// Copyright (c) 2025, b-robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include "outage_tracker.hpp"

namespace beckhoff_ads_client
{

void OutageTracker::reset(std::chrono::nanoseconds grace, bool latches)
{
  grace_ = grace;
  latches_ = latches;
  consecutive_failures_ = 0;
  stable_since_.reset();
  faulted_.store(false, std::memory_order_release);
  interrupted_.store(false, std::memory_order_release);
}

void OutageTracker::record_failure(std::chrono::steady_clock::time_point now)
{
  if (consecutive_failures_ == 0) {
    outage_start_ = now;
  }
  ++consecutive_failures_;
  interrupted_.store(true, std::memory_order_release);
  stable_since_.reset();
  if (now - outage_start_ >= grace_) {
    faulted_.store(true, std::memory_order_release);
  }
}

std::optional<OutageRecovery> OutageTracker::record_success(
  std::chrono::steady_clock::time_point now, bool keep_fault)
{
  if (!keep_fault && !(latches_ && faulted_.load(std::memory_order_acquire))) {
    faulted_.store(false, std::memory_order_release);
  }
  std::optional<OutageRecovery> result;
  if (consecutive_failures_ > 0) {
    if (!stable_since_) {
      stable_since_ = now;
    } else if (now - *stable_since_ >= RECOVERY_STABLE_PERIOD) {
      result = OutageRecovery{
        consecutive_failures_,
        std::chrono::duration_cast<std::chrono::milliseconds>(*stable_since_ - outage_start_)};
      consecutive_failures_ = 0;
      stable_since_.reset();
      interrupted_.store(false, std::memory_order_release);
    }
  }
  return result;
}

void OutageTracker::declare_fault()
{
  faulted_.store(true, std::memory_order_release);
}

bool OutageTracker::faulted() const noexcept
{
  return faulted_.load(std::memory_order_acquire);
}

bool OutageTracker::interrupted() const noexcept
{
  return interrupted_.load(std::memory_order_acquire);
}

std::size_t OutageTracker::consecutive_failures() const
{
  return consecutive_failures_;
}

}  // namespace beckhoff_ads_client
