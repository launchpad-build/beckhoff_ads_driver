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
#include <optional>

namespace beckhoff_ads_client
{

struct OutageRecovery
{
  std::size_t failed_round_trips{0};
  std::chrono::milliseconds duration{0};
};

class OutageTracker
{
public:
  static constexpr std::chrono::seconds RECOVERY_STABLE_PERIOD{1};

  void reset(std::chrono::nanoseconds grace, bool latches);

  void record_failure(std::chrono::steady_clock::time_point now);

  std::optional<OutageRecovery> record_success(std::chrono::steady_clock::time_point now, bool keep_fault);

  void declare_fault();

  bool faulted() const noexcept;

  bool interrupted() const noexcept;

  std::size_t consecutive_failures() const;

private:
  std::chrono::nanoseconds grace_{std::chrono::milliseconds(100)};
  bool latches_{true};
  std::size_t consecutive_failures_{0};
  std::chrono::steady_clock::time_point outage_start_{};
  std::optional<std::chrono::steady_clock::time_point> stable_since_;
  std::atomic<bool> faulted_{false};
  std::atomic<bool> interrupted_{false};
};

}  // namespace beckhoff_ads_client
