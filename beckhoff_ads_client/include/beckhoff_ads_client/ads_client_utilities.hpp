// Copyright (c) 2025, b-robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

/**
 * @file ads_client_utilities.hpp
 * @brief Parsing, sum-request compaction, setpoint sequencing and I/O thread scheduling helpers.
 */
#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace beckhoff_ads_client
{
namespace utilities
{

/** @brief Outcome of parsing an AMS port string. */
struct AmsPortParseResult
{
  bool valid{false};
  uint16_t port{0};
  std::string error;
};

/** @brief Parses an AMS port string, accepting only whole numbers from 1 to 65535. */
AmsPortParseResult parseAmsPort(const std::string & text);

/** @brief Parses an AMS net id such as "192.168.1.1.1.1" into its six bytes, or nothing if malformed. */
std::optional<std::array<uint8_t, 6>> parseAmsNetId(const std::string & text);

/** @brief Returns an upper-case copy of the given text. */
std::string toUpperCopy(const std::string & text);

/** @brief Byte spans of one item inside a packed ADS sum-write request. */
struct SumWriteItemSpan
{
  size_t header_offset{0};
  size_t header_length{0};
  size_t data_offset{0};
  size_t data_length{0};
};

/**
 * @brief Builds a sum-write request holding only the items whose include entry is non-zero.
 *
 * Fills compact_request with the included headers followed by their data, and included_indices
 * with the original index of each included item. Neither allocates once both hold the capacity
 * of the full request.
 */
void compactSumWriteRequest(
  const std::vector<uint8_t> & full_request,
  const std::vector<SumWriteItemSpan> & spans,
  const std::vector<uint8_t> & include,
  std::vector<uint8_t> & compact_request,
  std::vector<size_t> & included_indices);

/** @brief Scheduling policy for the I/O threads: leave as created, SCHED_FIFO, SCHED_RR or SCHED_OTHER. */
enum class SchedulingPolicy
{
  INHERIT,
  FIFO,
  ROUND_ROBIN,
  OTHER,
};

/** @brief Scheduling configuration applied to both I/O threads. */
struct ThreadSchedulingConfig
{
  SchedulingPolicy policy{SchedulingPolicy::FIFO};
  int priority{50};
  std::vector<unsigned int> cpu_affinity;
};

/** @brief Outcome of parsing the I/O thread scheduling parameters. */
struct ThreadSchedulingParseResult
{
  bool valid{true};
  ThreadSchedulingConfig config;
  std::string error;
};

/**
 * @brief Parses the I/O thread scheduling parameters.
 *
 * Policy is one of fifo, rr, other or inherit (empty means fifo), priority 1 to 99 for the
 * real-time policies (empty means 50), affinity a comma-separated CPU list (empty means no pinning).
 * An invalid input yields valid = false and the default configuration.
 */
ThreadSchedulingParseResult parseThreadScheduling(
  const std::string & policy_text,
  const std::string & priority_text,
  const std::string & affinity_text);

/** @brief Monotonically increasing sequence counter for streamed setpoints, wrapping at 32 bits. */
class SetpointSequenceCounter
{
public:
  SetpointSequenceCounter() = default;

  /** @brief Creates a counter that holds start before the first advance. */
  explicit SetpointSequenceCounter(uint32_t start);

  /** @brief Advances the counter by one write cycle and returns the new value. */
  uint32_t next();

  /** @brief The value returned by the most recent advance. */
  uint32_t current() const;

private:
  uint32_t value_{0};
};

/** @brief Converts a steady-clock instant to seconds, as a PLC LREAL carries them. */
double monotonicSeconds(const std::chrono::steady_clock::time_point & instant);

}  // namespace utilities
}  // namespace beckhoff_ads_client
