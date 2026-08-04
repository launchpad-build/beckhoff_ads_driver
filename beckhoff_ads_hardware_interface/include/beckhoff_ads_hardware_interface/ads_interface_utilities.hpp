// Copyright (c) 2025, b-robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

namespace beckhoff_ads_hardware_interface
{
namespace utilities
{

  /**
   * @brief Outcome of parsing an AMS port string
   */
  struct AmsPortParseResult
  {
    bool valid{false};
    uint16_t port{0};
    std::string error;
  };

  /**
   * @brief Parses an AMS port string with full range validation
   *
   * std::stoul alone truncates values above 65535 silently when narrowed to
   * uint16_t, so the value is range-checked before narrowing.
   *
   * @param text The port text from the hardware parameters
   * @returns AmsPortParseResult with the parsed port, or an error description
   */
  AmsPortParseResult parseAmsPort(const std::string &text);

  /**
   * @brief Returns an upper-case copy of the given text
   *
   * ::toupper on a plain char is undefined for negative values, so each
   * character is cast through unsigned char first.
   *
   * @param text The text to convert
   * @returns The upper-cased copy
   */
  std::string toUpperCopy(const std::string &text);

  /**
   * @brief Byte spans of one item inside a packed ADS sum-write request
   */
  struct SumWriteItemSpan
  {
    size_t header_offset{0};
    size_t header_length{0};
    size_t data_offset{0};
    size_t data_length{0};
  };

  /**
   * @brief Builds a sum-write request holding only the included items
   *
   * The full request is laid out as all item headers followed by all item
   * data blocks. The compacted request keeps that layout for the included
   * items only, so items whose values were never provided are not
   * transmitted at all.
   *
   * @param full_request The complete packed sum-write request
   * @param spans Header and data spans of each item in the full request
   * @param include One entry per item; zero excludes the item
   * @param compact_request Receives the compacted request bytes
   * @param included_indices Receives the original index of each included item
   */
  void compactSumWriteRequest(
      const std::vector<uint8_t> &full_request,
      const std::vector<SumWriteItemSpan> &spans,
      const std::vector<uint8_t> &include,
      std::vector<uint8_t> &compact_request,
      std::vector<size_t> &included_indices);

  /**
   * @brief Scheduling policy for the I/O threads
   */
  enum class SchedulingPolicy
  {
    INHERIT,     // leave the thread as created
    FIFO,        // SCHED_FIFO real-time scheduling
    ROUND_ROBIN, // SCHED_RR real-time scheduling
    OTHER,       // SCHED_OTHER, the normal time-sharing policy
  };

  /**
   * @brief Scheduling configuration applied to both I/O threads
   */
  struct ThreadSchedulingConfig
  {
    SchedulingPolicy policy{SchedulingPolicy::FIFO};
    int priority{50};
    std::vector<unsigned int> cpu_affinity;
  };

  /**
   * @brief Outcome of parsing the I/O thread scheduling parameters
   */
  struct ThreadSchedulingParseResult
  {
    bool valid{true};
    ThreadSchedulingConfig config;
    std::string error;
  };

  /**
   * @brief Parses the I/O thread scheduling hardware parameters
   *
   * Empty texts keep the defaults: SCHED_FIFO at priority 50 with no CPU
   * pinning. A real-time priority must lie in 1 to 99; inherit and other
   * take no priority. The affinity text is a comma-separated CPU list.
   *
   * @param policy_text One of fifo, rr, other or inherit, or empty for the default
   * @param priority_text The real-time priority, or empty for the default
   * @param affinity_text Comma-separated CPU indices, or empty for no pinning
   * @returns ThreadSchedulingParseResult with the configuration, or an error description
   */
  ThreadSchedulingParseResult parseThreadScheduling(
      const std::string &policy_text,
      const std::string &priority_text,
      const std::string &affinity_text);

  /**
   * @brief Wait-free latest-value hand-over between one writer and one reader
   *
   * Classic triple buffer: the writer always owns one slot, the reader always
   * owns one slot, and the third slot sits in the control word carrying the
   * most recently published sample. Neither side ever blocks or touches a
   * slot the other holds, so the reader always sees a whole sample and never
   * a mix of two, which per-element atomics cannot guarantee.
   *
   * @tparam SampleType The sample published as one unit
   */
  template <typename SampleType>
  class LatestSampleBuffer
  {
  public:
    /**
     * @brief Grants the writer access to its current slot
     *
     * @returns Reference to the slot the writer may fill; writer thread only
     */
    SampleType &writeSlot()
    {
      return slots_[write_index_];
    }

    /**
     * @brief Publishes the write slot as the latest sample
     *
     * The writer receives a fresh slot to fill next; writer thread only.
     */
    void publish()
    {
      write_index_ = control_.exchange(write_index_ | FRESH_FLAG, std::memory_order_acq_rel) & INDEX_MASK;
    }

    /**
     * @brief Takes the latest published sample as the read slot, if newer
     *
     * @returns True when a sample published after the last take was taken; reader thread only
     */
    bool refreshReadSlot()
    {
      bool result = false;
      if ((control_.load(std::memory_order_acquire) & FRESH_FLAG) != 0)
      {
        read_index_ = control_.exchange(read_index_, std::memory_order_acq_rel) & INDEX_MASK;
        result = true;
      }
      return result;
    }

    /**
     * @brief Grants the reader access to its current slot
     *
     * @returns Reference to the most recently taken sample; reader thread only
     */
    const SampleType &readSlot() const
    {
      return slots_[read_index_];
    }

    /**
     * @brief Applies a function to every slot, for pre-allocation at configure
     *
     * Only safe while no other thread touches the buffer.
     *
     * @tparam InitFunction Callable taking SampleType&
     * @param initialise The function applied to each slot
     */
    template <typename InitFunction>
    void initialiseSlots(InitFunction initialise)
    {
      for (SampleType &slot : slots_)
      {
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

} // namespace utilities
} // namespace beckhoff_ads_hardware_interface
