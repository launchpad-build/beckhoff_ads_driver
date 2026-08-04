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

} // namespace utilities
} // namespace beckhoff_ads_hardware_interface
