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

} // namespace utilities
} // namespace beckhoff_ads_hardware_interface
