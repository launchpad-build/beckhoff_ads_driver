// Copyright (c) 2025, b-robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include "beckhoff_ads_hardware_interface/ads_interface_utilities.hpp"

#include <algorithm>
#include <cctype>
#include <stdexcept>

namespace beckhoff_ads_hardware_interface
{
namespace utilities
{

  AmsPortParseResult parseAmsPort(const std::string &text)
  {
    AmsPortParseResult result;
    try
    {
      size_t consumed = 0;
      const unsigned long value = std::stoul(text, &consumed);
      if (consumed != text.size())
      {
        result.error = "trailing characters after the number";
      }
      else if (value < 1 || value > 65535)
      {
        result.error = "value out of the AMS port range 1 to 65535";
      }
      else
      {
        result.port = static_cast<uint16_t>(value);
        result.valid = true;
      }
    }
    catch (const std::exception &ex)
    {
      result.error = ex.what();
    }
    return result;
  }

  std::string toUpperCopy(const std::string &text)
  {
    std::string result = text;
    std::transform(result.begin(), result.end(), result.begin(),
                   [](unsigned char c)
                   { return static_cast<char>(std::toupper(c)); });
    return result;
  }

} // namespace utilities
} // namespace beckhoff_ads_hardware_interface
