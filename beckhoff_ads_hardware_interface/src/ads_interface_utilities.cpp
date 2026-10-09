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
#include <cstring>
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

  ByteCountParseResult parseByteCount(const std::string &text)
  {
    ByteCountParseResult result;
    if (text.empty() || !std::all_of(text.begin(), text.end(), [](unsigned char c)
                                     { return std::isdigit(c) != 0; }))
    {
      result.error = "not a non-negative whole number";
    }
    else
    {
      try
      {
        result.value = static_cast<size_t>(std::stoull(text));
        result.valid = true;
      }
      catch (const std::exception &ex)
      {
        result.error = ex.what();
      }
    }
    return result;
  }

  std::string validateStructFields(size_t byte_size, std::vector<StructFieldSpan> fields)
  {
    std::string error;
    std::sort(fields.begin(), fields.end(), [](const StructFieldSpan &a, const StructFieldSpan &b)
              { return a.offset < b.offset; });
    for (size_t i = 0; i < fields.size() && error.empty(); ++i)
    {
      const StructFieldSpan &field = fields[i];
      if (field.size == 0 || field.offset > byte_size || field.size > byte_size - field.offset)
      {
        error = "field '" + field.name + "' at byte " + std::to_string(field.offset) + " (" +
                std::to_string(field.size) + " bytes) does not fit in " + std::to_string(byte_size) + " bytes";
      }
      else if (i + 1 < fields.size() && fields[i + 1].offset < field.offset + field.size)
      {
        error = "fields '" + field.name + "' and '" + fields[i + 1].name + "' overlap at byte " +
                std::to_string(fields[i + 1].offset);
      }
    }
    return error;
  }

  std::optional<uint32_t> symbolSizeFromEntry(const std::vector<uint8_t> &entry, size_t bytes_read)
  {
    // AdsSymbolEntry: entryLength, iGroup, iOffs, size, dataType, flags (uint32 each), then three uint16 lengths.
    constexpr size_t SIZE_OFFSET = 12;
    constexpr size_t HEADER_SIZE = 6 * sizeof(uint32_t) + 3 * sizeof(uint16_t);
    std::optional<uint32_t> result;
    if (bytes_read >= HEADER_SIZE && entry.size() >= HEADER_SIZE)
    {
      uint32_t size = 0;
      std::memcpy(&size, entry.data() + SIZE_OFFSET, sizeof(size));
      result = size;
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

  namespace
  {
    bool parseSchedulingPolicy(const std::string &policy_text, SchedulingPolicy &policy)
    {
      bool result = true;
      if (policy_text.empty() || policy_text == "fifo")
      {
        policy = SchedulingPolicy::FIFO;
      }
      else if (policy_text == "rr")
      {
        policy = SchedulingPolicy::ROUND_ROBIN;
      }
      else if (policy_text == "other")
      {
        policy = SchedulingPolicy::OTHER;
      }
      else if (policy_text == "inherit")
      {
        policy = SchedulingPolicy::INHERIT;
      }
      else
      {
        result = false;
      }
      return result;
    }

    bool parseCpuList(const std::string &affinity_text, std::vector<unsigned int> &cpus)
    {
      bool result = true;
      size_t start = 0;
      while (result && start <= affinity_text.size() && !affinity_text.empty())
      {
        const size_t comma = affinity_text.find(',', start);
        const std::string token = affinity_text.substr(
            start, comma == std::string::npos ? std::string::npos : comma - start);
        try
        {
          size_t consumed = 0;
          const unsigned long cpu = std::stoul(token, &consumed);
          if (consumed != token.size() || cpu > 1023)
          {
            result = false;
          }
          else
          {
            cpus.push_back(static_cast<unsigned int>(cpu));
          }
        }
        catch (const std::exception &)
        {
          result = false;
        }
        if (comma == std::string::npos)
        {
          break;
        }
        start = comma + 1;
      }
      return result;
    }
  } // namespace

  ThreadSchedulingParseResult parseThreadScheduling(
      const std::string &policy_text,
      const std::string &priority_text,
      const std::string &affinity_text)
  {
    ThreadSchedulingParseResult result;

    if (!parseSchedulingPolicy(policy_text, result.config.policy))
    {
      result.valid = false;
      result.error = "unknown policy '" + policy_text + "', expected fifo, rr, other or inherit";
    }

    const bool real_time = result.config.policy == SchedulingPolicy::FIFO ||
                           result.config.policy == SchedulingPolicy::ROUND_ROBIN;
    if (result.valid && !real_time)
    {
      result.config.priority = 0;
    }
    if (result.valid && real_time && !priority_text.empty())
    {
      try
      {
        size_t consumed = 0;
        const int priority = std::stoi(priority_text, &consumed);
        if (consumed != priority_text.size() || priority < 1 || priority > 99)
        {
          result.valid = false;
          result.error = "priority '" + priority_text + "' outside 1 to 99";
        }
        else
        {
          result.config.priority = priority;
        }
      }
      catch (const std::exception &)
      {
        result.valid = false;
        result.error = "priority '" + priority_text + "' is not a number";
      }
    }

    if (result.valid && !affinity_text.empty() &&
        !parseCpuList(affinity_text, result.config.cpu_affinity))
    {
      result.valid = false;
      result.error = "affinity '" + affinity_text + "' is not a comma-separated CPU list";
    }

    if (!result.valid)
    {
      result.config = ThreadSchedulingConfig{};
    }

    return result;
  }

  SetpointSequenceCounter::SetpointSequenceCounter(uint32_t start)
      : value_(start)
  {
  }

  uint32_t SetpointSequenceCounter::next()
  {
    value_ += 1u;
    const uint32_t result = value_;
    return result;
  }

  uint32_t SetpointSequenceCounter::current() const
  {
    return value_;
  }

  double monotonicSeconds(const std::chrono::steady_clock::time_point &instant)
  {
    const std::chrono::duration<double> seconds = instant.time_since_epoch();
    return seconds.count();
  }

  void compactSumWriteRequest(
      const std::vector<uint8_t> &full_request,
      const std::vector<SumWriteItemSpan> &spans,
      const std::vector<uint8_t> &include,
      std::vector<uint8_t> &compact_request,
      std::vector<size_t> &included_indices)
  {
    compact_request.clear();
    included_indices.clear();
    for (size_t i = 0; i < spans.size(); ++i)
    {
      if (include[i] != 0)
      {
        compact_request.insert(compact_request.end(),
                               full_request.begin() + spans[i].header_offset,
                               full_request.begin() + spans[i].header_offset + spans[i].header_length);
        included_indices.push_back(i);
      }
    }
    for (const size_t i : included_indices)
    {
      compact_request.insert(compact_request.end(),
                             full_request.begin() + spans[i].data_offset,
                             full_request.begin() + spans[i].data_offset + spans[i].data_length);
    }
  }

} // namespace utilities
} // namespace beckhoff_ads_hardware_interface
