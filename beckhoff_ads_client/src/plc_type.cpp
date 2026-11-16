// Copyright (c) 2025, b-robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include "beckhoff_ads_client/plc_type.hpp"

#include <array>
#include <cmath>
#include <cstring>
#include <string>
#include <utility>

#include "beckhoff_ads_client/ads_client_utilities.hpp"

namespace beckhoff_ads_client
{
namespace
{

constexpr std::array<std::pair<std::string_view, PlcType>, 10> PLC_TYPE_NAMES{{
  {"BOOL", PlcType::Bool},
  {"BYTE", PlcType::Byte},
  {"SINT", PlcType::Sint},
  {"USINT", PlcType::Usint},
  {"INT", PlcType::Int},
  {"UINT", PlcType::Uint},
  {"DINT", PlcType::Dint},
  {"UDINT", PlcType::Udint},
  {"REAL", PlcType::Real},
  {"LREAL", PlcType::Lreal},
}};

template<typename StoredType>
double load_as_double(std::span<const std::uint8_t> bytes)
{
  StoredType stored{};
  std::memcpy(&stored, bytes.data(), sizeof(stored));
  return static_cast<double>(stored);
}

template<typename StoredType>
void store(StoredType stored, std::span<std::uint8_t> bytes)
{
  std::memcpy(bytes.data(), &stored, sizeof(stored));
}

template<typename StoredType>
void store_rounded(double value, std::span<std::uint8_t> bytes)
{
  store(static_cast<StoredType>(std::round(value)), bytes);
}

}  // namespace

std::optional<PlcType> parse_plc_type(std::string_view name)
{
  const std::string upper = utilities::toUpperCopy(std::string(name));
  std::optional<PlcType> result;
  for (const auto & [type_name, type] : PLC_TYPE_NAMES) {
    if (upper == type_name) {
      result = type;
    }
  }
  return result;
}

std::string_view plc_type_name(PlcType type)
{
  std::string_view result;
  for (const auto & [type_name, candidate] : PLC_TYPE_NAMES) {
    if (candidate == type) {
      result = type_name;
    }
  }
  return result;
}

std::size_t plc_type_size(PlcType type)
{
  std::size_t result = 1;
  switch (type) {
    case PlcType::Bool:
    case PlcType::Byte:
    case PlcType::Sint:
    case PlcType::Usint:
      result = 1;
      break;
    case PlcType::Int:
    case PlcType::Uint:
      result = 2;
      break;
    case PlcType::Dint:
    case PlcType::Udint:
    case PlcType::Real:
      result = 4;
      break;
    case PlcType::Lreal:
      result = 8;
      break;
  }
  return result;
}

double decode_plc_value(PlcType type, std::span<const std::uint8_t> bytes)
{
  double result = 0.0;
  switch (type) {
    case PlcType::Bool:
      result = bytes[0] != 0 ? 1.0 : 0.0;
      break;
    case PlcType::Byte:
    case PlcType::Usint:
      result = load_as_double<std::uint8_t>(bytes);
      break;
    case PlcType::Sint:
      result = load_as_double<std::int8_t>(bytes);
      break;
    case PlcType::Int:
      result = load_as_double<std::int16_t>(bytes);
      break;
    case PlcType::Uint:
      result = load_as_double<std::uint16_t>(bytes);
      break;
    case PlcType::Dint:
      result = load_as_double<std::int32_t>(bytes);
      break;
    case PlcType::Udint:
      result = load_as_double<std::uint32_t>(bytes);
      break;
    case PlcType::Real:
      result = load_as_double<float>(bytes);
      break;
    case PlcType::Lreal:
      result = load_as_double<double>(bytes);
      break;
  }
  return result;
}

void encode_plc_value(PlcType type, double value, std::span<std::uint8_t> bytes)
{
  switch (type) {
    case PlcType::Bool:
      store<std::uint8_t>(value != 0.0 ? 1 : 0, bytes);
      break;
    case PlcType::Byte:
    case PlcType::Usint:
      store_rounded<std::uint8_t>(value, bytes);
      break;
    case PlcType::Sint:
      store_rounded<std::int8_t>(value, bytes);
      break;
    case PlcType::Int:
      store_rounded<std::int16_t>(value, bytes);
      break;
    case PlcType::Uint:
      store_rounded<std::uint16_t>(value, bytes);
      break;
    case PlcType::Dint:
      store_rounded<std::int32_t>(value, bytes);
      break;
    case PlcType::Udint:
      store_rounded<std::uint32_t>(value, bytes);
      break;
    case PlcType::Real:
      store(static_cast<float>(value), bytes);
      break;
    case PlcType::Lreal:
      store(value, bytes);
      break;
  }
}

}  // namespace beckhoff_ads_client
