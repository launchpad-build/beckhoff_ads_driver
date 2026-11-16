// Copyright (c) 2025, b-robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

/**
 * @file plc_type.hpp
 * @brief The PLC element types the client exchanges, and their conversion to and from double.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>

namespace beckhoff_ads_client
{

/** @brief A scalar TwinCAT type, carried as one double on the client side. */
enum class PlcType
{
  Bool,
  Byte,
  Sint,
  Usint,
  Int,
  Uint,
  Dint,
  Udint,
  Real,
  Lreal,
};

/** @brief The type named by a TwinCAT type name such as "LREAL", in any case, or nothing if unsupported. */
std::optional<PlcType> parse_plc_type(std::string_view name);

/** @brief The TwinCAT name of the type, upper case. */
std::string_view plc_type_name(PlcType type);

/** @brief Size of one element of the type on the PLC, in bytes. */
std::size_t plc_type_size(PlcType type);

/** @brief Decodes one element from the front of bytes, which holds at least plc_type_size(type) bytes. */
double decode_plc_value(PlcType type, std::span<const std::uint8_t> bytes);

/**
 * @brief Encodes value as one element at the front of bytes, which holds at least plc_type_size(type) bytes.
 *
 * BOOL stores whether value is non-zero; integer types store value rounded to the nearest whole number.
 */
void encode_plc_value(PlcType type, double value, std::span<std::uint8_t> bytes);

}  // namespace beckhoff_ads_client
