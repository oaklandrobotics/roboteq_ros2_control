#ifndef CANOPEN_SDO_HPP
#define CANOPEN_SDO_HPP

#include <cstdint>
#include <linux/can.h>

#include "canopen_enums.hpp"
#include "roboteq_object_dictionary.hpp"

namespace canopen
{
  can_frame build_sdo_read_request(std::uint8_t nodeId, roboteq::ObjectKey key);

  template <typename T>
  can_frame build_sdo_write_request(std::uint8_t nodeId, roboteq::ObjectKey key, T value);

  bool is_sdo_response(const can_frame& frame, std::uint8_t nodeId);
  bool is_sdo_error_response(const can_frame& frame);
  std::uint32_t parse_sdo_abort_code(const can_frame& frame);

  template <typename T>
  T parse_sdo_read_response(const can_frame& frame, std::uint8_t nodeId);

  template <typename T>
  can_frame build_pdo_message(std::uint8_t nodeId, COBID cobId, T value);
}

#endif