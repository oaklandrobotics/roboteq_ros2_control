#ifndef CANOPEN_ENUMS_HPP
#define CANOPEN_ENUMS_HPP

#include <cstdint>

namespace canopen
{
  inline constexpr std::uint8_t kBroadcastNodeId = 0;
  inline constexpr std::uint8_t kMinNodeId = 1;
  inline constexpr std::uint8_t kMaxNodeId = 127;

  /// @brief CANopen communication object identifier (COB-ID) base values.
  ///
  /// Representation of the standard 11-bit CAN id bases used by CANopen services
  /// For node-specific services, the node ID is typically added to the base value
  /// NMT is a fixed identifier `0x000`
  enum class COBID : std::uint16_t
  {
    NMT = 0x000,

    SDOResponse = 0x580,
    SDORequest = 0x600,
    Heartbeat = 0x700,

    TPDO1 = 0x180,
    TPDO2 = 0x280,
    TPDO3 = 0x380,
    TPDO4 = 0x480,
    RPDO1 = 0x200,
    RPDO2 = 0x300,
    RPDO3 = 0x400,
    RPDO4 = 0x500
  };

  /// @brief Network Management (NMT) command specifiers.
  ///
  /// These values are placed in the first data byte of an NMT frame
  /// transmitted with the `COBID::NMT` identifier
  enum class NMT : std::uint8_t
  {
    goToOperational = 0x01,
    goToStopped = 0x02,
    goToPreOperational = 0x80,
    goToResetNode = 0x81,
    goToResetComms = 0x82
  };

  /// @brief Service Data Object (SDO) command specifiers.
  ///
  /// These values occupy the command byte of an SDO frame and determine
  /// the transfer direction and payload size
  enum class SDOCommand : std::uint8_t
  {
    writeRequest4Byte = 0x23,
    writeRequest2Byte = 0x2B,
    writeRequest1Byte = 0x2F,

    writeResponse = 0x60,

    readRequest = 0x40,

    readResponse4Byte = 0x43,
    readResponse2Byte = 0x4B,
    readResponse1Byte = 0x4F,

    errorResponse = 0x80
  };
}

#endif