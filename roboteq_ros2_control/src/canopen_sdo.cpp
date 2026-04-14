#include "roboteq_ros2_control/canopen_sdo.hpp"

// Helper function in a blank namespace so it stays internal
namespace
{
  can_frame make_base_sdo_request(std::uint8_t nodeId, roboteq::ObjectKey key, canopen::SDOCommand cmd)
  {
    can_frame frame{};

    frame.can_id = static_cast<canid_t>(static_cast<std::uint16_t>(canopen::COBID::SDORequest) + nodeId);
    frame.can_dlc = 8;
    frame.data[0] = static_cast<std::uint8_t>(cmd);

    // Index
    frame.data[1] = static_cast<std::uint8_t>(key.index & 0x00FF);
    frame.data[2] = static_cast<std::uint8_t>((key.index >> 8) & 0x00FF);

    // Sub-index
    frame.data[3] = key.subindex;

    return frame;
  }
}

namespace canopen
{
  can_frame build_sdo_read_request(std::uint8_t nodeId, roboteq::ObjectKey key)
  {
    // Create base frame
    can_frame frame = make_base_sdo_request(nodeId, key, SDOCommand::readRequest);

    // Data
    // Read requests don't require any data bytes

    return frame;
  }

  template <typename T>
  can_frame build_sdo_write_request(std::uint8_t nodeId, roboteq::ObjectKey key, T value)
  {
    if constexpr (sizeof(T) == 1)
    {
      // Create base frame
      can_frame frame = make_base_sdo_request(nodeId, key, SDOCommand::writeRequest1Byte);

      // Data
      frame.data[4] = static_cast<std::uint8_t>(value);

      return frame;
    }
    else if constexpr (sizeof(T) == 2)
    {
      // Create base frame
      can_frame frame = make_base_sdo_request(nodeId, key, SDOCommand::writeRequest2Byte);

      // Data
      frame.data[4] = static_cast<std::uint8_t>(value & 0x00FF);
      frame.data[5] = static_cast<std::uint8_t>((value >> 8) & 0x00FF);

      return frame;
    }
    else if constexpr (sizeof(T) == 4)
    {
      // Create base frame
      can_frame frame = make_base_sdo_request(nodeId, key, SDOCommand::writeRequest4Byte);

      // Data
      frame.data[4] = static_cast<std::uint8_t>(value & 0x00FF);
      frame.data[5] = static_cast<std::uint8_t>((value >> 8) & 0x00FF);
      frame.data[6] = static_cast<std::uint8_t>((value >> 16) & 0x00FF);
      frame.data[7] = static_cast<std::uint8_t>((value >> 24) & 0x00FF);

      return frame;
    }
    else
    {
      static_assert(sizeof(T) == 1 || sizeof(T) == 2 || sizeof(T) == 4,
                    "SDO write request only supports 1, 2, or 4 byte values");
      return can_frame{};
    }
  }

  bool is_sdo_response(const can_frame& frame, std::uint8_t nodeId)
  {
    canid_t responseHeader = static_cast<canid_t>(static_cast<std::uint16_t>(COBID::SDOResponse) + nodeId);

    return (frame.can_id & CAN_SFF_MASK) == responseHeader;
  }

  bool is_sdo_error_response(const can_frame& frame)
  {
    return (frame.data[0] == static_cast<std::uint8_t>(SDOCommand::errorResponse));
  }

  std::uint32_t parse_sdo_abort_code(const can_frame& frame)
  {
    if (!is_sdo_error_response(frame))
    {
      return 0;
    }

    return (static_cast<std::uint32_t>(frame.data[7]) << 24) |
           (static_cast<std::uint32_t>(frame.data[6]) << 16) |
           (static_cast<std::uint32_t>(frame.data[5]) << 8)  |
           (static_cast<std::uint32_t>(frame.data[4]));
  }

  template <typename T>
  T parse_sdo_read_response(const can_frame& frame, std::uint8_t nodeId)
  {
    if (!is_sdo_response(frame, nodeId) || is_sdo_error_response(frame))
    {
      return T{};
    }

    if constexpr (sizeof(T) == 1)
    {
      if (frame.data[0] != static_cast<std::uint8_t>(SDOCommand::readResponse1Byte))
      {
        return T{};
      }

      return static_cast<T>(frame.data[4]);
    }
    else if constexpr (sizeof(T) == 2)
    {
      if (frame.data[0] != static_cast<std::uint8_t>(SDOCommand::readResponse2Byte))
      {
        return T{};
      }

      return static_cast<T>(
        (static_cast<std::uint16_t>(frame.data[5]) << 8) |
        (static_cast<std::uint16_t>(frame.data[4]))
      );
    }
    else if constexpr (sizeof(T) == 4)
    {
      if (frame.data[0] != static_cast<std::uint8_t>(SDOCommand::readResponse4Byte))
      {
        return T{};
      }

      return static_cast<T>(
        (static_cast<std::uint32_t>(frame.data[7]) << 24) |
        (static_cast<std::uint32_t>(frame.data[6]) << 16) |
        (static_cast<std::uint32_t>(frame.data[5]) << 8) |
        (static_cast<std::uint32_t>(frame.data[4]))
      );
    }
    else
    {
      static_assert(sizeof(T) == 1 || sizeof(T) == 2 || sizeof(T) == 4,
                    "SDO read response only supports 1, 2, or 4 byte values");
      return T{};
    }
  }

  template can_frame build_sdo_write_request<std::uint8_t>(std::uint8_t, roboteq::ObjectKey, std::uint8_t);
  template can_frame build_sdo_write_request<std::uint16_t>(std::uint8_t, roboteq::ObjectKey, std::uint16_t);
  template can_frame build_sdo_write_request<std::uint32_t>(std::uint8_t, roboteq::ObjectKey, std::uint32_t);

  template std::uint8_t parse_sdo_read_response<std::uint8_t>(const can_frame&, std::uint8_t);
  template std::uint16_t parse_sdo_read_response<std::uint16_t>(const can_frame&, std::uint8_t);
  template std::uint32_t parse_sdo_read_response<std::uint32_t>(const can_frame&, std::uint8_t);
}
