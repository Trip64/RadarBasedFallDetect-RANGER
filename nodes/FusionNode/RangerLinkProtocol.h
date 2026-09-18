#pragma once

#include <Arduino.h>

namespace RangerLink {

constexpr uint16_t TELEMETRY_MAGIC = 0x5247; // "RG" on little-endian MCUs
constexpr uint8_t PROTOCOL_VERSION = 3;
constexpr uint8_t FUSION_I2C_ADDRESS = 0x42;

enum TelemetryFlags : uint8_t {
  FLAG_WEARABLE_LINK = 1U << 0,
  FLAG_RADAR_PRESENT = 1U << 1,
  FLAG_SOS_ACTIVE = 1U << 2,
  FLAG_RELAY_ACTIVE = 1U << 3,
  FLAG_BASE_FRESH = 1U << 4,
  FLAG_RADAR_FRESH = 1U << 5,
  FLAG_AUDIO_READY = 1U << 6,
};

enum Command : uint8_t {
  CMD_NONE = 0x00,
  CMD_CANCEL_ALARM = 0x01,
  CMD_RELAY_TEST = 0x02,
  CMD_IMPACT_ENABLE = 0x03,
  CMD_IMPACT_DISABLE = 0x04,
  CMD_STILLNESS_ENABLE = 0x05,
  CMD_STILLNESS_DISABLE = 0x06,
};

#pragma pack(push, 1)
struct TelemetryPacket {
  uint16_t magic;
  uint8_t protocolVersion;
  uint8_t packetSize;
  uint16_t sequence;
  uint16_t uptimeSeconds;

  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t gx;
  int16_t gy;
  int16_t gz;

  uint8_t battery;
  uint8_t fallState;
  uint8_t flags;
  uint8_t radarTargets;
  int16_t radarDistanceMm;
  int16_t radarSpeedCms;
  uint8_t mlFall;
  uint8_t mlConfidence;

  uint16_t crc16;
};

struct CommandPacket {
  uint16_t magic;
  uint8_t protocolVersion;
  uint8_t command;
  uint8_t crc8;
};
#pragma pack(pop)

static_assert(sizeof(TelemetryPacket) == 32,
              "TelemetryPacket must remain one 32-byte I2C transfer");
static_assert(sizeof(CommandPacket) == 5, "CommandPacket wire layout changed");

inline uint16_t crc16Ccitt(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  while (length--) {
    crc ^= static_cast<uint16_t>(*data++) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
                            : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

inline uint8_t crc8(const uint8_t *data, size_t length) {
  uint8_t crc = 0;
  while (length--) {
    crc ^= *data++;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80U) ? static_cast<uint8_t>((crc << 1) ^ 0x07U)
                          : static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

inline void finalizeTelemetry(TelemetryPacket &packet) {
  packet.magic = TELEMETRY_MAGIC;
  packet.protocolVersion = PROTOCOL_VERSION;
  packet.packetSize = sizeof(TelemetryPacket);
  packet.crc16 = crc16Ccitt(reinterpret_cast<const uint8_t *>(&packet),
                            sizeof(TelemetryPacket) - sizeof(packet.crc16));
}

inline bool validateTelemetry(const TelemetryPacket &packet) {
  if (packet.magic != TELEMETRY_MAGIC ||
      packet.protocolVersion != PROTOCOL_VERSION ||
      packet.packetSize != sizeof(TelemetryPacket)) {
    return false;
  }

  return packet.crc16 ==
         crc16Ccitt(reinterpret_cast<const uint8_t *>(&packet),
                    sizeof(TelemetryPacket) - sizeof(packet.crc16));
}

inline CommandPacket makeCommand(Command command) {
  CommandPacket packet = {};
  packet.magic = TELEMETRY_MAGIC;
  packet.protocolVersion = PROTOCOL_VERSION;
  packet.command = static_cast<uint8_t>(command);
  packet.crc8 = crc8(reinterpret_cast<const uint8_t *>(&packet),
                     sizeof(CommandPacket) - sizeof(packet.crc8));
  return packet;
}

inline bool validateCommand(const CommandPacket &packet) {
  return packet.magic == TELEMETRY_MAGIC &&
         packet.protocolVersion == PROTOCOL_VERSION &&
         packet.crc8 == crc8(reinterpret_cast<const uint8_t *>(&packet),
                             sizeof(CommandPacket) - sizeof(packet.crc8));
}

} // namespace RangerLink
