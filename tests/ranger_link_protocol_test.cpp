#include <cassert>
#include <cstdint>
#include <iostream>

#include "../nodes/CrowPanel/RangerLinkProtocol.h"

int main() {
  const uint8_t canonical[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  assert(RangerLink::crc16Ccitt(canonical, sizeof(canonical)) == 0x29B1);
  assert(RangerLink::crc8(canonical, sizeof(canonical)) == 0xF4);

  RangerLink::TelemetryPacket telemetry{};
  telemetry.sequence = 65535;
  telemetry.ax = -1234;
  telemetry.battery = 87;
  telemetry.flags = RangerLink::FLAG_WEARABLE_LINK |
                    RangerLink::FLAG_BASE_FRESH |
                    RangerLink::FLAG_AUDIO_READY;
  telemetry.radarDistanceMm = 2468;
  telemetry.mlFall = 91;
  RangerLink::finalizeTelemetry(telemetry);

  assert(sizeof(telemetry) == 32);
  assert(telemetry.magic == RangerLink::TELEMETRY_MAGIC);
  assert(telemetry.protocolVersion == RangerLink::PROTOCOL_VERSION);
  assert(telemetry.packetSize == sizeof(telemetry));
  assert(RangerLink::validateTelemetry(telemetry));

  RangerLink::TelemetryPacket corrupted = telemetry;
  corrupted.gy ^= 0x40;
  assert(!RangerLink::validateTelemetry(corrupted));

  const RangerLink::CommandPacket cancel =
      RangerLink::makeCommand(RangerLink::CMD_CANCEL_ALARM);
  const RangerLink::CommandPacket relay =
      RangerLink::makeCommand(RangerLink::CMD_RELAY_TEST);
  const RangerLink::CommandPacket impactOff =
      RangerLink::makeCommand(RangerLink::CMD_IMPACT_DISABLE);
  const RangerLink::CommandPacket stillnessOn =
      RangerLink::makeCommand(RangerLink::CMD_STILLNESS_ENABLE);
  assert(sizeof(cancel) == 5);
  assert(RangerLink::validateCommand(cancel));
  assert(RangerLink::validateCommand(relay));
  assert(RangerLink::validateCommand(impactOff));
  assert(RangerLink::validateCommand(stillnessOn));

  RangerLink::CommandPacket badCommand = cancel;
  badCommand.command ^= 0x10;
  assert(!RangerLink::validateCommand(badCommand));

  std::cout << "Ranger Link protocol tests passed\n";
  return 0;
}
