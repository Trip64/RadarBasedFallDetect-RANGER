#include <cassert>
#include <cstdint>
#include <iostream>

namespace {

constexpr uint8_t kFallThreshold = 85;
constexpr uint8_t kFalseAlarmMargin = 10;

bool qualifiesAsFall(uint8_t fall, uint8_t falseAlarm, uint8_t idle,
                     uint8_t walk) {
  const bool fallIsWinner =
      fall >= falseAlarm && fall >= idle && fall >= walk;
  const bool clearsFalseAlarm =
      static_cast<uint16_t>(fall) >=
      static_cast<uint16_t>(falseAlarm) + kFalseAlarmMargin;
  return fall >= kFallThreshold && fallIsWinner && clearsFalseAlarm;
}

uint8_t packBatterySos(uint8_t battery, bool sos) {
  return (battery & 0x7fU) | (sos ? 0x80U : 0U);
}

} // namespace

int main() {
  assert(qualifiesAsFall(95, 2, 2, 1));
  assert(qualifiesAsFall(85, 75, 10, 5));
  assert(!qualifiesAsFall(84, 0, 0, 0));
  assert(!qualifiesAsFall(90, 85, 2, 1));
  assert(!qualifiesAsFall(90, 5, 95, 0));

  const uint8_t normal = packBatterySos(87, false);
  const uint8_t emergency = packBatterySos(87, true);
  assert((normal & 0x7fU) == 87 && (normal & 0x80U) == 0);
  assert((emergency & 0x7fU) == 87 && (emergency & 0x80U) != 0);

  std::cout << "RANGER fall policy tests passed\n";
  return 0;
}
