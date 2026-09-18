/**
 * RANGER v5.3 — Fusion Node (Raspberry Pi Pico / RP2040)
 *
 * UART0 GP0/GP1: Base CSV @115200; UART1 GP4/GP5: RD-03D @256000.
 * I2C1 GP2/GP3: CrowPanel master link; this node is slave 0x42.
 * Link v3 is versioned, sequenced and CRC16 checked. I2C callbacks only
 * move immutable bytes/flags; state changes happen in loop() context.
 * v5.3 makes impact and stillness behavior CrowPanel-controlled while keeping
 * the latched alarm, deterministic buzzer and serialized DFPlayer handling.
 */
#include "RangerLinkProtocol.h"
#include <Arduino.h>
#include <DFRobotDFPlayerMini.h>
#include <Wire.h>

#define VERSION "5.3-FUSION-FALL-CONFIG"

constexpr uint8_t PIN_BASE_TX = 0, PIN_BASE_RX = 1, PIN_I2C_SDA = 2,
                  PIN_I2C_SCL = 3;
constexpr uint8_t PIN_RADAR_TX = 4, PIN_RADAR_RX = 5, PIN_DFP_TX = 8,
                  PIN_DFP_RX = 9;
constexpr uint8_t PIN_LED_R = 13, PIN_LED_G = 14, PIN_LED_B = 15,
                  PIN_RELAY = 16, PIN_BUZZER = 22;
constexpr uint32_t RELAY_ON_TIME_MS = 60000, BASE_STALE_MS = 2000;
constexpr uint32_t RADAR_STALE_MS = 1500, TELEMETRY_PERIOD_MS = 100;

// The documented hardware is a passive piezo on GP22. Set this to true only
// when the attached part is an active buzzer module that sounds from steady DC.
constexpr bool BUZZER_IS_ACTIVE = false;
constexpr uint16_t BUZZER_LOW_HZ = 2200, BUZZER_HIGH_HZ = 2850;
constexpr uint32_t BUZZER_PATTERN_MS = 900, BUZZER_SELF_TEST_MS = 1800;

alignas(
    4) static uint8_t txBuffers[2][sizeof(RangerLink::TelemetryPacket)] = {};
static volatile uint8_t activeTxBuffer = 0;
static volatile bool cancelCommandPending = false,
                     relayTestCommandPending = false;
static volatile int8_t impactSettingPending = -1,
                       stillnessSettingPending = -1;
static volatile uint32_t i2cRequestCount = 0, i2cCommandCount = 0,
                         i2cCommandErrorCount = 0;
static uint16_t telemetrySequence = 0;

void onI2CRequest() {
  const uint8_t index = activeTxBuffer;
  Wire1.write(txBuffers[index], sizeof(RangerLink::TelemetryPacket));
  ++i2cRequestCount;
}

void onI2CReceive(int howMany) {
  RangerLink::CommandPacket command{};
  uint8_t *bytes = reinterpret_cast<uint8_t *>(&command);
  int count = 0;
  while (Wire1.available() && count < (int)sizeof(command))
    bytes[count++] = (uint8_t)Wire1.read();
  while (Wire1.available())
    Wire1.read();
  if (howMany != (int)sizeof(command) || count != (int)sizeof(command) ||
      !RangerLink::validateCommand(command)) {
    ++i2cCommandErrorCount;
    return;
  }
  if (command.command == RangerLink::CMD_CANCEL_ALARM)
    cancelCommandPending = true;
  else if (command.command == RangerLink::CMD_RELAY_TEST)
    relayTestCommandPending = true;
  else if (command.command == RangerLink::CMD_IMPACT_ENABLE)
    impactSettingPending = 1;
  else if (command.command == RangerLink::CMD_IMPACT_DISABLE)
    impactSettingPending = 0;
  else if (command.command == RangerLink::CMD_STILLNESS_ENABLE)
    stillnessSettingPending = 1;
  else if (command.command == RangerLink::CMD_STILLNESS_DISABLE)
    stillnessSettingPending = 0;
  else {
    ++i2cCommandErrorCount;
    return;
  }
  ++i2cCommandCount;
}

float ax = 0, ay = 0, az = 1, gx = 0, gy = 0, gz = 0, totalAccel = 1;
bool wearableLinked = false, sosActive = false;
int batteryPercent = 0;
int32_t currentPressurePa = 0;
uint32_t irVal = 0, redVal = 0, lastBasePacketMs = 0, lastImuPacketMs = 0;
uint8_t mlFall = 0, mlFalseAlarm = 0, mlIdle = 0, mlWalk = 0, mlWinnerIdx = 0,
        mlConfidence = 0, mlFlags = 0;

constexpr uint8_t ML_FALL_THRESHOLD = 85;
constexpr uint8_t ML_FALSE_ALARM_MARGIN = 10;
constexpr float IMPACT_TRIGGER_G = 2.5f;
constexpr uint32_t STILLNESS_REQUIRED_MS = 3000;
constexpr uint32_t FALL_VERIFY_TIMEOUT_MS = 8000, FALL_COOLDOWN_MS = 30000;
constexpr float STILLNESS_G_THRESHOLD = 0.25f;
enum FallTriggerSource : uint8_t {
  FALL_SOURCE_NONE,
  FALL_SOURCE_ML,
  FALL_SOURCE_IMPACT,
  FALL_SOURCE_BOTH
};
const char *fallSourceName(FallTriggerSource source);
void beginFallVerification(FallTriggerSource source, float triggerG);
void rejectFallCandidate(const char *reason);
void confirmFall();
int fallDetected = 0;
bool impactTriggerEnabled = false, stillnessCheckEnabled = true;
FallTriggerSource fallTriggerSource = FALL_SOURCE_NONE;
uint32_t fallTriggerTime = 0, stillnessStartMs = 0, lastFallTime = 0;
float candidatePeakG = 0;

bool relayActive = false, buzzerActive = false, buzzerSelfTestActive = false;
uint32_t relayStartMs = 0, buzzerSelfTestStartMs = 0,
         buzzerPatternStartMs = 0;
uint16_t buzzerFrequency = 0;
uint32_t lastRadarTime = 0;
bool radarPresence = false;
float radarDistM = 0;
int16_t radarSpeed = 0;
uint8_t radarTargetCount = 0;

uint32_t imuPackets = 0, envPackets = 0, mlPackets = 0, baseParseErrors = 0,
         radarFrames = 0;
SerialPIO dfpSerial(PIN_DFP_TX, PIN_DFP_RX);
DFRobotDFPlayerMini myDFPlayer;
bool dfpReady = false;

enum AudioTrack : uint8_t {
  AUDIO_NONE = 0,
  AUDIO_BOOT = 1,
  AUDIO_CONNECTED = 2,
  AUDIO_DISCONNECTED = 3,
  AUDIO_ALERT = 4,
  AUDIO_READY = 5,
};
constexpr size_t AUDIO_QUEUE_SIZE = 8;
constexpr uint32_t AUDIO_NORMAL_FALLBACK_MS = 5000,
                   AUDIO_ALERT_FALLBACK_MS = 10000;
uint8_t audioQueue[AUDIO_QUEUE_SIZE] = {};
size_t audioQueueHead = 0, audioQueueCount = 0;
uint8_t audioCurrentTrack = AUDIO_NONE;
uint32_t audioStartedMs = 0;
bool audioLinkCandidate = false, audioLinkStable = false,
     audioLinkCandidateInitialized = false, audioEverConnected = false;
uint32_t audioLinkCandidateSinceMs = 0;

template <typename T> T clampValue(T v, T lo, T hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}
int16_t scaledInt16(float v, float scale) {
  float s = v * scale;
  if (s > 32767)
    return 32767;
  if (s < (-32768))
    return -32768;
  return (int16_t)s;
}
bool baseFresh(uint32_t now) {
  return lastBasePacketMs && now - lastBasePacketMs <= BASE_STALE_MS;
}
bool radarFresh(uint32_t now) {
  return lastRadarTime && now - lastRadarTime <= RADAR_STALE_MS;
}

void publishTelemetry() {
  const uint32_t now = millis();
  RangerLink::TelemetryPacket p{};
  p.magic = RangerLink::TELEMETRY_MAGIC;
  p.protocolVersion = RangerLink::PROTOCOL_VERSION;
  p.packetSize = sizeof(p);
  p.sequence = ++telemetrySequence;
  p.uptimeSeconds = (uint16_t)((now / 1000UL) & 0xFFFFU);
  p.ax = scaledInt16(ax, 4096);
  p.ay = scaledInt16(ay, 4096);
  p.az = scaledInt16(az, 4096);
  p.gx = scaledInt16(gx, 65.5f);
  p.gy = scaledInt16(gy, 65.5f);
  p.gz = scaledInt16(gz, 65.5f);
  p.battery = (uint8_t)clampValue(batteryPercent, 0, 100);
  p.fallState = (uint8_t)clampValue(fallDetected, 0, 2);
  if (wearableLinked && baseFresh(now))
    p.flags |= RangerLink::FLAG_WEARABLE_LINK;
  if (radarPresence && radarFresh(now))
    p.flags |= RangerLink::FLAG_RADAR_PRESENT;
  if (sosActive)
    p.flags |= RangerLink::FLAG_SOS_ACTIVE;
  if (relayActive)
    p.flags |= RangerLink::FLAG_RELAY_ACTIVE;
  if (baseFresh(now))
    p.flags |= RangerLink::FLAG_BASE_FRESH;
  if (radarFresh(now))
    p.flags |= RangerLink::FLAG_RADAR_FRESH;
  if (dfpReady)
    p.flags |= RangerLink::FLAG_AUDIO_READY;
  p.radarTargets = radarFresh(now) ? radarTargetCount : 0;
  p.radarDistanceMm = radarFresh(now) ? scaledInt16(radarDistM, 1000) : 0;
  p.radarSpeedCms = radarFresh(now) ? radarSpeed : 0;
  p.mlFall = mlFall;
  p.mlConfidence = mlConfidence;
  RangerLink::finalizeTelemetry(p);
  const uint8_t next = activeTxBuffer ^ 1U;
  memcpy(txBuffers[next], &p, sizeof(p));
  noInterrupts();
  activeTxBuffer = next;
  interrupts();
}
void updateTelemStruct() { publishTelemetry(); }

const uint8_t RD03D_FRAME_HEADER[] = {0xAA, 0xFF, 0x03, 0x00};
constexpr int16_t SPEED_SENTINEL_248 = 248, SPEED_SENTINEL_256 = 256;
constexpr uint8_t RD03D_TARGET_DATA_SIZE = 8, RD03D_MAX_TARGETS = 3;
int16_t rd03dDecodeValue(uint8_t lo, uint8_t hi) {
  int16_t v = ((hi & 0x7F) << 8) | lo;
  if ((hi & 0x80) == 0)
    v = -v;
  return v;
}
bool rd03dSpeedValid(int16_t s) {
  int16_t a = s < 0 ? -s : s;
  return a != SPEED_SENTINEL_248 && a != SPEED_SENTINEL_256;
}

void setRGB(bool r, bool g, bool b) {
  digitalWrite(PIN_LED_R, r);
  digitalWrite(PIN_LED_G, g);
  digitalWrite(PIN_LED_B, b);
}
void relayOn() {
  if (relayActive)
    return;
  digitalWrite(PIN_RELAY, HIGH);
  relayActive = true;
  relayStartMs = millis();
  Serial.println("[RELAY] ON (60 s)");
  publishTelemetry();
}
void relayOff() {
  if (!relayActive)
    return;
  digitalWrite(PIN_RELAY, LOW);
  relayActive = false;
  Serial.println("[RELAY] OFF");
  publishTelemetry();
}
void relayTick() {
  if (relayActive && millis() - relayStartMs >= RELAY_ON_TIME_MS)
    relayOff();
}

void updateLEDs() {
  uint32_t now = millis();
  bool pulse = (now % 1000) < 500, fast = (now % 200) < 100;
  if (sosActive || fallDetected == 2)
    setRGB(fast, 0, 0);
  else if (relayActive)
    setRGB(0, pulse, pulse);
  else if (!wearableLinked)
    setRGB(0, 0, pulse);
  else if (radarPresence && radarFresh(now))
    setRGB(1, 1, 0);
  else
    setRGB(0, 1, 0);
}

const char *audioTrackName(uint8_t track) {
  switch (track) {
  case AUDIO_BOOT:
    return "boot";
  case AUDIO_CONNECTED:
    return "wearable connected";
  case AUDIO_DISCONNECTED:
    return "wearable disconnected";
  case AUDIO_ALERT:
    return "fall alert";
  case AUDIO_READY:
    return "system ready";
  default:
    return "unknown";
  }
}

void clearAudioQueue() {
  audioQueueHead = 0;
  audioQueueCount = 0;
}

bool audioAlreadyQueued(uint8_t track) {
  if (audioCurrentTrack == track)
    return true;
  for (size_t i = 0; i < audioQueueCount; ++i) {
    if (audioQueue[(audioQueueHead + i) % AUDIO_QUEUE_SIZE] == track)
      return true;
  }
  return false;
}

void queueAudio(uint8_t track) {
  if (!dfpReady || track == AUDIO_NONE || audioAlreadyQueued(track))
    return;
  if (audioQueueCount >= AUDIO_QUEUE_SIZE) {
    Serial.printf("[AUDIO] Queue full; dropped track %u (%s)\n", track,
                  audioTrackName(track));
    return;
  }
  const size_t tail = (audioQueueHead + audioQueueCount) % AUDIO_QUEUE_SIZE;
  audioQueue[tail] = track;
  ++audioQueueCount;
  Serial.printf("[AUDIO] Queued MP3/%04u.mp3 (%s)\n", track,
                audioTrackName(track));
}

void startAudioTrack(uint8_t track, bool interrupt) {
  if (!dfpReady || track == AUDIO_NONE)
    return;
  if (interrupt && audioCurrentTrack != AUDIO_NONE) {
    myDFPlayer.stop();
    Serial.printf("[AUDIO] Interrupted track %u for priority event\n",
                  audioCurrentTrack);
  }
  audioCurrentTrack = track;
  audioStartedMs = millis();
  myDFPlayer.playMp3Folder(track);
  Serial.printf("[AUDIO] Playing MP3/%04u.mp3 (%s)\n", track,
                audioTrackName(track));
}

void playPriorityAudio(uint8_t track) {
  if (!dfpReady)
    return;
  clearAudioQueue();
  startAudioTrack(track, true);
}

void stopAllAudio() {
  clearAudioQueue();
  if (dfpReady && audioCurrentTrack != AUDIO_NONE)
    myDFPlayer.stop();
  audioCurrentTrack = AUDIO_NONE;
}

void serviceAudio() {
  if (!dfpReady)
    return;
  while (myDFPlayer.available()) {
    const uint8_t type = myDFPlayer.readType();
    const uint16_t value = myDFPlayer.read();
    if (type == DFPlayerPlayFinished) {
      Serial.printf("[AUDIO] Finished track %u\n", value);
      audioCurrentTrack = AUDIO_NONE;
    } else if (type == DFPlayerCardRemoved) {
      Serial.println("[AUDIO] SD card removed");
      dfpReady = false;
      audioCurrentTrack = AUDIO_NONE;
      clearAudioQueue();
    } else if (type == DFPlayerCardInserted || type == DFPlayerCardOnline) {
      Serial.println("[AUDIO] SD card online");
    } else if (type == DFPlayerError) {
      Serial.printf("[AUDIO] DFPlayer error %u while playing track %u\n", value,
                    audioCurrentTrack);
      audioCurrentTrack = AUDIO_NONE;
    }
  }
  if (!dfpReady)
    return;
  const uint32_t fallbackMs = audioCurrentTrack == AUDIO_ALERT
                                  ? AUDIO_ALERT_FALLBACK_MS
                                  : AUDIO_NORMAL_FALLBACK_MS;
  if (audioCurrentTrack != AUDIO_NONE &&
      millis() - audioStartedMs >= fallbackMs) {
    Serial.printf("[AUDIO] No finish event for track %u; advancing queue\n",
                  audioCurrentTrack);
    audioCurrentTrack = AUDIO_NONE;
  }
  if (audioCurrentTrack == AUDIO_NONE && audioQueueCount != 0) {
    const uint8_t next = audioQueue[audioQueueHead];
    audioQueueHead = (audioQueueHead + 1) % AUDIO_QUEUE_SIZE;
    --audioQueueCount;
    startAudioTrack(next, false);
  }
}

void updateAudioLinkCues(uint32_t now) {
  const bool observed = wearableLinked && baseFresh(now);
  if (!audioLinkCandidateInitialized) {
    audioLinkCandidateInitialized = true;
    audioLinkCandidate = observed;
    audioLinkStable = observed;
    audioLinkCandidateSinceMs = now;
    if (observed) {
      audioEverConnected = true;
      queueAudio(AUDIO_CONNECTED);
    }
    return;
  }
  if (observed != audioLinkCandidate) {
    audioLinkCandidate = observed;
    audioLinkCandidateSinceMs = now;
    return;
  }
  const uint32_t requiredMs = observed ? 1000 : 3000;
  if (observed == audioLinkStable || now - audioLinkCandidateSinceMs < requiredMs)
    return;
  audioLinkStable = observed;
  if (observed) {
    audioEverConnected = true;
    queueAudio(AUDIO_CONNECTED);
  } else if (audioEverConnected) {
    queueAudio(AUDIO_DISCONNECTED);
  }
}

void setBuzzerOutput(bool on, uint16_t frequency = BUZZER_LOW_HZ) {
  if (!on) {
    if (!buzzerActive)
      return;
    noTone(PIN_BUZZER);
    digitalWrite(PIN_BUZZER, LOW);
    buzzerActive = false;
    buzzerFrequency = 0;
    return;
  }
  if (buzzerActive && (BUZZER_IS_ACTIVE || buzzerFrequency == frequency))
    return;
  if (BUZZER_IS_ACTIVE)
    digitalWrite(PIN_BUZZER, HIGH);
  else
    tone(PIN_BUZZER, frequency);
  buzzerActive = true;
  buzzerFrequency = frequency;
}

void startBuzzerSelfTest() {
  buzzerSelfTestActive = true;
  buzzerSelfTestStartMs = millis();
  buzzerPatternStartMs = buzzerSelfTestStartMs;
  Serial.printf("[BUZZER] GP%u self-test started (%s buzzer mode)\n", PIN_BUZZER,
                BUZZER_IS_ACTIVE ? "active" : "passive");
}

void serviceBuzzer(bool alarmActive) {
  const uint32_t now = millis();
  if (buzzerSelfTestActive &&
      now - buzzerSelfTestStartMs >= BUZZER_SELF_TEST_MS) {
    buzzerSelfTestActive = false;
    Serial.println("[BUZZER] Self-test finished");
  }
  if (!alarmActive && !buzzerSelfTestActive) {
    if (buzzerActive)
      setBuzzerOutput(false);
    return;
  }
  const uint32_t phase = (now - buzzerPatternStartMs) % BUZZER_PATTERN_MS;
  if (phase < 220)
    setBuzzerOutput(true, BUZZER_LOW_HZ);
  else if (phase < 360)
    setBuzzerOutput(false);
  else if (phase < 580)
    setBuzzerOutput(true, BUZZER_HIGH_HZ);
  else
    setBuzzerOutput(false);
}

void processActuators() {
  updateLEDs();
  static bool prevAlert = false;
  bool alert = sosActive || fallDetected == 2;
  if (alert && !prevAlert) {
    Serial.println("[ALARM] Latched alarm outputs active until cancel");
    buzzerPatternStartMs = millis();
    playPriorityAudio(AUDIO_ALERT);
  }
  serviceBuzzer(alert);
  updateAudioLinkCues(millis());
  serviceAudio();
  prevAlert = alert;
}
void handleI2CCommands() {
  bool cancel = false, test = false;
  int8_t impactSetting = -1, stillnessSetting = -1;
  noInterrupts();
  if (cancelCommandPending) {
    cancel = true;
    cancelCommandPending = false;
  }
  if (relayTestCommandPending) {
    test = true;
    relayTestCommandPending = false;
  }
  if (impactSettingPending != -1) {
    impactSetting = impactSettingPending;
    impactSettingPending = -1;
  }
  if (stillnessSettingPending != -1) {
    stillnessSetting = stillnessSettingPending;
    stillnessSettingPending = -1;
  }
  interrupts();
  if (cancel) {
    fallDetected = 0;
    fallTriggerSource = FALL_SOURCE_NONE;
    sosActive = false;
    stillnessStartMs = 0;
    candidatePeakG = 0;
    setBuzzerOutput(false);
    stopAllAudio();
    Serial.println("[LINK] Alarm cancelled");
    publishTelemetry();
  }
  if (test) {
    Serial.println("[LINK] Relay test");
    relayOn();
  }
  if (impactSetting != -1 || stillnessSetting != -1) {
    const bool newImpact = impactSetting == -1
                               ? impactTriggerEnabled
                               : impactSetting == 1;
    const bool newStillness = stillnessSetting == -1
                                  ? stillnessCheckEnabled
                                  : stillnessSetting == 1;
    const bool changed = newImpact != impactTriggerEnabled ||
                         newStillness != stillnessCheckEnabled;
    impactTriggerEnabled = newImpact;
    stillnessCheckEnabled = newStillness;
    Serial.printf("[FALL CFG] impact=%d stillness=%d\n", impactTriggerEnabled,
                  stillnessCheckEnabled);
    if (changed && fallDetected == 1)
      rejectFallCandidate("configuration changed");
  }
}

const char *fallSourceName(FallTriggerSource source) {
  switch (source) {
  case FALL_SOURCE_ML:
    return "ML";
  case FALL_SOURCE_IMPACT:
    return "IMPACT";
  case FALL_SOURCE_BOTH:
    return "ML+IMPACT";
  default:
    return "NONE";
  }
}

bool fallCooldownActive(uint32_t now) {
  return lastFallTime != 0 && now - lastFallTime < FALL_COOLDOWN_MS;
}

void beginFallVerification(FallTriggerSource source, float triggerG) {
  const uint32_t now = millis();
  if (fallDetected == 2 || fallCooldownActive(now))
    return;
  if (fallDetected == 1) {
    if (source != fallTriggerSource && fallTriggerSource != FALL_SOURCE_BOTH) {
      fallTriggerSource = FALL_SOURCE_BOTH;
      Serial.println("[FALL] Candidate reinforced by ML + impact");
    }
    if (triggerG > candidatePeakG)
      candidatePeakG = triggerG;
    return;
  }
  fallDetected = 1;
  fallTriggerSource = source;
  fallTriggerTime = now;
  stillnessStartMs = 0;
  candidatePeakG = triggerG;
  if (stillnessCheckEnabled)
    Serial.printf("[FALL] Candidate source=%s trigger=%.2fg; waiting for %.1fs "
                  "continuous stillness\n",
                  fallSourceName(source), triggerG,
                  STILLNESS_REQUIRED_MS / 1000.0f);
  else
    Serial.printf("[FALL] Candidate source=%s trigger=%.2fg\n",
                  fallSourceName(source), triggerG);
  publishTelemetry();
  if (!stillnessCheckEnabled) {
    Serial.println("[FALL] Stillness check disabled; confirming candidate");
    confirmFall();
  }
}

void rejectFallCandidate(const char *reason) {
  Serial.printf("[FALL] Rejected source=%s peak=%.2fg reason=%s\n",
                fallSourceName(fallTriggerSource), candidatePeakG, reason);
  fallDetected = 0;
  fallTriggerSource = FALL_SOURCE_NONE;
  stillnessStartMs = 0;
  candidatePeakG = 0;
  publishTelemetry();
}

void confirmFall() {
  if (fallDetected != 1)
    return;
  fallDetected = 2;
  lastFallTime = millis();
  stillnessStartMs = 0;
  Serial.printf("[FALL] CONFIRMED source=%s peak=%.2fg ml=%u%%\n",
                fallSourceName(fallTriggerSource), candidatePeakG, mlFall);
  publishTelemetry();
  relayOn();
}

void processImuFallSample(uint32_t now) {
  if (impactTriggerEnabled && totalAccel >= IMPACT_TRIGGER_G)
    beginFallVerification(FALL_SOURCE_IMPACT, totalAccel);
  if (fallDetected != 1)
    return;
  if (now - fallTriggerTime >= FALL_VERIFY_TIMEOUT_MS) {
    rejectFallCandidate("verification timeout");
    return;
  }
  const bool still = fabsf(totalAccel - 1.0f) < STILLNESS_G_THRESHOLD;
  if (!still) {
    if (stillnessStartMs != 0)
      Serial.printf("[FALL] Stillness reset at %.2fg\n", totalAccel);
    stillnessStartMs = 0;
    return;
  }
  if (stillnessStartMs == 0) {
    stillnessStartMs = now;
    Serial.println("[FALL] Stillness timer started");
    return;
  }
  if (now - stillnessStartMs >= STILLNESS_REQUIRED_MS)
    confirmFall();
}

void serviceFallVerification(uint32_t now) {
  if (fallDetected != 1)
    return;
  if (!lastImuPacketMs || now - lastImuPacketMs > 300) {
    if (stillnessStartMs != 0) {
      stillnessStartMs = 0;
      Serial.println("[FALL] Stillness reset: IMU stream stale");
    }
  }
  if (now - fallTriggerTime >= FALL_VERIFY_TIMEOUT_MS)
    rejectFallCandidate("verification timeout");
}

void processMLResult(uint8_t fall, uint8_t falseAlarm, uint8_t idle,
                     uint8_t walk, uint8_t winner, uint8_t confidence,
                     uint8_t flags) {
  mlFall = fall;
  mlFalseAlarm = falseAlarm;
  mlIdle = idle;
  mlWalk = walk;
  mlWinnerIdx = winner;
  mlConfidence = confidence;
  mlFlags = flags;
  // SOS is a safety event: latch it here and clear it only through the
  // CrowPanel cancel command. Releasing the wearable button must not silence
  // an emergency that was already raised.
  if (flags & 1)
    sosActive = true;
  const bool fallIsWinner = fall >= falseAlarm && fall >= idle && fall >= walk;
  const bool clearsFalseAlarm =
      (uint16_t)fall >= (uint16_t)falseAlarm + ML_FALSE_ALARM_MARGIN;
  if (fall >= ML_FALL_THRESHOLD && fallIsWinner && clearsFalseAlarm)
    beginFallVerification(FALL_SOURCE_ML, totalAccel);
  else if (fall >= ML_FALL_THRESHOLD)
    Serial.printf("[ML] Fall score %u%% vetoed (false=%u idle=%u walk=%u)\n",
                  fall, falseAlarm, idle, walk);
}

void processBaseLine(char *line) {
  uint32_t now = millis();
  if (line[0] == 'I' && line[1] == ',') {
    int16_t x, y, z, gxi, gyi, gzi;
    int seq;
    if (sscanf(line, "I,%hd,%hd,%hd,%hd,%hd,%hd,%d", &x, &y, &z, &gxi, &gyi,
               &gzi, &seq) >= 6) {
      ++imuPackets;
      lastBasePacketMs = lastImuPacketMs = now;
      wearableLinked = true;
      ax = x / 4096.0f;
      ay = y / 4096.0f;
      az = z / 4096.0f;
      gx = gxi / 65.5f;
      gy = gyi / 65.5f;
      gz = gzi / 65.5f;
      totalAccel = sqrtf(ax * ax + ay * ay + az * az);
      processImuFallSample(now);
      return;
    }
  } else if (line[0] == 'E' && line[1] == ',') {
    long pressure;
    unsigned long ir, red;
    int battery, sos;
    unsigned int seq;
    if (sscanf(line, "E,%ld,%lu,%lu,%d,%d,%u", &pressure, &ir, &red, &battery,
               &sos, &seq) >= 5) {
      ++envPackets;
      lastBasePacketMs = now;
      wearableLinked = true;
      currentPressurePa = pressure;
      irVal = ir;
      redVal = red;
      batteryPercent = clampValue(battery, 0, 100);
      if (sos != 0)
        sosActive = true;
      return;
    }
  } else if (line[0] == 'M' && line[1] == ',') {
    int f, fa, id, wk, wi, co, fl;
    if (sscanf(line, "M,%d,%d,%d,%d,%d,%d,%d", &f, &fa, &id, &wk, &wi, &co,
               &fl) == 7) {
      ++mlPackets;
      lastBasePacketMs = now;
      wearableLinked = true;
      processMLResult(clampValue(f, 0, 100), clampValue(fa, 0, 100),
                      clampValue(id, 0, 100), clampValue(wk, 0, 100),
                      clampValue(wi, 0, 255), clampValue(co, 0, 100),
                      clampValue(fl, 0, 255));
      return;
    }
  } else if (strncmp(line, "STAT,", 5) == 0) {
    const char *e = line + 5;
    lastBasePacketMs = now;
    if (!strcmp(e, "BASE_LINK_OK") || !strcmp(e, "W_OK"))
      wearableLinked = true;
    else if (!strcmp(e, "LINK_LOST") || !strcmp(e, "W_LOST"))
      wearableLinked = false;
    Serial.print("[BASE] ");
    Serial.println(e);
    return;
  }
  ++baseParseErrors;
  Serial.print("[BASE] Parse error: ");
  Serial.println(line);
}
void pollBase() {
  static char buffer[300];
  static size_t used = 0;
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\n' || c == '\r') {
      if (used > 1) {
        buffer[used] = '\0';
        processBaseLine(buffer);
        used = 0;
      }
    } else if (used < sizeof(buffer) - 1)
      buffer[used++] = c;
    else {
      used = 0;
      ++baseParseErrors;
    }
  }
}

void pollRadar() {
  static uint8_t buffer[64], used = 0;
  while (Serial2.available()) {
    uint8_t v = (uint8_t)Serial2.read();
    if (used < 4) {
      if (v == RD03D_FRAME_HEADER[used])
        buffer[used++] = v;
      else if (v == RD03D_FRAME_HEADER[0]) {
        buffer[0] = v;
        used = 1;
      } else
        used = 0;
      continue;
    }
    buffer[used++] = v;
    if (used >= sizeof(buffer)) {
      used = 0;
      continue;
    }
    if (used < 30 || buffer[used - 2] != 0x55 || buffer[used - 1] != 0xCC)
      continue;
    ++radarFrames;
    lastRadarTime = millis();
    radarTargetCount = 0;
    float best = 0;
    int16_t bestSpeed = 0;
    for (uint8_t i = 0; i < RD03D_MAX_TARGETS; ++i) {
      uint8_t o = 4 + i * RD03D_TARGET_DATA_SIZE;
      int16_t x = rd03dDecodeValue(buffer[o], buffer[o + 1]),
              y = rd03dDecodeValue(buffer[o + 2], buffer[o + 3]),
              s = rd03dDecodeValue(buffer[o + 4], buffer[o + 5]);
      if ((x == 0 && y == 0) || !rd03dSpeedValid(s))
        continue;
      ++radarTargetCount;
      float d = sqrtf((float)x * x + (float)y * y);
      if (d > best) {
        best = d;
        bestSpeed = s;
      }
    }
    radarPresence = radarTargetCount > 0;
    radarDistM = best / 1000.0f;
    radarSpeed = bestSpeed;
    used = 0;
  }
}

void pollDebugConsole() {
  while (Serial.available()) {
    const char command = (char)Serial.read();
    if (command == 'b' || command == 'B') {
      startBuzzerSelfTest();
    } else if (command >= '1' && command <= '5') {
      const uint8_t track = (uint8_t)(command - '0');
      Serial.printf("[TEST] Requested audio track %u\n", track);
      queueAudio(track);
    } else if (command == 'h' || command == 'H' || command == '?') {
      Serial.println("[TEST] Commands: B=buzzer self-test, 1..5=queue MP3 track");
    }
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(115200);
  delay(700);
  Serial.println("\n--- RANGER FUSION " VERSION " ---");
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(PIN_RELAY, LOW);
  setRGB(0, 0, 0);
  Serial1.setRX(PIN_BASE_RX);
  Serial1.setTX(PIN_BASE_TX);
  Serial1.begin(115200);
  Serial1.setTimeout(10);
  Serial2.setTX(PIN_RADAR_TX);
  Serial2.setRX(PIN_RADAR_RX);
  Serial2.begin(256000);
  publishTelemetry();
  Wire1.setSDA(PIN_I2C_SDA);
  Wire1.setSCL(PIN_I2C_SCL);
  Wire1.begin(RangerLink::FUSION_I2C_ADDRESS);
  Wire1.onRequest(onI2CRequest);
  Wire1.onReceive(onI2CReceive);
  Serial.println("[LINK] I2C1 slave 0x42 ready on GP2/GP3, protocol v3");
  dfpSerial.begin(9600);
  delay(1000);
  // Use no-ACK mode so missing/corrupt feedback can never stall sensor
  // polling. Playback-finished events are still consumed when GP9 is wired;
  // conservative timeouts sequence clips correctly on TX-only installations.
  if (myDFPlayer.begin(dfpSerial, false, true)) {
    dfpReady = true;
    myDFPlayer.setTimeOut(250);
    myDFPlayer.volume(24);
    queueAudio(AUDIO_BOOT);
    queueAudio(AUDIO_READY);
    Serial.println("[AUDIO] DFPlayer configured; ordered startup cues queued");
  } else
    Serial.println("[AUDIO] DFPlayer setup failed; continuing without voice");
  startBuzzerSelfTest();
  publishTelemetry();
  Serial.println("[BOOT] Setup complete; send H over USB serial for test commands");
}
void loop() {
  uint32_t now = millis();
  handleI2CCommands();
  pollBase();
  pollRadar();
  serviceFallVerification(now);
  if (wearableLinked && !baseFresh(now)) {
    wearableLinked = false;
    Serial.println("[BASE] Wearable timed out");
  }
  if (!radarFresh(now) && radarPresence) {
    radarPresence = false;
    radarTargetCount = 0;
    radarDistM = 0;
    radarSpeed = 0;
  }
  processActuators();
  relayTick();
  pollDebugConsole();
  static uint32_t lastPublish = 0;
  if (now - lastPublish >= TELEMETRY_PERIOD_MS) {
    lastPublish = now;
    publishTelemetry();
  }
  static uint32_t lastStats = 0;
  if (now - lastStats >= 2000) {
    lastStats = now;
    uint32_t rq, cmd, err;
    noInterrupts();
    rq = i2cRequestCount;
    cmd = i2cCommandCount;
    err = i2cCommandErrorCount;
    interrupts();
    Serial.printf("[STATS] seq=%u i2c=%lu cmd=%lu/%lu base=%d imu=%lu env=%lu "
                  "ml=%lu parse=%lu radar=%lu targets=%u fall=%d source=%s "
                  "peak=%.2fg impactCfg=%d stillCfg=%d relay=%d buzzer=%d "
                  "audio=%u queue=%u\n",
                  telemetrySequence, rq, cmd, err, baseFresh(now), imuPackets,
                  envPackets, mlPackets, baseParseErrors, radarFrames,
                  radarTargetCount, fallDetected,
                  fallSourceName(fallTriggerSource), candidatePeakG,
                  impactTriggerEnabled, stillnessCheckEnabled,
                  relayActive, buzzerActive, audioCurrentTrack,
                  (unsigned)audioQueueCount);
  }
  static uint32_t lastBlink = 0;
  if (now - lastBlink >= 500) {
    lastBlink = now;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}
