/**
 * RANGER v5.0 — CrowPanel Advance 4.3" V1.0
 *
 * A touch-first safety dashboard and diagnostic terminal. Telemetry comes
 * from the RP2040 Fusion Node over the shared GPIO15/16 I2C bus at 0x42.
 * Protocol v3 provides magic/version/size checks, sequence tracking, CRC16,
 * and synchronized fall-policy commands.
 *
 * Board facts verified against Elecrow's V1.0 schematic/example:
 *   GPIO8: passive buzzer (direct GPIO)       GPIO6: I2S LRCLK (not a button)
 *   I2C: GPIO15 SDA / GPIO16 SCL              TCA9534: 0x18
 *   GT911: 0x5D                               Fusion Pico: 0x42
 */
#include "RangerLinkProtocol.h"
#include "gfx_conf.h"
#include "secrets.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <HTTPClient.h>
#include <TCA9534.h>
#include <WiFi.h>
#include <Wire.h>

#define VERSION "5.8-CROW-DIRECT-ALERTS"
#define BUILD_DATE "2026-09-14"

constexpr uint8_t PIN_I2C_SDA = 15, PIN_I2C_SCL = 16, PIN_TOUCH_INT = 1,
                  PIN_BUZZER = 8;
constexpr uint32_t LINK_STALE_MS = 1500, LINK_FROZEN_MS = 1200,
                   POLL_PERIOD_MS = 250,
                   FALL_SETTINGS_SYNC_RETRY_MS = 5000;
constexpr uint16_t SCR_W = DISPLAY_WIDTH, SCR_H = DISPLAY_HEIGHT;

static LGFX lcd;
static LGFX_Sprite canvas(&lcd);
// Retained as unallocated legacy render targets while the classic drawing
// helpers remain available for comparison. v5.4 draws the live dashboard
// directly to the RGB framebuffer and never calls their pushSprite paths.
static LGFX_Sprite headerLayer(&lcd), statusLayer(&lcd), radarLayer(&lcd),
    motionLayer(&lcd), footerLayer(&lcd);
static LGFX_Sprite motionGraphLayer(&lcd);
bool motionGraphLayerReady = false;
TCA9534 ioex;

// Original high-contrast RANGER palette, retained for the 800x480 rebuild.
constexpr uint16_t C_BG = 0x0841, C_SURFACE = 0x18C3, C_CARD = 0x1082,
                   C_LINE = 0x3186;
constexpr uint16_t C_TEXT = 0xC618, C_MUTED = 0x630C, C_WHITE = 0xFFFF,
                   C_CYAN = 0x07FF;
constexpr uint16_t C_GREEN = 0x07E0, C_RED = 0xF800, C_YELLOW = 0xFFE0,
                   C_ORANGE = 0xFD20;
constexpr uint16_t C_BLUE = 0x2C7F, C_MAGENTA = 0xF29F,
                   C_SECURE = 0x2E8B;

// ============== SETTINGS ==============
constexpr size_t EEPROM_SIZE = 16;
bool muted = false, radarEnabled = true, motionEnabled = true, langTR = false;
bool impactTriggerEnabled = false, stillnessCheckEnabled = true;
uint8_t themeIndex = 0;

uint16_t accent() {
  const uint16_t colors[] = {C_CYAN,   C_GREEN,  C_MAGENTA,
                             C_YELLOW, C_ORANGE, C_BLUE};
  return colors[themeIndex % 6];
}
void saveSettings() {
  EEPROM.write(0, muted);
  EEPROM.write(1, langTR);
  EEPROM.write(2, radarEnabled);
  EEPROM.write(3, motionEnabled);
  EEPROM.write(4, themeIndex);
  EEPROM.write(5, impactTriggerEnabled);
  EEPROM.write(6, stillnessCheckEnabled);
  EEPROM.commit();
}
void loadSettings() {
  if (EEPROM.read(0) == 0xFF)
    return;
  muted = EEPROM.read(0) == 1;
  langTR = EEPROM.read(1) == 1;
  radarEnabled = EEPROM.read(2) == 1;
  motionEnabled = EEPROM.read(3) == 1;
  themeIndex = EEPROM.read(4) % 6;
  // Bytes 5 and 6 did not exist in v5.5. Preserve safe defaults when a panel
  // upgrades with erased (0xFF) values in those slots.
  if (EEPROM.read(5) != 0xFF)
    impactTriggerEnabled = EEPROM.read(5) == 1;
  if (EEPROM.read(6) != 0xFF)
    stillnessCheckEnabled = EEPROM.read(6) == 1;
}

// ============== FUSION LINK ==============
enum LinkError : uint8_t {
  LINK_OK,
  LINK_NO_RESPONSE,
  LINK_SHORT_PACKET,
  LINK_BAD_MAGIC,
  LINK_BAD_VERSION,
  LINK_BAD_SIZE,
  LINK_BAD_CRC,
  LINK_FROZEN
};
struct LinkStats {
  uint32_t attempts = 0, valid = 0, noResponse = 0, shortPackets = 0,
           badMagic = 0;
  uint32_t badVersion = 0, badSize = 0, badCrc = 0, sequenceGaps = 0,
           duplicates = 0;
  uint16_t lastSequence = 0;
  bool haveSequence = false;
  uint32_t lastValidMs = 0, lastAdvanceMs = 0;
  uint16_t consecutiveFailures = 0;
  LinkError error = LINK_NO_RESPONSE;
} linkStats;

RangerLink::TelemetryPacket telem{};
bool fusionOnline = false;
bool fallSettingsSynced = false;
uint32_t lastFallSettingsSyncAttemptMs = 0;
constexpr size_t MOTION_HISTORY_SIZE = 112;
constexpr size_t MOTION_RENDER_POINTS = 32;
float motionHistory[MOTION_HISTORY_SIZE] = {};
size_t motionHistoryHead = 0, motionHistoryCount = 0;

void recordMotionSample(int16_t rawX, int16_t rawY, int16_t rawZ) {
  const float x = rawX / 4096.0f;
  const float y = rawY / 4096.0f;
  const float z = rawZ / 4096.0f;
  motionHistory[motionHistoryHead] = sqrtf(x * x + y * y + z * z);
  motionHistoryHead = (motionHistoryHead + 1) % MOTION_HISTORY_SIZE;
  if (motionHistoryCount < MOTION_HISTORY_SIZE)
    ++motionHistoryCount;
}

const char *linkErrorName(uint8_t e) {
  switch (e) {
  case LINK_OK:
    return "OK";
  case LINK_NO_RESPONSE:
    return "NO RESPONSE";
  case LINK_SHORT_PACKET:
    return "SHORT READ";
  case LINK_BAD_MAGIC:
    return "BAD MAGIC";
  case LINK_BAD_VERSION:
    return "VERSION MISMATCH";
  case LINK_BAD_SIZE:
    return "SIZE MISMATCH";
  case LINK_BAD_CRC:
    return "CRC ERROR";
  case LINK_FROZEN:
    return "PICO FROZEN";
  }
  return "UNKNOWN";
}

void linkFailure(uint8_t error) {
  linkStats.error = static_cast<LinkError>(error);
  ++linkStats.consecutiveFailures;
  if (error == LINK_NO_RESPONSE)
    ++linkStats.noResponse;
  else if (error == LINK_SHORT_PACKET)
    ++linkStats.shortPackets;
  else if (error == LINK_BAD_MAGIC)
    ++linkStats.badMagic;
  else if (error == LINK_BAD_VERSION)
    ++linkStats.badVersion;
  else if (error == LINK_BAD_SIZE)
    ++linkStats.badSize;
  else if (error == LINK_BAD_CRC)
    ++linkStats.badCrc;
}

bool pollFusion() {
  ++linkStats.attempts;
  uint8_t bytes[sizeof(RangerLink::TelemetryPacket)] = {};
  const int got = Wire.requestFrom((uint8_t)RangerLink::FUSION_I2C_ADDRESS,
                                   (uint8_t)sizeof(bytes), (uint8_t)true);
  if (got == 0) {
    linkFailure(LINK_NO_RESPONSE);
    return false;
  }
  if (got != (int)sizeof(bytes)) {
    while (Wire.available())
      Wire.read();
    linkFailure(LINK_SHORT_PACKET);
    return false;
  }
  for (size_t i = 0; i < sizeof(bytes); ++i)
    bytes[i] = (uint8_t)Wire.read();
  RangerLink::TelemetryPacket candidate{};
  memcpy(&candidate, bytes, sizeof(candidate));
  if (candidate.magic != RangerLink::TELEMETRY_MAGIC) {
    linkFailure(LINK_BAD_MAGIC);
    return false;
  }
  if (candidate.protocolVersion != RangerLink::PROTOCOL_VERSION) {
    linkFailure(LINK_BAD_VERSION);
    return false;
  }
  if (candidate.packetSize != sizeof(candidate)) {
    linkFailure(LINK_BAD_SIZE);
    return false;
  }
  if (!RangerLink::validateTelemetry(candidate)) {
    linkFailure(LINK_BAD_CRC);
    return false;
  }

  const uint32_t now = millis();
  if (linkStats.haveSequence) {
    const uint16_t delta =
        (uint16_t)(candidate.sequence - linkStats.lastSequence);
    if (delta == 0)
      ++linkStats.duplicates;
    else {
      if (delta > 1 && delta < 0x8000)
        linkStats.sequenceGaps += delta - 1;
      linkStats.lastAdvanceMs = now;
      linkStats.lastSequence = candidate.sequence;
      recordMotionSample(candidate.ax, candidate.ay, candidate.az);
    }
  } else {
    linkStats.haveSequence = true;
    linkStats.lastSequence = candidate.sequence;
    linkStats.lastAdvanceMs = now;
    recordMotionSample(candidate.ax, candidate.ay, candidate.az);
  }
  telem = candidate;
  ++linkStats.valid;
  linkStats.lastValidMs = now;
  linkStats.consecutiveFailures = 0;
  linkStats.error =
      (now - linkStats.lastAdvanceMs > LINK_FROZEN_MS) ? LINK_FROZEN : LINK_OK;
  fusionOnline = (linkStats.error != LINK_FROZEN);
  return true;
}

bool sendFusionCommand(RangerLink::Command command) {
  const RangerLink::CommandPacket packet = RangerLink::makeCommand(command);
  Wire.beginTransmission(RangerLink::FUSION_I2C_ADDRESS);
  Wire.write((const uint8_t *)&packet, sizeof(packet));
  const uint8_t result = Wire.endTransmission(true);
  Serial0.printf("[LINK] Command 0x%02X result=%u\n", (uint8_t)command, result);
  return result == 0;
}

void syncFallSettings() {
  lastFallSettingsSyncAttemptMs = millis();
  const bool impactOk = sendFusionCommand(
      impactTriggerEnabled ? RangerLink::CMD_IMPACT_ENABLE
                           : RangerLink::CMD_IMPACT_DISABLE);
  const bool stillnessOk = sendFusionCommand(
      stillnessCheckEnabled ? RangerLink::CMD_STILLNESS_ENABLE
                            : RangerLink::CMD_STILLNESS_DISABLE);
  fallSettingsSynced = impactOk && stillnessOk;
  Serial0.printf("[FALL CFG] impact=%d stillness=%d synced=%d\n",
                 impactTriggerEnabled, stillnessCheckEnabled,
                 fallSettingsSynced);
}

float toG(int16_t raw) { return raw / 4096.0f; }
float toDps(int16_t raw) { return raw / 65.5f; }
bool flag(uint8_t value) { return (telem.flags & value) != 0; }

// ============== TOUCH ==============
int16_t touchX = -1, touchY = -1;
bool touchDown = false, touchJustPressed = false, touchJustReleased = false;
void pollTouch() {
  lgfx::touch_point_t point;
  const bool nowDown = lcd.getTouch(&point, 1) > 0;
  touchJustPressed = nowDown && !touchDown;
  touchJustReleased = !nowDown && touchDown;
  touchDown = nowDown;
  if (nowDown) {
    touchX = point.x;
    touchY = point.y;
  }
}
bool hit(int x, int y, int w, int h) {
  return touchJustPressed && touchX >= x && touchX < x + w && touchY >= y &&
         touchY < y + h;
}

// ============== NETWORK ==============
volatile bool triggerTelegram = false;
volatile float telegramImpact = 0;
String urlEncode(const String &input) {
  String out;
  for (size_t i = 0; i < input.length(); ++i) {
    char c = input[i];
    if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~')
      out += c;
    else if (c == ' ')
      out += '+';
    else {
      char b[4];
      snprintf(b, sizeof(b), "%%%02X", (uint8_t)c);
      out += b;
    }
  }
  return out;
}
void networkTask(void *) {
  for (;;) {
    if (!triggerTelegram) {
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

    triggerTelegram = false;
    const float impact = telegramImpact;
    if (strlen(WIFI_SSID) == 0 || strlen(TELEGRAM_TOKEN) <= 8 ||
        strlen(TELEGRAM_CHAT_ID) == 0) {
      Serial0.println("[TELEGRAM] Credentials not configured; WiFi stays off");
      continue;
    }

    // RGB scan DMA and WiFi both consume the ESP32-S3 external-memory path.
    // Keep the radio fully off during normal monitoring, then connect only for
    // an emergency message. Power-save remains enabled during that short burst.
    Serial0.println("[WIFI] Emergency on-demand connection...");
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(true);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    const uint32_t connectStarted = millis();
    while (WiFi.status() != WL_CONNECTED &&
           millis() - connectStarted < 15000)
      vTaskDelay(pdMS_TO_TICKS(100));

    if (WiFi.status() == WL_CONNECTED) {
      String message = langTR ? "RANGER: Onaylanmis dusme. Ivme: "
                              : "RANGER: Verified fall. Impact: ";
      message += String(impact, 1) + "g";
      HTTPClient http;
      http.setTimeout(6000);
      const String url =
          "https://api.telegram.org/bot" + String(TELEGRAM_TOKEN) +
          "/sendMessage?chat_id=" + String(TELEGRAM_CHAT_ID) +
          "&text=" + urlEncode(message);
      http.begin(url);
      const int code = http.GET();
      http.end();
      Serial0.printf("[TELEGRAM] HTTP %d\n", code);
    } else {
      Serial0.println("[TELEGRAM] WiFi connection timed out");
    }
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    Serial0.println("[WIFI] Radio off; display bus released");
  }
}

// ============== UI ==============
enum Page : uint8_t {
  PAGE_DASHBOARD,
  PAGE_SETTINGS,
  PAGE_ALERT,
  PAGE_DANGER,
  PAGE_CLEARED
};
constexpr uint32_t ALERT_ESCALATE_MS = 30000;
Page page = PAGE_DASHBOARD;
uint32_t pageStartMs = 0;
bool alertActive = false;
float alertImpact = 0;
bool alertToneOn = false;
uint32_t lastToneChangeMs = 0;
uint32_t alertSuppressedUntilMs = 0;
bool dashboardBackgroundDirty = true;
bool settingsDirty = true;
bool emergencyPageDirty = true;

void setBuzzer(bool on) {
  if (on && !muted)
    tone(PIN_BUZZER, 2300);
  else
    noTone(PIN_BUZZER);
  alertToneOn = on && !muted;
}
void box(int x, int y, int w, int h, uint16_t fill, uint16_t border = C_LINE) {
  canvas.fillRoundRect(x, y, w, h, 10, fill);
  canvas.drawRoundRect(x, y, w, h, 10, border);
}
void text(const String &value, int x, int y, uint8_t size, uint16_t color) {
  canvas.setTextSize(size);
  canvas.setTextColor(color);
  canvas.setCursor(x, y);
  canvas.print(value);
}
void centered(const String &value, int centerX, int y, uint8_t size,
              uint16_t color) {
  canvas.setTextSize(size);
  canvas.setTextColor(color);
  canvas.setCursor(centerX - canvas.textWidth(value) / 2, y);
  canvas.print(value);
}
void pill(int x, int y, int w, const char *label, bool ok) {
  uint16_t color = ok ? C_GREEN : C_RED;
  canvas.fillRoundRect(x, y, w, 25, 12, color);
  centered(label, x + w / 2, y + 5, 1, C_BG);
}
void drawHeader(const char *title) {
  canvas.fillRect(0, 0, SCR_W, 58, C_SURFACE);
  canvas.fillRect(0, 56, SCR_W, 2, accent());
  text("RANGER", 18, 11, 3, C_WHITE);
  text(title, 174, 19, 2, C_MUTED);
  char uptime[20];
  uint32_t s = millis() / 1000;
  snprintf(uptime, sizeof(uptime), "%02lu:%02lu:%02lu", s / 3600, (s / 60) % 60,
           s % 60);
  text(uptime, 610, 20, 2, C_TEXT);
}

void layerText(LGFX_Sprite &layer, const String &value, int x, int y,
               uint8_t size, uint16_t color) {
  layer.setTextSize(size);
  layer.setTextColor(color);
  layer.setCursor(x, y);
  layer.print(value);
}

void layerCentered(LGFX_Sprite &layer, const String &value, int centerX, int y,
                   uint8_t size, uint16_t color) {
  layer.setTextSize(size);
  layer.setTextColor(color);
  layer.setCursor(centerX - layer.textWidth(value) / 2, y);
  layer.print(value);
}

void drawMotionHistoryOn(LGFX_Sprite &layer, int x, int y, int w, int h) {
  layer.drawRect(x, y, w, h, C_LINE);
  layer.drawFastHLine(x, y + h / 2, w, C_LINE);
  if (motionHistoryCount < 2)
    return;
  const size_t oldest =
      (motionHistoryHead + MOTION_HISTORY_SIZE - motionHistoryCount) %
      MOTION_HISTORY_SIZE;
  const size_t renderCount = min(motionHistoryCount, MOTION_RENDER_POINTS);
  int previousX = x;
  float first = motionHistory[oldest];
  if (first > 4.0f)
    first = 4.0f;
  int previousY = y + h - 2 - (int)(first * (h - 4) / 4.0f);
  for (size_t i = 1; i < renderCount; ++i) {
    const size_t sourceOffset =
        i * (motionHistoryCount - 1) / (renderCount - 1);
    const size_t index = (oldest + sourceOffset) % MOTION_HISTORY_SIZE;
    float value = motionHistory[index];
    if (value > 4.0f)
      value = 4.0f;
    const int currentX = x + (int)(i * (w - 1) / (renderCount - 1));
    const int currentY = y + h - 2 - (int)(value * (h - 4) / 4.0f);
    layer.drawLine(previousX, previousY, currentX, currentY,
                   value > 2.5f ? C_RED : accent());
    previousX = currentX;
    previousY = currentY;
  }
}

void drawRadarScopeOn(LGFX_Sprite &layer, int centerX, int centerY,
                      int radius) {
  layer.drawCircle(centerX, centerY, radius, C_LINE);
  layer.drawCircle(centerX, centerY, radius * 2 / 3, C_LINE);
  layer.drawCircle(centerX, centerY, radius / 3, C_LINE);
  layer.drawFastVLine(centerX, centerY - radius, radius * 2, C_LINE);
  layer.drawFastHLine(centerX - radius, centerY, radius * 2, C_LINE);
  if (telem.radarTargets == 0)
    return;
  float distance = telem.radarDistanceMm / 1000.0f;
  if (distance < 0)
    distance = 0;
  if (distance > 5.0f)
    distance = 5.0f;
  const int targetY = centerY - (int)(distance / 5.0f * (radius - 5));
  layer.fillCircle(centerX, targetY, 6, C_YELLOW);
  layer.drawCircle(centerX, targetY, 10, C_ORANGE);
}
String fusionStatus() {
  if (!fusionOnline)
    return langTR ? "FUSION BAGLANTISI YOK" : "FUSION LINK OFFLINE";
  if (telem.fallState == 2 || flag(RangerLink::FLAG_SOS_ACTIVE))
    return langTR ? "ACIL DURUM" : "EMERGENCY ACTIVE";
  if (telem.fallState == 1)
    return langTR ? "DUSME DOGRULANIYOR" : "VERIFYING POSSIBLE FALL";
  if (!flag(RangerLink::FLAG_WEARABLE_LINK))
    return langTR ? "GIYILEBILIR ARANIYOR" : "WAITING FOR WEARABLE";
  return langTR ? "SISTEM NORMAL" : "SYSTEM NOMINAL";
}
uint16_t fusionStatusColor() {
  if (!fusionOnline)
    return C_RED;
  if (telem.fallState == 2 || flag(RangerLink::FLAG_SOS_ACTIVE))
    return C_RED;
  if (telem.fallState == 1 || !flag(RangerLink::FLAG_WEARABLE_LINK))
    return C_YELLOW;
  return C_GREEN;
}

void drawDashboardBackground() {
  lcd.fillScreen(C_BG);
  lcd.drawRoundRect(10, 55, 380, 180, 6, C_LINE);
  lcd.drawRoundRect(410, 55, 380, 180, 6, C_LINE);
  lcd.drawRoundRect(10, 250, 780, 190, 6, C_LINE);
  lcd.setTextSize(2);
  lcd.setTextColor(accent(), C_BG);
  lcd.setCursor(25, 62);
  lcd.print(langTR ? "SISTEM DURUMU" : "SYSTEM STATUS");
  lcd.setCursor(425, 62);
  lcd.print(langTR ? "RADAR IZLEME" : "RADAR TRACKING");
  lcd.setCursor(25, 257);
  lcd.print(langTR ? "HAREKET ANALIZI" : "MOTION ANALYSIS");
  lcd.drawFastHLine(15, 82, 370, C_LINE);
  lcd.drawFastHLine(415, 82, 370, C_LINE);
  lcd.drawFastHLine(15, 277, 770, C_LINE);
}

void drawHeaderIndicator(int x, int width, const char *label, bool ok) {
  const uint16_t fill = ok ? C_GREEN : C_CARD;
  const uint16_t border = ok ? C_GREEN : C_LINE;
  headerLayer.fillRoundRect(x, 7, width, 22, 4, fill);
  headerLayer.drawRoundRect(x, 7, width, 22, 4, border);
  layerCentered(headerLayer, label, x + width / 2, 14, 1,
                ok ? C_BG : C_MUTED);
}

void drawDashboardHeader() {
  headerLayer.fillSprite(C_SURFACE);
  headerLayer.drawFastHLine(0, 49, SCR_W, accent());
  layerText(headerLayer, langTR ? "RANGER IZLEME" : "RANGER MONITOR", 15, 10,
            3, accent());

  const bool wearable = fusionOnline && flag(RangerLink::FLAG_WEARABLE_LINK);
  const bool radar = fusionOnline && flag(RangerLink::FLAG_RADAR_FRESH);
  drawHeaderIndicator(480, 54, "WEAR", wearable);
  drawHeaderIndicator(540, 54, "PICO", fusionOnline);
  drawHeaderIndicator(600, 48, "RAD", radar);

  char battery[12];
  snprintf(battery, sizeof(battery), fusionOnline ? "%u%%" : "--%%",
           telem.battery);
  drawHeaderIndicator(654, 62, battery, fusionOnline && telem.battery > 20);
  headerLayer.fillRoundRect(738, 7, 52, 32, 4, C_CARD);
  headerLayer.drawRoundRect(738, 7, 52, 32, 4, accent());
  layerCentered(headerLayer, "SET", 764, 17, 1, C_WHITE);

  char detail[96];
  const uint32_t seconds = millis() / 1000;
  snprintf(detail, sizeof(detail), "%s  UP %02lu:%02lu:%02lu", VERSION,
           seconds / 3600, (seconds / 60) % 60, seconds % 60);
  layerText(headerLayer, detail, 18, 38, 1, C_MUTED);
  if (!fusionOnline)
    layerText(headerLayer, linkErrorName(linkStats.error), 300, 38, 1, C_RED);
  headerLayer.pushSprite(0, 0);
}

void drawSystemStatusPanel() {
  statusLayer.fillSprite(C_BG);
  const bool wearable = fusionOnline && flag(RangerLink::FLAG_WEARABLE_LINK);
  const bool sos = fusionOnline && flag(RangerLink::FLAG_SOS_ACTIVE);
  const char *statusText;
  uint16_t badgeBackground, badgeText;
  if (!fusionOnline) {
    statusText = langTR ? "PICO BEKLENIYOR" : "WAITING FOR PICO";
    badgeBackground = C_CARD;
    badgeText = C_RED;
  } else if (sos || telem.fallState == 2) {
    statusText = sos ? "SOS!" : (langTR ? "DUSME!" : "FALL!");
    badgeBackground = C_RED;
    badgeText = C_WHITE;
  } else if (telem.fallState == 1) {
    statusText = langTR ? "DOGRULANIYOR" : "VERIFYING";
    badgeBackground = C_ORANGE;
    badgeText = C_BG;
  } else if (!wearable) {
    statusText = langTR ? "SINYAL YOK" : "NO WEARABLE";
    badgeBackground = C_CARD;
    badgeText = C_YELLOW;
  } else {
    statusText = langTR ? "GUVENLI" : "SECURE";
    badgeBackground = C_SECURE;
    badgeText = C_GREEN;
  }

  statusLayer.fillRoundRect(15, 10, 340, 50, 8, badgeBackground);
  statusLayer.drawRoundRect(15, 10, 340, 50, 8, badgeText);
  layerCentered(statusLayer, statusText, 185, 22, 3, badgeText);

  char row[64];
  if (fusionOnline)
    snprintf(row, sizeof(row), "BAT: %3u%%", telem.battery);
  else
    snprintf(row, sizeof(row), "BAT:  --%%");
  layerText(statusLayer, row, 20, 75, 2, C_TEXT);
  statusLayer.drawRect(195, 77, 140, 16, C_LINE);
  int batteryFill = fusionOnline ? constrain((int)telem.battery, 0, 100) * 136 / 100 : 0;
  if (batteryFill)
    statusLayer.fillRect(197, 79, batteryFill, 12,
                         telem.battery > 50 ? C_GREEN
                         : telem.battery > 20 ? C_YELLOW
                                              : C_RED);

  snprintf(row, sizeof(row), "ML FALL %3u%%   CONF %3u%%", telem.mlFall,
           telem.mlConfidence);
  layerText(statusLayer, row, 20, 105, 1, fusionOnline ? C_TEXT : C_MUTED);
  const uint32_t quality =
      linkStats.attempts ? linkStats.valid * 100UL / linkStats.attempts : 0;
  snprintf(row, sizeof(row), "LINK %-13s Q %3lu%%", linkErrorName(linkStats.error),
           quality);
  layerText(statusLayer, row, 20, 123, 1,
            fusionOnline ? C_GREEN : C_RED);
  statusLayer.pushSprite(15, 85);
}

void drawRadarPanelClassic() {
  radarLayer.fillSprite(C_BG);
  const bool fresh = fusionOnline && radarEnabled &&
                     flag(RangerLink::FLAG_RADAR_FRESH);
  const bool target = fresh && telem.radarTargets > 0;
  const char *label = !radarEnabled
                          ? (langTR ? "KAPALI" : "DISABLED")
                          : !fusionOnline ? "OFFLINE"
                          : !fresh ? (langTR ? "VERI YOK" : "NO DATA")
                          : target ? (langTR ? "HEDEF" : "TARGET")
                                   : (langTR ? "TARANIYOR" : "SCANNING");
  const uint16_t color = target ? C_YELLOW : fresh ? accent() : C_MUTED;
  radarLayer.fillRoundRect(15, 10, 340, 50, 8, target ? C_CARD : C_BG);
  radarLayer.drawRoundRect(15, 10, 340, 50, 8, color);
  layerCentered(radarLayer, label, 185, 22, 3, color);

  char row[48];
  if (fresh) {
    snprintf(row, sizeof(row), "DIST: %.2f m",
             telem.radarDistanceMm / 1000.0f);
    layerText(radarLayer, row, 20, 77, 2, C_WHITE);
    snprintf(row, sizeof(row), "SPD:  %+d cm/s", telem.radarSpeedCms);
    layerText(radarLayer, row, 20, 103, 2, C_TEXT);
    snprintf(row, sizeof(row), "TARGETS: %u", telem.radarTargets);
    layerText(radarLayer, row, 20, 130, 1, C_MUTED);
    drawRadarScopeOn(radarLayer, 302, 103, 36);
  } else {
    layerCentered(radarLayer,
                  radarEnabled ? "Waiting for fresh radar frames"
                               : "Enable in Settings",
                  185, 92, 1, C_MUTED);
  }
  radarLayer.pushSprite(415, 85);
}

void drawMotionPanelClassic() {
  motionLayer.fillSprite(C_BG);
  if (!fusionOnline || !motionEnabled) {
    layerCentered(motionLayer,
                  motionEnabled ? "Waiting for Fusion Pico telemetry"
                                : "Motion panel disabled in Settings",
                  385, 62, 2, C_MUTED);
    motionLayer.pushSprite(15, 280);
    return;
  }

  const float axes[3] = {toG(telem.ax), toG(telem.ay), toG(telem.az)};
  const char *labels[3] = {"X-AXIS", "Y-AXIS", "Z-AXIS"};
  for (int i = 0; i < 3; ++i) {
    const int y = 13 + i * 36;
    layerText(motionLayer, labels[i], 15, y, 2, C_TEXT);
    const int barX = 100, barW = 180, centerX = barX + barW / 2;
    motionLayer.drawRect(barX, y + 2, barW, 14, C_LINE);
    motionLayer.drawFastVLine(centerX, y + 3, 12, C_MUTED);
    const int extent = (int)(constrain(axes[i], -2.0f, 2.0f) * (barW / 4));
    const uint16_t barColor = fabsf(axes[i]) > 1.5f ? C_ORANGE : accent();
    if (extent > 0)
      motionLayer.fillRect(centerX, y + 4, extent, 10, barColor);
    else if (extent < 0)
      motionLayer.fillRect(centerX + extent, y + 4, -extent, 10, barColor);
    char value[16];
    snprintf(value, sizeof(value), "%+.2fg", axes[i]);
    layerText(motionLayer, value, 292, y + 1, 2, C_WHITE);
  }

  char row[64];
  snprintf(row, sizeof(row), "GYRO  X%+5.0f  Y%+5.0f  Z%+5.0f dps",
           toDps(telem.gx), toDps(telem.gy), toDps(telem.gz));
  layerText(motionLayer, row, 390, 14, 1, C_MUTED);
  const float magnitude =
      sqrtf(axes[0] * axes[0] + axes[1] * axes[1] + axes[2] * axes[2]);
  snprintf(row, sizeof(row), "|G| %.2f", magnitude);
  layerText(motionLayer, row, 390, 38, 2,
            magnitude > 2.5f ? C_RED : accent());
  snprintf(row, sizeof(row), "ML %u%%  CONF %u%%", telem.mlFall,
           telem.mlConfidence);
  layerText(motionLayer, row, 555, 42, 1, C_TEXT);
  layerText(motionLayer,
            flag(RangerLink::FLAG_BASE_FRESH) ? "BASE STREAM FRESH"
                                              : "BASE STREAM STALE",
            390, 68, 1,
            flag(RangerLink::FLAG_BASE_FRESH) ? C_GREEN : C_YELLOW);
  drawMotionHistoryOn(motionLayer, 390, 86, 350, 52);
  motionLayer.pushSprite(15, 280);
}

void drawFooterChip(int x, int width, const char *label, bool ok,
                    uint16_t activeColor = C_GREEN) {
  footerLayer.fillRoundRect(x, 5, width, 24, 4, ok ? activeColor : C_CARD);
  footerLayer.drawRoundRect(x, 5, width, 24, 4, ok ? activeColor : C_LINE);
  layerCentered(footerLayer, label, x + width / 2, 12, 1,
                ok ? C_BG : C_MUTED);
}

void drawDashboardFooter() {
  footerLayer.fillSprite(C_SURFACE);
  footerLayer.drawFastHLine(0, 0, SCR_W, C_LINE);
  drawFooterChip(15, 74, "WEAR", fusionOnline &&
                                      flag(RangerLink::FLAG_WEARABLE_LINK));
  drawFooterChip(99, 68, "RAD", fusionOnline &&
                                    flag(RangerLink::FLAG_RADAR_FRESH),
                 accent());
  drawFooterChip(177, 74, "PICO", fusionOnline);
  drawFooterChip(261, 78, "CAM", fusionOnline &&
                                      flag(RangerLink::FLAG_RELAY_ACTIVE),
                 C_ORANGE);
  drawFooterChip(349, 74, "WIFI", WiFi.status() == WL_CONNECTED);
  char details[100];
  if (fusionOnline)
    snprintf(details, sizeof(details), "SEQ %u  CRC %lu  GAP %lu  PICO %us",
             linkStats.lastSequence, linkStats.badCrc, linkStats.sequenceGaps,
             telem.uptimeSeconds);
  else
    snprintf(details, sizeof(details), "SDA15-GP2  SCL16-GP3  GND-GND");
  layerText(footerLayer, details, 438, 13, 1,
            fusionOnline ? C_MUTED : C_YELLOW);
  footerLayer.pushSprite(0, 445);
}

void lcdText(const String &value, int x, int y, uint8_t size, uint16_t color) {
  lcd.setTextSize(size);
  lcd.setTextColor(color);
  lcd.setCursor(x, y);
  lcd.print(value);
}

// Avoid constructing temporary Arduino String objects during periodic direct
// rendering. Long-running heap churn can fragment memory and make redraw
// timing progressively less predictable.
void lcdText(const char *value, int x, int y, uint8_t size, uint16_t color) {
  lcd.setTextSize(size);
  lcd.setTextColor(color);
  lcd.setCursor(x, y);
  lcd.print(value);
}

void lcdCentered(const String &value, int centerX, int y, uint8_t size,
                 uint16_t color) {
  lcd.setTextSize(size);
  lcd.setTextColor(color);
  lcd.setCursor(centerX - lcd.textWidth(value) / 2, y);
  lcd.print(value);
}

void lcdCentered(const char *value, int centerX, int y, uint8_t size,
                 uint16_t color) {
  lcd.setTextSize(size);
  lcd.setTextColor(color);
  lcd.setCursor(centerX - lcd.textWidth(value) / 2, y);
  lcd.print(value);
}

void drawDirectIndicator(int x, int width, const char *label, bool ok,
                         uint16_t activeColor = C_GREEN) {
  const uint16_t fill = ok ? activeColor : C_CARD;
  lcd.fillRoundRect(x, 7, width, 22, 4, fill);
  lcd.drawRoundRect(x, 7, width, 22, 4, ok ? activeColor : C_LINE);
  lcdCentered(label, x + width / 2, 14, 1, ok ? C_BG : C_MUTED);
}

void drawDashboardHeaderDirect(bool force) {
  static uint32_t previousSignature = UINT32_MAX;
  static uint32_t previousSecond = UINT32_MAX;
  const bool wearable = fusionOnline && flag(RangerLink::FLAG_WEARABLE_LINK);
  const bool radar = fusionOnline && flag(RangerLink::FLAG_RADAR_FRESH);
  const uint32_t signature = (uint32_t)wearable | ((uint32_t)fusionOnline << 1) |
                             ((uint32_t)radar << 2) |
                             ((uint32_t)telem.battery << 8);
  if (force) {
    lcd.fillRect(0, 0, SCR_W, 50, C_SURFACE);
    lcd.drawFastHLine(0, 49, SCR_W, accent());
    lcdText(langTR ? "RANGER IZLEME" : "RANGER MONITOR", 15, 10, 3,
            accent());
    lcd.fillRoundRect(738, 7, 52, 32, 4, C_CARD);
    lcd.drawRoundRect(738, 7, 52, 32, 4, accent());
    lcdCentered("SET", 764, 17, 1, C_WHITE);
    previousSignature = UINT32_MAX;
    previousSecond = UINT32_MAX;
  }
  if (signature != previousSignature) {
    lcd.fillRect(475, 0, 255, 34, C_SURFACE);
    drawDirectIndicator(480, 54, "WEAR", wearable);
    drawDirectIndicator(540, 54, "PICO", fusionOnline);
    drawDirectIndicator(600, 48, "RAD", radar, accent());
    char battery[12];
    snprintf(battery, sizeof(battery), fusionOnline ? "%u%%" : "--%%",
             telem.battery);
    drawDirectIndicator(654, 62, battery,
                        fusionOnline && telem.battery > 20,
                        telem.battery > 50 ? C_GREEN : C_YELLOW);
    previousSignature = signature;
  }
  const uint32_t seconds = millis() / 1000;
  if (seconds != previousSecond) {
    lcd.fillRect(15, 34, 455, 14, C_SURFACE);
    char detail[96];
    snprintf(detail, sizeof(detail), "%s  UP %02lu:%02lu:%02lu", VERSION,
             seconds / 3600, (seconds / 60) % 60, seconds % 60);
    lcdText(detail, 18, 38, 1, C_MUTED);
    if (!fusionOnline)
      lcdText(linkErrorName(linkStats.error), 300, 38, 1, C_RED);
    previousSecond = seconds;
  }
}

void drawSystemStatusDirect(bool force) {
  static uint32_t previousSignature = UINT32_MAX;
  const uint32_t quality =
      linkStats.attempts ? linkStats.valid * 100UL / linkStats.attempts : 0;
  const uint32_t signature =
      (uint32_t)fusionOnline | ((uint32_t)telem.fallState << 1) |
      ((uint32_t)telem.flags << 4) | ((uint32_t)telem.battery << 12) |
      ((uint32_t)telem.mlFall << 20) | ((uint32_t)linkStats.error << 28);
  if (!force && signature == previousSignature)
    return;
  previousSignature = signature;
  lcd.fillRect(15, 85, 370, 145, C_BG);
  const bool wearable = fusionOnline && flag(RangerLink::FLAG_WEARABLE_LINK);
  const bool sos = fusionOnline && flag(RangerLink::FLAG_SOS_ACTIVE);
  const char *statusText;
  uint16_t badgeBackground, badgeText;
  if (!fusionOnline) {
    statusText = langTR ? "PICO BEKLENIYOR" : "WAITING FOR PICO";
    badgeBackground = C_CARD;
    badgeText = C_RED;
  } else if (sos || telem.fallState == 2) {
    statusText = sos ? "SOS!" : (langTR ? "DUSME!" : "FALL!");
    badgeBackground = C_RED;
    badgeText = C_WHITE;
  } else if (telem.fallState == 1) {
    statusText = langTR ? "DOGRULANIYOR" : "VERIFYING";
    badgeBackground = C_ORANGE;
    badgeText = C_BG;
  } else if (!wearable) {
    statusText = langTR ? "SINYAL YOK" : "NO WEARABLE";
    badgeBackground = C_CARD;
    badgeText = C_YELLOW;
  } else {
    statusText = langTR ? "GUVENLI" : "SECURE";
    badgeBackground = C_SECURE;
    badgeText = C_GREEN;
  }
  lcd.fillRoundRect(30, 95, 340, 50, 8, badgeBackground);
  lcd.drawRoundRect(30, 95, 340, 50, 8, badgeText);
  lcdCentered(statusText, 200, 107, 3, badgeText);
  char row[72];
  snprintf(row, sizeof(row), fusionOnline ? "BAT: %3u%%" : "BAT:  --%%",
           telem.battery);
  lcdText(row, 35, 160, 2, C_TEXT);
  lcd.drawRect(210, 162, 140, 16, C_LINE);
  const int batteryFill =
      fusionOnline ? constrain((int)telem.battery, 0, 100) * 136 / 100 : 0;
  if (batteryFill)
    lcd.fillRect(212, 164, batteryFill, 12,
                 telem.battery > 50 ? C_GREEN
                 : telem.battery > 20 ? C_YELLOW
                                      : C_RED);
  snprintf(row, sizeof(row), "ML FALL %3u%%   CONF %3u%%", telem.mlFall,
           telem.mlConfidence);
  lcdText(row, 35, 190, 1, fusionOnline ? C_TEXT : C_MUTED);
  snprintf(row, sizeof(row), "LINK %-13s Q %3lu%%", linkErrorName(linkStats.error),
           quality);
  lcdText(row, 35, 208, 1, fusionOnline ? C_GREEN : C_RED);
}

void drawRadarScopeDirect(int centerX, int centerY, int radius) {
  lcd.drawCircle(centerX, centerY, radius, C_LINE);
  lcd.drawCircle(centerX, centerY, radius * 2 / 3, C_LINE);
  lcd.drawCircle(centerX, centerY, radius / 3, C_LINE);
  lcd.drawFastVLine(centerX, centerY - radius, radius * 2, C_LINE);
  lcd.drawFastHLine(centerX - radius, centerY, radius * 2, C_LINE);
  if (!telem.radarTargets)
    return;
  const float distance = constrain(telem.radarDistanceMm / 1000.0f, 0.0f, 5.0f);
  const int targetY = centerY - (int)(distance / 5.0f * (radius - 5));
  lcd.fillCircle(centerX, targetY, 6, C_YELLOW);
  lcd.drawCircle(centerX, targetY, 10, C_ORANGE);
}

void drawRadarPanelDirect(bool force) {
  static uint32_t previousState = UINT32_MAX;
  static uint32_t previousValues = UINT32_MAX;
  const bool fresh = fusionOnline && radarEnabled &&
                     flag(RangerLink::FLAG_RADAR_FRESH);
  const bool target = fresh && telem.radarTargets > 0;
  const uint32_t state = (uint32_t)radarEnabled | ((uint32_t)fusionOnline << 1) |
                         ((uint32_t)fresh << 2) | ((uint32_t)target << 3);
  const uint32_t values = (uint16_t)telem.radarDistanceMm |
                          ((uint32_t)(uint16_t)telem.radarSpeedCms << 16) |
                          ((uint32_t)telem.radarTargets << 30);
  if (force || state != previousState) {
    lcd.fillRect(415, 85, 370, 145, C_BG);
    const char *label = !radarEnabled
                            ? (langTR ? "KAPALI" : "DISABLED")
                            : !fusionOnline ? "OFFLINE"
                            : !fresh ? (langTR ? "VERI YOK" : "NO DATA")
                            : target ? (langTR ? "HEDEF" : "TARGET")
                                     : (langTR ? "TARANIYOR" : "SCANNING");
    const uint16_t color = target ? C_YELLOW : fresh ? accent() : C_MUTED;
    lcd.fillRoundRect(430, 95, 340, 50, 8, target ? C_CARD : C_BG);
    lcd.drawRoundRect(430, 95, 340, 50, 8, color);
    lcdCentered(label, 600, 107, 3, color);
    previousState = state;
    previousValues = UINT32_MAX;
  }
  if (!fresh || (!force && values == previousValues))
    return;
  lcd.fillRect(430, 155, 340, 72, C_BG);
  char row[48];
  snprintf(row, sizeof(row), "DIST: %.2f m",
           telem.radarDistanceMm / 1000.0f);
  lcdText(row, 435, 162, 2, C_WHITE);
  snprintf(row, sizeof(row), "SPD:  %+d cm/s", telem.radarSpeedCms);
  lcdText(row, 435, 188, 2, C_TEXT);
  snprintf(row, sizeof(row), "TARGETS: %u", telem.radarTargets);
  lcdText(row, 435, 215, 1, C_MUTED);
  drawRadarScopeDirect(717, 190, 34);
  previousValues = values;
}

void drawMotionHistoryDirect(int x, int y, int width, int height) {
  lcd.drawRect(x, y, width, height, C_LINE);
  lcd.drawFastHLine(x, y + height / 2, width, C_LINE);
  if (motionHistoryCount < 2)
    return;
  const size_t oldest =
      (motionHistoryHead + MOTION_HISTORY_SIZE - motionHistoryCount) %
      MOTION_HISTORY_SIZE;
  const size_t renderCount = min(motionHistoryCount, MOTION_RENDER_POINTS);
  int previousX = x;
  float first = min(motionHistory[oldest], 4.0f);
  int previousY = y + height - 2 - (int)(first * (height - 4) / 4.0f);
  for (size_t i = 1; i < renderCount; ++i) {
    const size_t sourceOffset =
        i * (motionHistoryCount - 1) / (renderCount - 1);
    const size_t index = (oldest + sourceOffset) % MOTION_HISTORY_SIZE;
    const float value = min(motionHistory[index], 4.0f);
    const int currentX = x + (int)(i * (width - 1) / (renderCount - 1));
    const int currentY =
        y + height - 2 - (int)(value * (height - 4) / 4.0f);
    lcd.drawLine(previousX, previousY, currentX, currentY,
                 value > 2.5f ? C_RED : accent());
    previousX = currentX;
    previousY = currentY;
  }
}

void drawMotionPanelDirect(bool force) {
  static bool previousAvailable = false;
  static uint16_t previousSequence = UINT16_MAX;
  static uint32_t lastGraphDrawMs = 0;
  const bool available = fusionOnline && motionEnabled;
  const bool availabilityChanged = available != previousAvailable;
  if (force || availabilityChanged) {
    lcd.fillRect(15, 280, 770, 155, C_BG);
    if (!available)
      lcdCentered(motionEnabled ? "Waiting for Fusion Pico telemetry"
                                : "Motion panel disabled in Settings",
                  400, 342, 2, C_MUTED);
    previousAvailable = available;
    previousSequence = UINT16_MAX;
  }
  if (!available || (!force && telem.sequence == previousSequence))
    return;
  previousSequence = telem.sequence;
  const float axes[3] = {toG(telem.ax), toG(telem.ay), toG(telem.az)};
  const char *labels[3] = {"X-AXIS", "Y-AXIS", "Z-AXIS"};
  for (int i = 0; i < 3; ++i) {
    const int y = 293 + i * 36;
    lcdText(labels[i], 30, y, 2, C_TEXT);
    lcd.fillRect(115, y + 2, 180, 16, C_BG);
    lcd.drawRect(115, y + 2, 180, 14, C_LINE);
    const int centerX = 205;
    lcd.drawFastVLine(centerX, y + 3, 12, C_MUTED);
    const int extent = (int)(constrain(axes[i], -2.0f, 2.0f) * 45);
    const uint16_t barColor = fabsf(axes[i]) > 1.5f ? C_ORANGE : accent();
    if (extent > 0)
      lcd.fillRect(centerX, y + 4, extent, 10, barColor);
    else if (extent < 0)
      lcd.fillRect(centerX + extent, y + 4, -extent, 10, barColor);
    lcd.fillRect(305, y, 85, 20, C_BG);
    char value[16];
    snprintf(value, sizeof(value), "%+.2fg", axes[i]);
    lcdText(value, 307, y + 1, 2, C_WHITE);
  }
  lcd.fillRect(405, 290, 350, 70, C_BG);
  char row[72];
  snprintf(row, sizeof(row), "GYRO  X%+5.0f  Y%+5.0f  Z%+5.0f dps",
           toDps(telem.gx), toDps(telem.gy), toDps(telem.gz));
  lcdText(row, 405, 294, 1, C_MUTED);
  const float magnitude =
      sqrtf(axes[0] * axes[0] + axes[1] * axes[1] + axes[2] * axes[2]);
  snprintf(row, sizeof(row), "|G| %.2f", magnitude);
  lcdText(row, 405, 318, 2, magnitude > 2.5f ? C_RED : accent());
  snprintf(row, sizeof(row), "ML %u%%  CONF %u%%", telem.mlFall,
           telem.mlConfidence);
  lcdText(row, 570, 322, 1, C_TEXT);
  lcdText(flag(RangerLink::FLAG_BASE_FRESH) ? "BASE STREAM FRESH"
                                            : "BASE STREAM STALE",
          405, 348, 1,
          flag(RangerLink::FLAG_BASE_FRESH) ? C_GREEN : C_YELLOW);
  if (force || availabilityChanged || millis() - lastGraphDrawMs >= 2000) {
    lastGraphDrawMs = millis();
    if (motionGraphLayerReady) {
      motionGraphLayer.fillSprite(C_BG);
      drawMotionHistoryOn(motionGraphLayer, 0, 0, 350, 52);
      motionGraphLayer.pushSprite(405, 366);
    } else {
      lcd.fillRect(405, 366, 350, 52, C_BG);
      drawMotionHistoryDirect(405, 366, 350, 52);
    }
  }
}

void drawFooterChipDirect(int x, int width, const char *label, bool ok,
                          uint16_t activeColor = C_GREEN) {
  lcd.fillRoundRect(x, 450, width, 24, 4, ok ? activeColor : C_CARD);
  lcd.drawRoundRect(x, 450, width, 24, 4, ok ? activeColor : C_LINE);
  lcdCentered(label, x + width / 2, 457, 1, ok ? C_BG : C_MUTED);
}

void drawDashboardFooterDirect(bool force) {
  static uint32_t previousSignature = UINT32_MAX;
  static uint16_t previousSequence = UINT16_MAX;
  const uint32_t signature = (uint32_t)fusionOnline |
      ((uint32_t)(fusionOnline && flag(RangerLink::FLAG_WEARABLE_LINK)) << 1) |
      ((uint32_t)(fusionOnline && flag(RangerLink::FLAG_RADAR_FRESH)) << 2) |
      ((uint32_t)(fusionOnline && flag(RangerLink::FLAG_RELAY_ACTIVE)) << 3) |
      ((uint32_t)(WiFi.status() == WL_CONNECTED) << 4) |
      ((uint32_t)linkStats.error << 8);
  if (force || signature != previousSignature) {
    lcd.fillRect(0, 445, SCR_W, 35, C_SURFACE);
    lcd.drawFastHLine(0, 445, SCR_W, C_LINE);
    drawFooterChipDirect(15, 74, "WEAR", fusionOnline &&
                                             flag(RangerLink::FLAG_WEARABLE_LINK));
    drawFooterChipDirect(99, 68, "RAD", fusionOnline &&
                                           flag(RangerLink::FLAG_RADAR_FRESH),
                         accent());
    drawFooterChipDirect(177, 74, "PICO", fusionOnline);
    drawFooterChipDirect(261, 78, "CAM", fusionOnline &&
                                             flag(RangerLink::FLAG_RELAY_ACTIVE),
                         C_ORANGE);
    drawFooterChipDirect(349, 74, "WIFI", WiFi.status() == WL_CONNECTED);
    previousSignature = signature;
    previousSequence = UINT16_MAX;
  }
  if (!force && telem.sequence == previousSequence)
    return;
  lcd.fillRect(438, 450, 352, 24, C_SURFACE);
  char details[100];
  if (fusionOnline)
    snprintf(details, sizeof(details), "SEQ %u  CRC %lu  GAP %lu  PICO %us",
             linkStats.lastSequence, linkStats.badCrc, linkStats.sequenceGaps,
             telem.uptimeSeconds);
  else
    snprintf(details, sizeof(details), "SDA15-GP2  SCL16-GP3  GND-GND");
  lcdText(details, 438, 458, 1, fusionOnline ? C_MUTED : C_YELLOW);
  previousSequence = telem.sequence;
}

void drawDashboard() {
  const bool force = dashboardBackgroundDirty;
  if (force) {
    drawDashboardBackground();
    dashboardBackgroundDirty = false;
  }
  drawDashboardHeaderDirect(force);
  drawSystemStatusDirect(force);
  drawRadarPanelDirect(force);
  drawMotionPanelDirect(force);
  drawDashboardFooterDirect(force);
}

void drawSettings() {
  if (!settingsDirty)
    return;
  settingsDirty = false;
  lcd.fillScreen(C_BG);
  lcd.fillRect(0, 0, SCR_W, 58, C_SURFACE);
  lcd.fillRect(0, 56, SCR_W, 2, accent());
  lcdText("RANGER", 18, 11, 3, C_WHITE);
  lcdText(langTR ? "AYARLAR" : "SETTINGS", 174, 19, 2, C_MUTED);
  lcdText(langTR ? "Bir secenege dokunun" : "Tap an option", 22, 75, 2,
          C_MUTED);
  const char *labelsEn[8] = {
      "ALARM SOUND",   "RADAR PANEL",  "MOTION PANEL", "IMPACT TRIGGER",
      "STILLNESS CHECK", "ACCENT COLOR", "LANGUAGE",     "BACK / SAVE"};
  const char *labelsTr[8] = {
      "ALARM SESI", "RADAR PANELI", "HAREKET PANELI", "DARBE TETIKLEYICI",
      "HAREKETSIZLIK", "TEMA RENGI", "DIL", "GERI / KAYDET"};
  for (int i = 0; i < 8; ++i) {
    int col = i % 2, row = i / 2, x = 20 + col * 390, y = 92 + row * 82;
    lcd.fillRoundRect(x, y, 370, 70, 10, C_CARD);
    lcd.drawRoundRect(x, y, 370, 70, 10, i == 7 ? accent() : C_LINE);
    lcdText(langTR ? labelsTr[i] : labelsEn[i], x + 18, y + 10, 2, C_TEXT);
    String state;
    bool enabled = false;
    if (i == 0)
      state = muted ? (langTR ? "SESSIZ" : "MUTED") : (langTR ? "ACIK" : "ON");
    else if (i == 1)
      state =
          radarEnabled ? (langTR ? "ACIK" : "ON") : (langTR ? "KAPALI" : "OFF");
    else if (i == 2)
      state = motionEnabled ? (langTR ? "ACIK" : "ON")
                            : (langTR ? "KAPALI" : "OFF");
    else if (i == 3)
      state = impactTriggerEnabled ? (langTR ? "ACIK" : "ON")
                                   : (langTR ? "KAPALI" : "OFF");
    else if (i == 4)
      state = stillnessCheckEnabled ? (langTR ? "ACIK" : "ON")
                                    : (langTR ? "KAPALI" : "OFF");
    else if (i == 5)
      state = langTR ? "SONRAKI" : "NEXT";
    else if (i == 6)
      state = langTR ? "TURKCE" : "ENGLISH";
    else
      state = langTR ? "GERI" : "RETURN";
    if (i == 0)
      enabled = !muted;
    else if (i == 1)
      enabled = radarEnabled;
    else if (i == 2)
      enabled = motionEnabled;
    else if (i == 3)
      enabled = impactTriggerEnabled;
    else if (i == 4)
      enabled = stillnessCheckEnabled;
    else
      enabled = true;
    const uint16_t pillColor = enabled ? (i >= 5 ? accent() : C_GREEN) : C_RED;
    lcd.fillRoundRect(x + 250, y + 38, 98, 23, 11, pillColor);
    lcdCentered(state, x + 299, y + 43, 1, C_BG);
  }
  lcdText(langTR ? "PICO ROLE TESTI" : "PICO RELAY TEST", 25, 445, 1,
          C_ORANGE);
  lcdText(langTR ? "dokun" : "tap here", 150, 445, 1, C_MUTED);
}

void drawAlert() {
  const bool urgent =
      telem.fallState == 2 || flag(RangerLink::FLAG_SOS_ACTIVE);
  const uint16_t color = urgent ? C_RED : C_YELLOW;
  static uint32_t previousRemaining = UINT32_MAX;
  if (emergencyPageDirty) {
    emergencyPageDirty = false;
    previousRemaining = UINT32_MAX;
    lcd.fillScreen(C_BG);
    lcd.fillRect(0, 0, SCR_W, 12, color);
    lcdCentered(langTR ? "DUSME ALGILANDI" : "FALL DETECTED", 400, 52, 4,
                color);
    lcdCentered(
        urgent ? (langTR ? "ACIL DURUM DOGRULANDI" : "EMERGENCY VERIFIED")
               : (langTR ? "DOGRULANIYOR" : "VERIFYING STILLNESS"),
        400, 112, 2, C_WHITE);
    char info[80];
    snprintf(info, sizeof(info), "Impact %.2f g    ML %u%%    Confidence %u%%",
             alertImpact, telem.mlFall, telem.mlConfidence);
    lcdCentered(info, 400, 162, 2, C_TEXT);
    lcd.fillRoundRect(130, 225, 540, 116, 10, color);
    lcd.drawRoundRect(130, 225, 540, 116, 10, color);
    lcdCentered(langTR ? "ALARMI IPTAL ET" : "CANCEL ALARM", 400, 255, 4,
                C_BG);
    lcdCentered(
        langTR ? "Guvendeyseniz dokunun" : "Tap only if the person is safe",
        400, 312, 1, C_BG);
    lcdCentered("CrowPanel GPIO8 alarm | Pico cancel command uses CRC8", 400,
                438, 1, C_MUTED);
  }
  uint32_t elapsedMs = millis() - pageStartMs;
  uint32_t remaining = elapsedMs < ALERT_ESCALATE_MS
                           ? (ALERT_ESCALATE_MS - elapsedMs + 999) / 1000
                           : 0;
  if (remaining != previousRemaining) {
    lcd.fillRect(180, 365, 440, 30, C_BG);
    char timer[50];
    snprintf(timer, sizeof(timer),
             langTR ? "Tehlike ekranina %lu sn" : "Danger escalation in %lu s",
             remaining);
    lcdCentered(timer, 400, 375, 2, remaining < 10 ? C_RED : C_MUTED);
    previousRemaining = remaining;
  }
}
void drawDanger() {
  static bool previousPulse = false;
  const bool force = emergencyPageDirty;
  if (force) {
    emergencyPageDirty = false;
    lcd.fillScreen(C_BG);
    lcdCentered(langTR ? "TEHLIKE" : "DANGER", 400, 48, 6, C_RED);
    lcdCentered(langTR ? "ACIL DURUM DEVAM EDIYOR"
                       : "EMERGENCY RESPONSE ACTIVE",
                400, 130, 2, C_WHITE);
    lcdCentered(langTR ? "Kaynak: Giyilebilir + ML"
                       : "Source: Wearable + ML",
                400, 172, 2, C_TEXT);
    lcd.fillRoundRect(130, 225, 540, 116, 10, C_RED);
    lcd.drawRoundRect(130, 225, 540, 116, 10, C_WHITE);
    lcdCentered(langTR ? "GUVENLIYSE IPTAL ET" : "CANCEL IF SAFE", 400, 255,
                4, C_BG);
    lcdCentered(langTR ? "Alarm ve bildirim etkin"
                       : "Alarm and notification active",
                400, 376, 2, C_YELLOW);
    lcdCentered("RANGER v5", 400, 444, 1, C_MUTED);
  }
  // Animate only the thin warning strip; never repaint the 800x480 surface.
  const bool pulse = (millis() / 700) % 2;
  if (force || pulse != previousPulse) {
    lcd.fillRect(0, 0, SCR_W, 18, pulse ? C_RED : C_ORANGE);
    previousPulse = pulse;
  }
}
void drawCleared() {
  if (!emergencyPageDirty)
    return;
  emergencyPageDirty = false;
  lcd.fillScreen(C_BG);
  lcdCentered(langTR ? "ALARM TEMIZLENDI" : "ALARM CLEARED", 400, 190, 4,
              C_GREEN);
  lcdCentered("Returning to live monitor", 400, 255, 2, C_MUTED);
}

void cancelAlert() {
  sendFusionCommand(RangerLink::CMD_CANCEL_ALARM);
  setBuzzer(false);
  alertActive = false;
  // Give the Pico two seconds to publish the cleared state. If the command
  // failed or the emergency remains active, the dashboard will alert again.
  alertSuppressedUntilMs = millis() + 2000;
  page = PAGE_CLEARED;
  pageStartMs = millis();
  emergencyPageDirty = true;
  Serial0.println("[ALERT] Cancel requested");
}
void enterAlert(float impact) {
  if (alertActive)
    return;
  alertActive = true;
  alertImpact = impact;
  page = PAGE_ALERT;
  pageStartMs = millis();
  emergencyPageDirty = true;
  telegramImpact = impact;
  triggerTelegram = true;
  setBuzzer(true);
  lastToneChangeMs = millis();
  Serial0.printf("[ALERT] Fusion impact=%.2f\n", impact);
}
void handleTouches() {
  if (!touchJustPressed)
    return;
  if (page == PAGE_DASHBOARD && hit(730, 0, 70, 65)) {
    page = PAGE_SETTINGS;
    pageStartMs = millis();
    settingsDirty = true;
    return;
  }
  if ((page == PAGE_ALERT || page == PAGE_DANGER) && hit(130, 225, 540, 116)) {
    cancelAlert();
    return;
  }
  if (page != PAGE_SETTINGS)
    return;
  for (int i = 0; i < 8; ++i) {
    int x = 20 + (i % 2) * 390, y = 92 + (i / 2) * 82;
    if (!hit(x, y, 370, 70))
      continue;
    if (i == 0)
      muted = !muted;
    else if (i == 1)
      radarEnabled = !radarEnabled;
    else if (i == 2)
      motionEnabled = !motionEnabled;
    else if (i == 3)
      impactTriggerEnabled = !impactTriggerEnabled;
    else if (i == 4)
      stillnessCheckEnabled = !stillnessCheckEnabled;
    else if (i == 5)
      themeIndex = (themeIndex + 1) % 6;
    else if (i == 6)
      langTR = !langTR;
    else {
      saveSettings();
      fallSettingsSynced = false;
      page = PAGE_DASHBOARD;
      dashboardBackgroundDirty = true;
    }
    if (i == 3 || i == 4)
      fallSettingsSynced = false;
    settingsDirty = true;
    return;
  }
  if (hit(15, 430, 220, 50))
    sendFusionCommand(RangerLink::CMD_RELAY_TEST);
}

bool probeI2C(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}
void drawBoot(bool expander, bool touch, bool fusion) {
  canvas.fillScreen(C_BG);
  centered("RANGER", 400, 55, 5, accent());
  centered("Care Monitor / hardware self-test", 400, 125, 2, C_MUTED);
  box(135, 195, 530, 170, C_CARD);
  text("DISPLAY", 175, 220, 2, C_TEXT);
  pill(500, 216, 115, "OK", true);
  text("I2C EXPANDER 0x18", 175, 258, 2, C_TEXT);
  pill(500, 254, 115, expander ? "OK" : "FAIL", expander);
  text("TOUCH GT911 0x5D", 175, 296, 2, C_TEXT);
  pill(500, 292, 115, touch ? "OK" : "CHECK", touch);
  text("FUSION PICO 0x42", 175, 334, 2, C_TEXT);
  pill(500, 330, 115, fusion ? "OK" : "WAIT", fusion);
  centered(VERSION "  |  " BUILD_DATE, 400, 415, 1, C_MUTED);
  canvas.pushSprite(0, 0);
}

void setup() {
  Serial0.begin(115200);
  delay(400);
  Serial0.println("\n--- RANGER " VERSION " ---");
  pinMode(PIN_BUZZER, OUTPUT);
  setBuzzer(false);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(30);
  ioex.attach(Wire);
  ioex.setDeviceAddress(TCA9534_ADDR);
  ioex.config(1, TCA9534::Config::OUT);
  ioex.config(2, TCA9534::Config::OUT);
  ioex.output(1, TCA9534::Level::H);
  pinMode(PIN_TOUCH_INT, OUTPUT);
  digitalWrite(PIN_TOUCH_INT, LOW);
  ioex.output(2, TCA9534::Level::L);
  delay(20);
  ioex.output(2, TCA9534::Level::H);
  delay(100);
  pinMode(PIN_TOUCH_INT, INPUT);
  bool expander = probeI2C(TCA9534_ADDR), touch = probeI2C(0x5D);
  lcd.init();
  lcd.initDMA();
  lcd.fillScreen(C_BG);
  if (ESP.getPsramSize() == 0) {
    lcd.setTextColor(C_RED);
    lcd.setTextSize(3);
    lcd.setCursor(25, 50);
    lcd.println("PSRAM NOT FOUND");
    lcd.setTextSize(2);
    lcd.println("Select OPI PSRAM + 16MB Flash");
    while (true)
      delay(1000);
  }
  canvas.setColorDepth(16);
  canvas.setPsram(true);
  if (!canvas.createSprite(SCR_W, SCR_H)) {
    lcd.setTextColor(C_RED);
    lcd.setTextSize(2);
    lcd.drawString("Full-screen buffer allocation failed", 30, 30);
    while (true)
      delay(1000);
  }
  // This small animated region fits safely in internal SRAM. Rendering it
  // there avoids reading a second PSRAM surface while RGB DMA is scanning.
  motionGraphLayer.setColorDepth(16);
  motionGraphLayer.setPsram(false);
  motionGraphLayerReady = motionGraphLayer.createSprite(350, 52) != nullptr;
  Serial0.printf("[DISPLAY] Internal motion buffer: %s\n",
                 motionGraphLayerReady ? "OK" : "fallback direct");
  EEPROM.begin(EEPROM_SIZE);
  loadSettings();
  bool fusion = pollFusion();
  drawBoot(expander, touch, fusion);
  // The Pico and its peripherals can take longer to settle than the display.
  // Retry throughout the boot page instead of treating the first probe as the
  // final result. Once seen, keep the boot result latched as OK.
  const uint32_t bootProbeStartMs = millis();
  uint32_t lastBootProbeMs = millis();
  while (millis() - bootProbeStartMs < 1800) {
    if (millis() - lastBootProbeMs >= POLL_PERIOD_MS) {
      lastBootProbeMs = millis();
      const bool wasFusion = fusion;
      fusion = pollFusion() || fusion;
      if (fusion != wasFusion)
        drawBoot(expander, touch, fusion);
    }
    delay(5);
  }

  // Continuous WiFi can starve RGB DMA and cause CRT-like horizontal jitter.
  // The network task turns it on only long enough to send an emergency alert.
  WiFi.mode(WIFI_OFF);
  xTaskCreatePinnedToCore(networkTask, "network", 8192, nullptr, 1, nullptr, 0);
  page = PAGE_DASHBOARD;
  pageStartMs = millis();
  dashboardBackgroundDirty = true;
  Serial0.println("[BOOT] Dashboard ready");
}

void loop() {
  uint32_t now = millis();
  static uint32_t lastPoll = 0;
  if (now - lastPoll >= POLL_PERIOD_MS) {
    lastPoll = now;
    pollFusion();
  }
  fusionOnline = linkStats.lastValidMs &&
                 now - linkStats.lastValidMs <= LINK_STALE_MS &&
                 linkStats.lastAdvanceMs &&
                 now - linkStats.lastAdvanceMs <= LINK_FROZEN_MS;
  if (!fusionOnline && linkStats.error == LINK_OK && linkStats.lastValidMs &&
      now - linkStats.lastAdvanceMs > LINK_FROZEN_MS)
    linkStats.error = LINK_FROZEN;
  static bool previousFusionOnline = false;
  const bool fusionStateChanged = fusionOnline != previousFusionOnline;
  if (fusionStateChanged) {
    previousFusionOnline = fusionOnline;
    if (!fusionOnline)
      fallSettingsSynced = false;
    Serial0.printf("[LINK] Pico transition: %s\n",
                   fusionOnline ? "ONLINE" : "OFFLINE");
  }
  if (fusionOnline && !fallSettingsSynced &&
      now - lastFallSettingsSyncAttemptMs >= FALL_SETTINGS_SYNC_RETRY_MS)
    syncFallSettings();
  static uint32_t lastTouchPoll = 0;
  if (now - lastTouchPoll >= 25) {
    lastTouchPoll = now;
    pollTouch();
    handleTouches();
  }

  if (fusionOnline &&
      (telem.fallState == 2 || flag(RangerLink::FLAG_SOS_ACTIVE)) &&
      !alertActive && (int32_t)(now - alertSuppressedUntilMs) >= 0) {
    float x = toG(telem.ax), y = toG(telem.ay), z = toG(telem.az);
    enterAlert(sqrtf(x * x + y * y + z * z));
  }
  if (page == PAGE_ALERT && now - pageStartMs >= ALERT_ESCALATE_MS) {
    page = PAGE_DANGER;
    pageStartMs = now;
    emergencyPageDirty = true;
    setBuzzer(true);
    Serial0.println("[ALERT] Escalated to danger screen");
  }
  if ((page == PAGE_ALERT || page == PAGE_DANGER) && !muted &&
      now - lastToneChangeMs >= 300) {
    lastToneChangeMs = now;
    setBuzzer(!alertToneOn);
  }
  if (page == PAGE_CLEARED && now - pageStartMs >= 1500) {
    page = PAGE_DASHBOARD;
    pageStartMs = now;
    dashboardBackgroundDirty = true;
  }

  static uint32_t lastDraw = 0;
  const uint32_t drawPeriod = page == PAGE_DANGER ? 700
                              : page == PAGE_DASHBOARD ? 1000
                              : page == PAGE_ALERT ? 1000
                                                       : 500;
  if (now - lastDraw >= drawPeriod) {
    lastDraw = now;
    if (page == PAGE_DASHBOARD)
      drawDashboard();
    else if (page == PAGE_SETTINGS)
      drawSettings();
    else if (page == PAGE_ALERT)
      drawAlert();
    else if (page == PAGE_DANGER)
      drawDanger();
    else
      drawCleared();
  }
  static uint32_t lastLog = 0;
  if (now - lastLog >= 5000) {
    lastLog = now;
    Serial0.printf("[CROW] link=%d error=%s seq=%u valid=%lu/%lu crc=%lu "
                   "gaps=%lu wifi=%d ch=%ld heap=%lu\n",
                   fusionOnline, linkErrorName(linkStats.error),
                   linkStats.lastSequence, linkStats.valid, linkStats.attempts,
                   linkStats.badCrc, linkStats.sequenceGaps,
                   WiFi.status() == WL_CONNECTED, (long)WiFi.channel(),
                   (unsigned long)ESP.getFreeHeap());
  }
}
