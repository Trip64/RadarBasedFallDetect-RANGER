/**
 * RANGER — Tactical HMI Dashboard (CrowPanel Advance 4.3" ESP32-S3)
 * 
 * Hardware:
 *   - Display: ST7265 RGB 16-bit Parallel, 800x480 IPS (LovyanGFX + PSRAM Framebuffer)
 *   - Touch: GT911 Capacitive Touch Controller (I2C @ 0x5D, GPIO 15/16)
 *   - IO Expander: TCA9534 (I2C @ 0x18) — Backlight (Pin 1), Touch Reset (Pin 2)
 *   - Bus: Shared I2C Master polling RP2040 Fusion Node @ 0x42
 *   - Radio: ESP-NOW peer-to-peer receiver for Bathroom zone fall alerts
 */

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <TCA9534.h>
#include "gfx_conf.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <esp_now.h>

#define VERSION     "3.5-CROWPANEL-ADV"
#define BUILD_DATE  "2026-04-21"

// Network & Notification Configuration
#define WIFI_SSID           "YOUR_WIFI_SSID"
#define WIFI_PASS           "YOUR_WIFI_PASSWORD"
#define TELEGRAM_BOT_TOKEN  "YOUR_TELEGRAM_BOT_TOKEN"
#define TELEGRAM_CHAT_ID    "YOUR_TELEGRAM_CHAT_ID"

// Audio Alert via I2C Expander
void setBuzzer(bool on) {
    Wire.beginTransmission(0x30);
    Wire.write(on ? 0xF6 : 0xF7);
    Wire.endTransmission();
}

volatile bool triggerTelegram = false;
volatile float telegramImpact = 0;

static LGFX lcd;
#define SCR_W DISPLAY_WIDTH
#define SCR_H DISPLAY_HEIGHT

TCA9534 ioex;

// I2C Fusion Node Protocol
#define FUSION_I2C_ADDR  0x42
#define TELEM_SIZE       24

#pragma pack(push, 1)
struct TelemetryPacket {
    uint8_t  magic;         // 0xAA preamble
    int16_t  ax, ay, az;    // Raw accel (÷4096 -> g)
    int16_t  gx, gy, gz;    // Raw gyro  (÷65.5 -> dps)
    uint8_t  battery;       // 0–100%
    uint8_t  sos;           // 0 or 1
    uint8_t  fallState;     // 0=normal, 1=verifying, 2=confirmed
    uint8_t  wearLink;      // 0=offline, 1=connected
    uint8_t  radarPresent;  // 0=clear, 1=target tracked
    int16_t  radarDist_mm;  // Distance in mm
    int16_t  radarSpeed;    // Velocity in cm/s
    uint8_t  radarTargets;  // Target count (0–3)
    uint8_t  checksum;      // XOR checksum
};
#pragma pack(pop)

TelemetryPacket telem;
unsigned long lastTelemTime = 0;
bool fusionOnline = false;

// UI Color Palette (RGB565)
#define COL_BG       0x0841   // Dark slate background
#define COL_PANEL    0x1082   // Card panel background
#define COL_BORDER   0x3186   // Border highlight
#define COL_HEADER   0x18C3   // Header bar
#define COL_TEXT     0xC618   // Off-white text
#define COL_BRIGHT   0xFFFF   // Pure white
#define COL_ACCENT   0x07FF   // Cyan accent
#define COL_GREEN    0x07E0   // Green status
#define COL_RED      0xF800   // Red emergency
#define COL_YELLOW   0xFFE0   // Warning yellow
#define COL_ORANGE   0xFD20   // Alert orange
#define COL_DIMTEXT  0x630C   // Dim secondary text
#define COL_SECURE   0x2E8B   // Secure badge background

bool langTR = false;

enum SysState { ST_BOOT, ST_MONITOR, ST_SETTINGS, ST_ALERT, ST_DANGER, ST_COOLDOWN };
SysState curState = ST_BOOT;
unsigned long stateStart = 0;

// Bathroom Node ESP-NOW Packet
#pragma pack(push, 1)
struct BathPacket {
    uint8_t  magic;       // 0xBB
    uint8_t  presence;    // 0=empty, 1=moving, 2=occupied
    uint16_t distance_cm;
    uint8_t  fallState;   // 0=none, 1=possible, 2=confirmed
    uint8_t  checksum;
};
#pragma pack(pop)

volatile bool bathOnline = false;
volatile uint8_t bathPresence = 0;
volatile uint8_t bathFallState = 0;
volatile uint16_t bathDist = 0;
volatile unsigned long lastBathTime = 0;

void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (len == sizeof(BathPacket)) {
        BathPacket pkt;
        memcpy(&pkt, data, sizeof(pkt));
        if (pkt.magic == 0xBB) {
            uint8_t ck = 0;
            for (size_t i = 0; i < sizeof(pkt) - 1; i++) ck ^= ((uint8_t*)&pkt)[i];
            if (ck == pkt.checksum) {
                bathPresence = pkt.presence;
                bathFallState = pkt.fallState;
                bathDist = pkt.distance_cm;
                lastBathTime = millis();
                bathOnline = true;
            }
        }
    }
}

void initEspNow() {
    if (esp_now_init() != ESP_OK) {
        Serial0.println("[ESP-NOW] Init FAILED");
        return;
    }
    esp_now_register_recv_cb(onEspNowRecv);
}

bool fallDetected = false;
float fallImpact = 0;
int alertTimeout = 30000;
unsigned long lastBeep = 0;

// Physical Input Button (GPIO 6, Active LOW)
#define BTN_PIN 6

volatile unsigned long isrBtnPressedAt = 0;
volatile bool isrBtnShortPress = false;
volatile bool isrBtnLongPress = false;

void IRAM_ATTR btnISR() {
    bool state = digitalRead(BTN_PIN);
    unsigned long t = millis();
    
    if (state == LOW) {
        if (isrBtnPressedAt == 0) isrBtnPressedAt = t;
    } else {
        if (isrBtnPressedAt > 0) {
            unsigned long dur = t - isrBtnPressedAt;
            if (dur > 50 && dur < 800) isrBtnShortPress = true;
            else if (dur >= 800) isrBtnLongPress = true;
            isrBtnPressedAt = 0;
        }
    }
}

// System Settings
bool muted = false;
bool radarEnabled = true;
bool mpuEnabled = true;
int themeIndex = 0;
int menuIndex = 0;
#define EEPROM_SIZE 16

void saveSettings();
void drawBackground();

void toggleSetting(int index) {
    if (index == 0) muted = !muted;
    else if (index == 1) radarEnabled = !radarEnabled;
    else if (index == 2) mpuEnabled = !mpuEnabled;
    else if (index == 3) { themeIndex++; if (themeIndex > 5) themeIndex = 0; }
    else if (index == 4) langTR = !langTR;
    else if (index == 5) {
        saveSettings();
        curState = ST_MONITOR;
        stateStart = millis();
        drawBackground();
    }
}

void saveSettings() {
    EEPROM.write(0, muted ? 1 : 0);
    EEPROM.write(1, langTR ? 1 : 0);
    EEPROM.write(2, radarEnabled ? 1 : 0);
    EEPROM.write(3, mpuEnabled ? 1 : 0);
    EEPROM.write(4, themeIndex);
    EEPROM.commit();
}

void loadSettings() {
    if (EEPROM.read(0) != 255) {
        muted = (EEPROM.read(0) == 1);
        langTR = (EEPROM.read(1) == 1);
        radarEnabled = (EEPROM.read(2) == 1);
        mpuEnabled = (EEPROM.read(3) == 1);
        themeIndex = EEPROM.read(4);
    }
}

uint16_t getThemeColor() {
    switch (themeIndex) {
        case 0: return COL_ACCENT;
        case 1: return COL_GREEN;
        case 2: return 0xF81F;     // Magenta
        case 3: return COL_YELLOW;
        case 4: return COL_ORANGE;
        case 5: return 0x03FF;     // Blue
        default: return COL_ACCENT;
    }
}

String urlEncode(const String& str) {
    String encoded = "";
    for (size_t i = 0; i < str.length(); i++) {
        char c = str.charAt(i);
        if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') encoded += c;
        else if (c == ' ') encoded += '+';
        else {
            char buf[4]; snprintf(buf, sizeof(buf), "%%%02X", (unsigned char)c);
            encoded += buf;
        }
    }
    return encoded;
}

// Background FreeRTOS task on Core 0 for non-blocking Telegram alerts
void networkTask(void *pvParameters) {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (true) {
        if (WiFi.status() == WL_CONNECTED && triggerTelegram) {
            triggerTelegram = false;
            
            String msg = langTR ? "🚨 ONAYLANMIS DUSME BILDIRIMI! 🚨\nSistem dusme tespit etti. Ivme: " : "🚨 VERIFIED FALL ALERT! 🚨\nImpact magnitude: ";
            msg += String(telegramImpact, 1) + "g";

            if (strlen(TELEGRAM_BOT_TOKEN) > 0 && strcmp(TELEGRAM_BOT_TOKEN, "YOUR_TELEGRAM_BOT_TOKEN") != 0) {
                HTTPClient http;
                String url = "https://api.telegram.org/bot" + String(TELEGRAM_BOT_TOKEN) 
                           + "/sendMessage?chat_id=" + String(TELEGRAM_CHAT_ID) 
                           + "&text=" + urlEncode(msg);
                http.begin(url);
                http.GET();
                http.end();
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

int16_t touchX = -1, touchY = -1;
bool touched = false;
unsigned long lastTouch = 0;

void pollTouch() {
    lgfx::touch_point_t tp;
    int n = lcd.getTouch(&tp, 1);
    if (n > 0) {
        touchX = tp.x;
        touchY = tp.y;
        touched = true;
        lastTouch = millis();
    } else {
        touched = false;
    }
}

bool pollFusion() {
    uint8_t buf[TELEM_SIZE];
    int got = Wire.requestFrom((uint8_t)FUSION_I2C_ADDR, (uint8_t)TELEM_SIZE);
    if (got < TELEM_SIZE) {
        while (Wire.available()) Wire.read();
        return false;
    }

    for (int i = 0; i < TELEM_SIZE; i++) {
        buf[i] = Wire.read();
    }

    if (buf[0] != 0xAA) return false;

    uint8_t ck = 0;
    for (int i = 0; i < TELEM_SIZE - 1; i++) ck ^= buf[i];
    if (ck != buf[TELEM_SIZE - 1]) return false;

    memcpy(&telem, buf, TELEM_SIZE);
    lastTelemTime = millis();
    fusionOnline = true;
    return true;
}

inline float toG(int16_t raw) { return (float)raw / 4096.0f; }
inline float toDPS(int16_t raw) { return (float)raw / 65.5f; }

// Double-buffered Sprite Rendering Architecture
void drawBackground() {
    lcd.fillScreen(COL_BG);
    lcd.fillRect(0, 0, SCR_W, 50, COL_HEADER);

    lcd.drawRoundRect(10, 55, 380, 180, 6, COL_BORDER);
    lcd.drawRoundRect(410, 55, 380, 180, 6, COL_BORDER);
    lcd.drawRoundRect(10, 250, 780, 170, 6, COL_BORDER);

    lcd.setTextColor(getThemeColor(), COL_BG);
    lcd.setTextSize(2);
    lcd.setCursor(25, 62);
    lcd.print(langTR ? "SISTEM DURUMU" : "SYSTEM STATUS");
    lcd.setCursor(425, 62);
    lcd.print(langTR ? "RADAR IZLEME" : "RADAR TRACKING");
    lcd.setCursor(25, 257);
    lcd.print(langTR ? "IVME SENSORU" : "ACCELEROMETER");

    lcd.drawFastHLine(15, 82, 370, COL_BORDER);
    lcd.drawFastHLine(415, 82, 370, COL_BORDER);
    lcd.drawFastHLine(15, 277, 770, COL_BORDER);

    lcd.fillRect(0, SCR_H - 35, SCR_W, 35, COL_HEADER);
    lcd.drawFastHLine(0, SCR_H - 36, SCR_W, COL_BORDER);
}

void drawHeader() {
    LGFX_Sprite sp(&lcd);
    sp.setColorDepth(16);
    sp.setPsram(true);
    sp.createSprite(SCR_W, 50);
    sp.fillSprite(COL_HEADER);
    sp.drawFastHLine(0, 49, SCR_W, getThemeColor());

    sp.setTextSize(3);
    sp.setTextColor(getThemeColor());
    sp.setCursor(15, 12);
    sp.print(langTR ? "RANGER IZLEME" : "RANGER MONITOR");

    sp.setTextSize(2);
    int bx = SCR_W - 290;

    sp.setTextColor(telem.wearLink ? COL_GREEN : COL_RED);
    sp.setCursor(bx, 8);
    sp.print("BLE");

    bx += 55;
    uint16_t batCol = telem.battery > 50 ? COL_GREEN : (telem.battery > 20 ? COL_YELLOW : COL_RED);
    sp.setTextColor(batCol);
    sp.setCursor(bx, 8);
    char batBuf[8];
    snprintf(batBuf, sizeof(batBuf), "%3d%%", telem.battery);
    sp.print(batBuf);

    bx += 60;
    sp.setTextColor(fusionOnline ? COL_GREEN : COL_RED);
    sp.setCursor(bx, 8);
    sp.print("FUSE");

    bx += 65;
    sp.setTextColor(telem.radarPresent ? getThemeColor() : COL_DIMTEXT);
    sp.setCursor(bx, 8);
    sp.print("RAD");

    sp.drawRoundRect(SCR_W - 60, 10, 50, 30, 4, COL_BORDER);
    sp.setTextColor(COL_DIMTEXT);
    sp.setCursor(SCR_W - 52, 17);
    sp.print("SET");

    sp.setTextSize(1);
    sp.setTextColor(COL_DIMTEXT);
    unsigned long sec = millis() / 1000;
    char uptBuf[40];
    snprintf(uptBuf, sizeof(uptBuf), "v%s  UP %02lu:%02lu:%02lu", VERSION, sec / 3600, (sec / 60) % 60, sec % 60);
    sp.setCursor(SCR_W - 260, 36);
    sp.print(uptBuf);

    sp.pushSprite(0, 0);
    sp.deleteSprite();
}

void drawStatusPanel() {
    LGFX_Sprite sp(&lcd);
    sp.setColorDepth(16);
    sp.setPsram(true);
    sp.createSprite(370, 145);
    sp.fillSprite(COL_BG);

    bool connected = telem.wearLink && fusionOnline;
    bool sos = telem.sos;

    uint16_t badgeBg, badgeTxt;
    const char* statusText;

    if (sos) {
        badgeBg = COL_RED; badgeTxt = COL_BRIGHT;
        statusText = "SOS!";
    } else if (!connected) {
        badgeBg = COL_RED; badgeTxt = COL_BRIGHT;
        statusText = langTR ? "SINYAL YOK" : "NO LINK";
    } else if (telem.fallState == 2) {
        badgeBg = COL_RED; badgeTxt = COL_BRIGHT;
        statusText = langTR ? "DUSME!" : "FALL!";
    } else if (telem.fallState == 1) {
        badgeBg = COL_ORANGE; badgeTxt = COL_BRIGHT;
        statusText = langTR ? "DOGRULANIYOR" : "VERIFYING";
    } else {
        badgeBg = COL_SECURE; badgeTxt = COL_GREEN;
        statusText = langTR ? "GUVENLI" : "SECURE";
    }

    sp.fillRoundRect(15, 10, 340, 50, 8, badgeBg);
    sp.drawRoundRect(15, 10, 340, 50, 8, COL_BRIGHT);
    sp.setTextSize(3);
    sp.setTextColor(badgeTxt);
    int tw = strlen(statusText) * 18;
    sp.setCursor(15 + (340 - tw) / 2, 22);
    sp.print(statusText);

    sp.setTextSize(2);
    sp.setTextColor(COL_TEXT);
    sp.setCursor(20, 75);
    sp.printf("BAT: %3d%%", telem.battery);

    int barX = 195, barW = 140, barH = 16;
    sp.drawRect(barX, 75, barW, barH, COL_BORDER);
    int fill = (telem.battery * (barW - 4)) / 100;
    uint16_t barCol = telem.battery > 50 ? COL_GREEN : (telem.battery > 20 ? COL_YELLOW : COL_RED);
    if (fill > 0) sp.fillRect(barX + 2, 77, fill, barH - 4, barCol);

    sp.setTextSize(1);
    sp.setTextColor(telem.sos ? COL_RED : COL_DIMTEXT);
    sp.setCursor(20, 105);
    sp.print(telem.sos ? "!! SOS ACTIVE !!" : "SOS: INACTIVE");

    sp.setTextColor(telem.wearLink ? COL_GREEN : COL_RED);
    sp.setCursor(195, 105);
    sp.printf("WEAR: %s", telem.wearLink ? "OK" : "LOST");

    sp.setCursor(20, 120);
    sp.setTextColor(COL_DIMTEXT);
    sp.printf("FALL STATE: %d", telem.fallState);

    sp.pushSprite(15, 85);
    sp.deleteSprite();
}

void drawRadarPanel() {
    LGFX_Sprite sp(&lcd);
    sp.setColorDepth(16);
    sp.setPsram(true);
    sp.createSprite(370, 145);
    sp.fillSprite(COL_BG);

    if (radarEnabled && telem.radarPresent && telem.radarTargets > 0) {
        sp.fillRoundRect(15, 10, 340, 50, 8, COL_PANEL);
        sp.drawRoundRect(15, 10, 340, 50, 8, getThemeColor());
        sp.setTextSize(3);
        sp.setTextColor(getThemeColor());
        const char* lbl = langTR ? "HEDEF" : "TARGET";
        int tw = strlen(lbl) * 18;
        sp.setCursor(15 + (340 - tw) / 2, 22);
        sp.print(lbl);

        sp.setTextSize(2);
        sp.setTextColor(COL_BRIGHT);
        float dist_m = telem.radarDist_mm / 1000.0f;
        sp.setCursor(20, 75);
        sp.printf("DIST: %.2f m", dist_m);
        sp.setCursor(20, 100);
        sp.printf("SPD:  %d cm/s", abs(telem.radarSpeed));

        sp.setTextSize(1);
        sp.setTextColor(COL_DIMTEXT);
        sp.setCursor(20, 125);
        sp.printf("TARGETS: %d", telem.radarTargets);
    } else if (radarEnabled && fusionOnline) {
        sp.fillRoundRect(15, 10, 340, 50, 8, COL_BG);
        sp.drawRoundRect(15, 10, 340, 50, 8, COL_BORDER);
        sp.setTextSize(2);
        sp.setTextColor(COL_DIMTEXT);
        const char* lbl = langTR ? "TARANIYOR" : "SCANNING";
        int tw = strlen(lbl) * 12;
        sp.setCursor(15 + (340 - tw) / 2, 27);
        sp.print(lbl);
    } else {
        sp.setTextSize(2);
        sp.setTextColor(COL_DIMTEXT);
        sp.setCursor(130, 60);
        sp.print("OFFLINE");
    }

    sp.pushSprite(415, 85);
    sp.deleteSprite();
}

void drawAccelPanel() {
    LGFX_Sprite sp(&lcd);
    sp.setColorDepth(16);
    sp.setPsram(true);
    sp.createSprite(770, 155);
    sp.fillSprite(COL_BG);

    float gxv = toG(telem.ax);
    float gyv = toG(telem.ay);
    float gzv = toG(telem.az);

    auto drawBar = [&](int x, int y, float g, const char* label) {
        sp.setTextSize(2);
        sp.setTextColor(COL_TEXT);
        sp.setCursor(x, y);
        sp.print(label);

        int bx = x + 85, bw = 140, bh = 14;
        sp.drawRect(bx, y + 2, bw, bh, COL_BORDER);

        int center = bx + bw / 2;
        int extent = (int)(constrain(g, -2.0f, 2.0f) * (bw / 4));
        uint16_t col = (abs(g) > 1.5f) ? COL_ORANGE : COL_ACCENT;
        if (extent > 0) sp.fillRect(center, y + 4, extent, bh - 4, col);
        else if (extent < 0) sp.fillRect(center + extent, y + 4, -extent, bh - 4, col);

        char vbuf[12];
        snprintf(vbuf, sizeof(vbuf), "%+.2fg", g);
        sp.setCursor(bx + bw + 10, y + 1);
        sp.setTextColor(COL_BRIGHT);
        sp.print(vbuf);
    };

    drawBar(15, 20, gxv, "X-AXIS");
    drawBar(15, 55, gyv, "Y-AXIS");
    drawBar(15, 90, gzv, "Z-AXIS");

    sp.setTextSize(1);
    sp.setTextColor(COL_DIMTEXT);
    char gyroBuf[24];
    snprintf(gyroBuf, sizeof(gyroBuf), "GX: %+7.1f dps", toDPS(telem.gx));
    sp.setCursor(500, 25);
    sp.print(gyroBuf);
    snprintf(gyroBuf, sizeof(gyroBuf), "GY: %+7.1f dps", toDPS(telem.gy));
    sp.setCursor(500, 45);
    sp.print(gyroBuf);
    snprintf(gyroBuf, sizeof(gyroBuf), "GZ: %+7.1f dps", toDPS(telem.gz));
    sp.setCursor(500, 65);
    sp.print(gyroBuf);

    float total = sqrt(gxv * gxv + gyv * gyv + gzv * gzv);
    sp.setTextSize(2);
    sp.setTextColor(total > 2.5f ? COL_RED : COL_ACCENT);
    char totalBuf[16];
    snprintf(totalBuf, sizeof(totalBuf), "|G| %.2f", total);
    sp.setCursor(500, 90);
    sp.print(totalBuf);

    sp.pushSprite(15, 280);
    sp.deleteSprite();
}

void drawBottomBar() {
    LGFX_Sprite sp(&lcd);
    sp.setColorDepth(16);
    sp.setPsram(true);
    sp.createSprite(SCR_W, 35);
    sp.fillSprite(COL_HEADER);
    sp.drawFastHLine(0, 0, SCR_W, COL_BORDER);

    sp.setTextSize(2);
    int x = 15;

    if (telem.wearLink) { sp.fillRoundRect(x, 5, 75, 24, 4, COL_GREEN); sp.setTextColor(0x0000); }
    else                { sp.fillRoundRect(x, 5, 75, 24, 4, COL_RED);   sp.setTextColor(COL_BRIGHT); }
    sp.setCursor(x + 6, 9);
    sp.print("WEAR");

    x += 95;
    if (telem.radarPresent) { sp.fillRoundRect(x, 5, 65, 24, 4, COL_ACCENT); sp.setTextColor(0x0000); }
    else                    { sp.fillRoundRect(x, 5, 65, 24, 4, COL_PANEL);  sp.setTextColor(COL_DIMTEXT); }
    sp.setCursor(x + 8, 9);
    sp.print("RAD");

    x += 85;
    if (fusionOnline) { sp.fillRoundRect(x, 5, 75, 24, 4, COL_GREEN); sp.setTextColor(0x0000); }
    else              { sp.fillRoundRect(x, 5, 75, 24, 4, COL_RED);   sp.setTextColor(COL_BRIGHT); }
    sp.setCursor(x + 6, 9);
    sp.print("FUSE");

    x += 95;
    if (bathOnline && millis() - lastBathTime < 5000) {
        uint16_t bathBg = (bathFallState == 2) ? COL_RED : (bathPresence > 0 ? COL_ACCENT : COL_PANEL);
        uint16_t bathFg = (bathFallState == 2) ? COL_BRIGHT : (bathPresence > 0 ? 0x0000 : COL_DIMTEXT);
        sp.fillRoundRect(x, 5, 75, 24, 4, bathBg);
        sp.setTextColor(bathFg);
    } else {
        sp.fillRoundRect(x, 5, 75, 24, 4, COL_PANEL);
        sp.setTextColor(COL_DIMTEXT);
    }
    sp.setCursor(x + 6, 9);
    sp.print("BATH");

    x += 95;
    bool wifiOk = (WiFi.status() == WL_CONNECTED);
    sp.fillRoundRect(x, 5, 65, 24, 4, wifiOk ? COL_GREEN : COL_PANEL);
    sp.setTextColor(wifiOk ? 0x0000 : COL_DIMTEXT);
    sp.setCursor(x + 8, 9);
    sp.print("WiFi");

    unsigned long sec = millis() / 1000;
    char uptBuf[16];
    snprintf(uptBuf, sizeof(uptBuf), "%02lu:%02lu:%02lu", sec / 3600, (sec / 60) % 60, sec % 60);
    sp.setTextColor(COL_DIMTEXT);
    sp.setCursor(SCR_W - 120, 10);
    sp.print(uptBuf);

    sp.pushSprite(0, SCR_H - 35);
    sp.deleteSprite();
}

void cancelAlert();

void drawAlertScreen() {
    unsigned long elapsed = millis() - stateStart;
    int remaining = max(0, (int)((alertTimeout - elapsed) / 1000));

    static int lastRem = -1;
    if (remaining != lastRem) {
        lastRem = remaining;

        LGFX_Sprite sp(&lcd);
        sp.setColorDepth(16);
        sp.setPsram(true);
        sp.createSprite(SCR_W, SCR_H);
        sp.fillSprite(0x0000);

        sp.drawRoundRect(50, 80, SCR_W - 100, SCR_H - 160, 12, COL_ORANGE);
        sp.drawRoundRect(52, 82, SCR_W - 104, SCR_H - 164, 12, COL_ORANGE);

        sp.setTextSize(3);
        sp.setTextColor(COL_ORANGE);
        const char* msg = langTR ? "DUSME TESPITI" : "IMPACT DETECTED";
        int tw = strlen(msg) * 18;
        sp.setCursor((SCR_W - tw) / 2, 110);
        sp.print(msg);

        sp.setTextSize(8);
        sp.setTextColor(COL_ORANGE);
        char numBuf[4];
        snprintf(numBuf, sizeof(numBuf), "%d", remaining);
        int nw = strlen(numBuf) * 48;
        sp.setCursor((SCR_W - nw) / 2, 200);
        sp.print(numBuf);

        sp.setTextSize(2);
        sp.setTextColor(COL_BRIGHT);
        char impBuf[24];
        snprintf(impBuf, sizeof(impBuf), "Impact: %.1fg", fallImpact);
        int iw = strlen(impBuf) * 12;
        sp.setCursor((SCR_W - iw) / 2, 310);
        sp.print(impBuf);

        sp.setTextColor(COL_DIMTEXT);
        const char* sub = langTR ? "IPTAL ICIN DOKUN" : "TOUCH OR PRESS TO CANCEL";
        int sw = strlen(sub) * 12;
        sp.setCursor((SCR_W - sw) / 2, SCR_H - 100);
        sp.print(sub);

        sp.pushSprite(0, 0);
        sp.deleteSprite();
    }

    static unsigned long beepStart = 0;
    if (!muted && millis() - lastBeep > (unsigned long)(300 - min(250UL, elapsed / 200))) {
        lastBeep = millis();
        beepStart = millis();
        setBuzzer(true);
    }
    if (beepStart > 0 && millis() - beepStart > 100) {
        setBuzzer(false);
        beepStart = 0;
    }

    if (touched || isrBtnShortPress) {
        isrBtnShortPress = false;
        cancelAlert();
        return;
    }

    if (elapsed >= (unsigned long)alertTimeout) {
        curState = ST_DANGER;
        stateStart = millis();
        lastRem = -1;
    }
}

void drawDangerScreen() {
    static int frame = 0;
    static unsigned long lastFrame = 0;
    unsigned long elapsed = millis() - stateStart;

    if (millis() - lastFrame < 200) return;
    lastFrame = millis();

    LGFX_Sprite sp(&lcd);
    sp.setColorDepth(16);
    sp.setPsram(true);
    sp.createSprite(SCR_W, SCR_H);
    sp.fillSprite(0x0000);

    uint16_t borderCol = (frame % 2) ? COL_RED : 0x0000;
    for (int i = 0; i < 8; i++) {
        sp.drawRect(i, i, SCR_W - 2 * i, SCR_H - 2 * i, borderCol);
    }

    sp.setTextSize(5);
    sp.setTextColor(COL_RED);
    const char* msg = langTR ? "TEHLIKE" : "DANGER";
    int tw = strlen(msg) * 30;
    sp.setCursor((SCR_W - tw) / 2, 150);
    sp.print(msg);

    sp.setTextSize(3);
    sp.setTextColor(COL_BRIGHT);
    const char* sub = langTR ? "ACIL DURUM!" : "EMERGENCY!";
    int sw = strlen(sub) * 18;
    sp.setCursor((SCR_W - sw) / 2, 250);
    sp.print(sub);

    sp.setTextSize(2);
    sp.setTextColor(COL_DIMTEXT);
    const char* cancel = langTR ? "IPTAL ICIN DOKUN" : "TOUCH OR PRESS TO CANCEL";
    int cw = strlen(cancel) * 12;
    sp.setCursor((SCR_W - cw) / 2, 350);
    sp.print(cancel);

    sp.pushSprite(0, 0);
    sp.deleteSprite();

    if (!muted) setBuzzer(frame % 2);
    frame++;

    if (touched || isrBtnShortPress) {
        isrBtnShortPress = false;
        cancelAlert();
        frame = 0;
        return;
    }

    if (elapsed >= 60000) {
        cancelAlert();
        frame = 0;
    }
}

void cancelAlert() {
    fallDetected = false;
    fallImpact = 0;
    
    Wire.beginTransmission(FUSION_I2C_ADDR);
    Wire.write(0xEE);
    Wire.endTransmission();
    setBuzzer(false);

    LGFX_Sprite sp(&lcd);
    sp.setColorDepth(16);
    sp.setPsram(true);
    sp.createSprite(SCR_W, SCR_H);
    sp.fillSprite(0x0000);

    sp.setTextSize(3);
    sp.setTextColor(COL_GREEN);
    const char* msg = langTR ? "ALARM TEMIZLENDI" : "ALARM CLEARED";
    int tw = strlen(msg) * 18;
    sp.setCursor((SCR_W - tw) / 2, 200);
    sp.print(msg);

    sp.setTextSize(2);
    sp.setTextColor(COL_DIMTEXT);
    sp.setCursor((SCR_W - 180) / 2, 260);
    sp.print("Returning to monitor...");

    sp.pushSprite(0, 0);
    sp.deleteSprite();

    curState = ST_COOLDOWN;
    stateStart = millis();
}

void drawSettingsMenu() {
    static unsigned long lastDraw = 0;
    
    if (touched && millis() - lastTouch > 250) {
        lastTouch = millis();
        int boxH = 50;
        int gap = 15;
        int startY = 80;
        
        for (int i = 0; i < 6; i++) {
            int itemY = startY + (i * (boxH + gap));
            if (touchY > itemY && touchY < itemY + boxH) {
                menuIndex = i;
                toggleSetting(i);
                if (curState != ST_SETTINGS) return;
                lastDraw = 0; 
            }
        }
    }

    if (millis() - lastDraw < 50) return;
    lastDraw = millis();
    
    LGFX_Sprite sp(&lcd);
    sp.setColorDepth(16);
    sp.setPsram(true);
    sp.createSprite(SCR_W, SCR_H);
    sp.fillSprite(COL_BG);
    
    sp.setTextSize(4);
    sp.setTextColor(getThemeColor());
    sp.setCursor(20, 20);
    sp.print(langTR ? "AYARLAR" : "SETTINGS");
    
    int boxH = 50;
    int startY = 80;
    int gap = 15;
    
    auto drawItem = [&](int idx, const char* label, String val, uint16_t valColor) {
        int y = startY + (idx * (boxH + gap));
        bool isActive = (idx == menuIndex);
        
        if (isActive) {
            sp.fillRoundRect(20, y, SCR_W - 40, boxH, 8, getThemeColor());
            sp.drawRoundRect(20, y, SCR_W - 40, boxH, 8, COL_BRIGHT);
            sp.setTextColor(COL_BG);
        } else {
            sp.fillRoundRect(20, y, SCR_W - 40, boxH, 8, COL_PANEL);
            sp.drawRoundRect(20, y, SCR_W - 40, boxH, 8, getThemeColor());
            sp.setTextColor(COL_BRIGHT);
        }
        
        sp.setTextSize(3);
        sp.setCursor(40, y + 14);
        sp.print(label);
        
        if (val.length() > 0) {
            if (isActive) sp.setTextColor(COL_BG);
            else sp.setTextColor(valColor);
            
            int valW = val.length() * 18;
            sp.setCursor(SCR_W - 40 - valW - 20, y + 14);
            sp.print(val);
        }
    };
    
    String muteVal = muted ? "[ ON ]" : "[ OFF ]";
    drawItem(0, langTR ? "ALARMLARI SUSTUR" : "MUTE ALARMS", muteVal, muted ? COL_GREEN : COL_RED);
    String radarVal = radarEnabled ? "[ ON ]" : "[ OFF ]";
    drawItem(1, "RADAR SENSOR", radarVal, radarEnabled ? COL_GREEN : COL_RED);
    String mpuVal = mpuEnabled ? "[ ON ]" : "[ OFF ]";
    drawItem(2, "MPU SENSOR", mpuVal, mpuEnabled ? COL_GREEN : COL_RED);
    
    String themeName = "[ CYAN ]";
    if (themeIndex == 0) themeName = langTR ? "[ TURKUAZ ]" : "[ CYAN ]";
    if (themeIndex == 1) themeName = langTR ? "[ YESIL ]" : "[ GREEN ]";
    if (themeIndex == 2) themeName = langTR ? "[ MOR ]" : "[ MAGENTA ]";
    if (themeIndex == 3) themeName = langTR ? "[ SARI ]" : "[ YELLOW ]";
    if (themeIndex == 4) themeName = langTR ? "[ TURUNCU ]" : "[ ORANGE ]";
    if (themeIndex == 5) themeName = langTR ? "[ MAVI ]" : "[ BLUE ]";
    drawItem(3, langTR ? "TEMA RENGI" : "THEME COLOR", themeName, getThemeColor());
    
    drawItem(4, langTR ? "DIL / LANGUAGE" : "LANGUAGE / DIL", langTR ? "[ TR ]" : "[ EN ]", getThemeColor());
    drawItem(5, langTR ? "KAYDET & CIK" : "SAVE & EXIT", "", COL_BRIGHT);

    sp.pushSprite(0, 0);
    sp.deleteSprite();
}

void drawBootScreen() {
    LGFX_Sprite sp(&lcd);
    sp.setColorDepth(16);
    sp.setPsram(true);
    sp.createSprite(SCR_W, SCR_H);
    sp.fillSprite(COL_BG);

    sp.drawRoundRect(40, 30, SCR_W - 80, SCR_H - 60, 12, COL_BORDER);
    sp.drawRoundRect(42, 32, SCR_W - 84, SCR_H - 64, 12, COL_BORDER);

    sp.setTextSize(5);
    sp.setTextColor(getThemeColor());
    const char* title = "RANGER";
    int titleW = strlen(title) * 30;
    sp.setCursor((SCR_W - titleW) / 2, 70);
    sp.print(title);

    sp.setTextSize(2);
    sp.setTextColor(COL_TEXT);
    char versionBuf[40];
    snprintf(versionBuf, sizeof(versionBuf), "v%s", VERSION);
    int versionW = strlen(versionBuf) * 12;
    sp.setCursor((SCR_W - versionW) / 2, 135);
    sp.print(versionBuf);

    sp.drawFastHLine(100, 170, SCR_W - 200, COL_BORDER);

    sp.setTextSize(2);
    sp.setTextColor(COL_DIMTEXT);
    sp.setCursor(120, 195);
    sp.print("CrowPanel Advance 4.3\" V1.0");
    sp.setCursor(120, 225);
    sp.print("800x480 ST7265 RGB | ESP32-S3");

    sp.setCursor(120, 265);
    sp.printf("CPU: %d MHz    PSRAM: %d KB", ESP.getCpuFreqMHz(), ESP.getPsramSize() / 1024);
    sp.setCursor(120, 295);
    sp.printf("Heap: %d KB   Build: %s", ESP.getFreeHeap() / 1024, BUILD_DATE);

    sp.drawFastHLine(100, 330, SCR_W - 200, COL_BORDER);

    sp.setTextSize(2);
    sp.setTextColor(COL_TEXT);
    sp.setCursor(120, 350);
    sp.print("TFT......");
    sp.setCursor(120, 380);
    sp.print("I2C......");
    sp.setCursor(420, 350);
    sp.print("FUSION...");
    sp.setCursor(420, 380);
    sp.print("TOUCH....");

    sp.pushSprite(0, 0);
    sp.deleteSprite();

    delay(200);
    lcd.setTextSize(2);
    lcd.setTextColor(COL_GREEN, COL_BG);
    lcd.setCursor(240, 350);
    lcd.print("OK");

    delay(300);
    lcd.setTextColor(COL_GREEN, COL_BG);
    lcd.setCursor(240, 380);
    lcd.print("OK");

    delay(300);
    bool got = pollFusion();
    lcd.setTextColor(got ? COL_GREEN : COL_YELLOW, COL_BG);
    lcd.setCursor(550, 350);
    lcd.print(got ? "OK" : "WAIT");

    lcd.setTextColor(COL_GREEN, COL_BG);
    lcd.setCursor(550, 380);
    lcd.print("OK");

    delay(1000);
}

void setup() {
    Serial0.begin(115200);
    
    Wire.begin(15, 16);
    Wire.setClock(400000);
    delay(50);

    // Initialize TCA9534 I2C Expander
    ioex.attach(Wire);
    ioex.setDeviceAddress(TCA9534_ADDR);
    ioex.config(1, TCA9534::Config::OUT);
    ioex.config(2, TCA9534::Config::OUT);
    ioex.output(1, TCA9534::Level::H); // Enable Display Backlight

    // GT911 Touch Controller Power-on Reset Sequence
    pinMode(1, OUTPUT);
    digitalWrite(1, LOW);
    ioex.output(2, TCA9534::Level::L);
    delay(20);
    ioex.output(2, TCA9534::Level::H);
    delay(100);
    pinMode(1, INPUT);

    lcd.init();
    lcd.initDMA();
    lcd.startWrite();
    lcd.fillScreen(TFT_BLACK);

    pinMode(BTN_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BTN_PIN), btnISR, CHANGE);
    setBuzzer(false);

    EEPROM.begin(EEPROM_SIZE);
    loadSettings();

    curState = ST_BOOT;
    stateStart = millis();
    drawBootScreen();

    curState = ST_MONITOR;
    stateStart = millis();
    drawBackground();

    initEspNow();
    
    // Core 0 Wi-Fi & Telegram task
    xTaskCreatePinnedToCore(networkTask, "NetTask", 8192, NULL, 1, NULL, 0);
}

void loop() {
    unsigned long now = millis();

    // 1. Poll Fusion Node @ 20Hz
    static unsigned long lastPoll = 0;
    if (now - lastPoll >= 50) {
        lastPoll = now;
        if (!pollFusion()) {
            if (now - lastTelemTime > 5000) {
                fusionOnline = false;
            }
        }

        float gxv = toG(telem.ax);
        float gyv = toG(telem.ay);
        float gzv = toG(telem.az);
        float localImpact = sqrt(gxv * gxv + gyv * gyv + gzv * gzv);
        
        bool fusionFall = (telem.fallState == 2);

        if (fusionFall && curState == ST_MONITOR && !fallDetected) {
            fallDetected = true;
            fallImpact = localImpact;
            alertTimeout = 30000;
            curState = ST_ALERT;
            stateStart = now;
            
            triggerTelegram = true;
            telegramImpact = fallImpact;
        }
    }

    // 2. Poll Capacitive Touch
    static unsigned long lastTouchPoll = 0;
    if (now - lastTouchPoll >= 100) {
        lastTouchPoll = now;
        pollTouch();
    }

    // 3. Process Button Events
    if (isrBtnShortPress) {
        isrBtnShortPress = false;
        if (curState == ST_ALERT || curState == ST_DANGER) {
            cancelAlert();
        } else if (curState == ST_SETTINGS) {
            menuIndex = (menuIndex + 1) % 6;
        } else if (curState == ST_MONITOR) {
            curState = ST_SETTINGS;
            menuIndex = 0;
        }
    }
    
    if (isrBtnLongPress) {
        isrBtnLongPress = false;
        if (curState == ST_SETTINGS) {
            toggleSetting(menuIndex);
        }
    }

    // 4. Ingest Bathroom Node ESP-NOW Fall Trigger
    if (bathOnline && bathFallState == 2 && curState == ST_MONITOR && !fallDetected) {
        fallDetected = true;
        fallImpact = 0;
        alertTimeout = 30000;
        curState = ST_ALERT;
        stateStart = now;
        triggerTelegram = true;
        telegramImpact = 0;
    }

    if (bathOnline && millis() - lastBathTime > 10000) {
        bathOnline = false;
    }

    // 5. GUI State Machine
    switch (curState) {
        case ST_MONITOR: {
            if (touched && touchX > SCR_W - 70 && touchY < 60) {
                curState = ST_SETTINGS;
                stateStart = now;
                touched = false;
                break;
            }
            static unsigned long lastHeader = 0;
            static unsigned long lastStatus = 0;
            static unsigned long lastRadar  = 0;
            static unsigned long lastBottom = 0;

            if (now - lastHeader >= 2000) { lastHeader = now; drawHeader(); }
            if (now - lastStatus >= 1000) { lastStatus = now; drawStatusPanel(); }
            if (now - lastRadar  >= 1000) { lastRadar  = now; drawRadarPanel(); drawAccelPanel(); }
            if (now - lastBottom >= 2000) { lastBottom = now; drawBottomBar(); }
            break;
        }

        case ST_ALERT:
            drawAlertScreen();
            break;

        case ST_DANGER:
            drawDangerScreen();
            break;
            
        case ST_SETTINGS:
            drawSettingsMenu();
            break;

        case ST_COOLDOWN:
            if (now - stateStart >= 1500) {
                curState = ST_MONITOR;
                stateStart = now;
                drawBackground();
            }
            break;

        default:
            break;
    }
}
