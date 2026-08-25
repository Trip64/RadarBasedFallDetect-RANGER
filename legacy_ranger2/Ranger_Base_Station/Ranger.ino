/**
 * Fall Detection Base Station - ESP32-S3
 * 
 * Fall Detection System with:
 * - WiFi AP + TCP Server for sensor node
 * - Radar (RD-03D) 24GHz mmWave multi-target tracking
 * - TFT Display for status GUI
 * - Escalating alert system
 * - Telegram notifications
 * 
 * Hardware Pins:
 * - TFT: CS=10, DC=9, RST=8, SCK=12, MOSI=11
 * - Buzzers: 15, 16
 * - White LED: 1 (PWM)
 * - Cancel Button: 14
 * - Radar: RX=4, TX=5 (RD-03D @ 256000 baud)
 */

#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <WiFiServer.h>
#include <esp_now.h>
#include <HTTPClient.h>  // For Telegram/Email
#include <time.h>        // For NTP time sync
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <HardwareSerial.h>
#include <esp_wifi.h>

#define VERSION "2.3-RD03D"
#define BUILD_DATE "2026-04-27"

// ============== PINS ==============
#define TFT_CS   10
#define TFT_DC   9
#define TFT_RST  8
#define TFT_SCK  12
#define TFT_MOSI 11

#define PIN_BUZZER1  16
#define PIN_BUZZER2  15
#define PIN_LED_W    1
#define PIN_BUTTON   14
#define PIN_RADAR_RX 4
#define PIN_RADAR_TX 5

// ============== CONFIGURATION ==============
const char* AP_SSID = "FALLDETECT_BASE";
const char* AP_PASS = "falldetect1234";
const int TCP_PORT = 8080;

const char* EMERGENCY_PHONE = "+1234567890";
const char* EMERGENCY_MSG = "ALERT: Fall detected! Please check immediately.";

#define MAX_ALERT_TIME 30000 
#define DANGER_DURATION 60000

// Home WiFi (for internet access - AP+STA dual mode)
const char* HOME_SSID = "YOUR_HOME_WIFI_SSID";
const char* HOME_PASS = "YOUR_HOME_WIFI_PASSWORD";

// Telegram Bot (Free push notifications)
const char* TELEGRAM_TOKEN = "YOUR_TELEGRAM_BOT_TOKEN";
const char* TELEGRAM_CHAT_ID = "YOUR_TELEGRAM_CHAT_ID";

// Email SMTP (Gmail with App Password)
const char* EMAIL_SMTP = "smtp.gmail.com";
const int EMAIL_PORT = 465;
const char* EMAIL_USER = "";  // your@gmail.com
const char* EMAIL_PASS = "";  // App password (not regular password)
const char* EMAIL_TO = "";    // Recipient email

// ============== HARDWARE ==============
Adafruit_ILI9341 tft = Adafruit_ILI9341(&SPI, TFT_DC, TFT_CS, TFT_RST);
HardwareSerial radarSerial(1);
WiFiServer server(TCP_PORT);
WiFiClient sensorClient;

#define W 320
#define H 240

// ============== LANGUAGE CONFIG ==============
bool languageTR = false; // Runtime flag

// Ternary macros for runtime switching
#define STR_SYSTEM_STATUS   (languageTR ? "SISTEM DURUMU" : "SYSTEM STATUS")
#define STR_RADAR_TRACKING  (languageTR ? "RADAR IZLEME" : "RADAR TRACKING")
#define STR_GRAVITY_SENSOR  (languageTR ? "YER C. SENSORU" : "GRAVITY SENSOR (MPU)")
#define STR_SECURE          (languageTR ? "GUVENLI" : "SECURE")
#define STR_NO_LINK         (languageTR ? "SINYAL YOK" : "NO LINK")
#define STR_TARGET          (languageTR ? "HEDEF" : "TARGET")
#define STR_SCANNING        (languageTR ? "TARANIYOR" : "SCANNING")
#define STR_AREA_CLEAR      (languageTR ? "BOLGE BOS" : "AREA CLEAR")
#define STR_WARNING         (languageTR ? "UYARI" : "WARNING")
#define STR_IMPACT_DETECTED (languageTR ? "DUSME TESPITI" : "IMPACT DETECTED")
#define STR_CANCEL_MSG      (languageTR ? "IPTAL ICIN BUTONA BAS" : "PRESS BUTTON TO CANCEL")
#define STR_CRITICAL        (languageTR ? "KRITIK" : "CRITICAL")
#define STR_DANGER          (languageTR ? "TEHLIKE" : "DANGER")
#define STR_SMS_SENT        (languageTR ? "ACIL SMS GONDERILDI" : "EMERGENCY SMS SENT")
#define STR_STATUS          (languageTR ? "DURUM" : "STATUS")
#define STR_ALARM_CLEARED   (languageTR ? "ALARM TEMIZLENDI" : "ALARM CLEARED")
#define STR_SYS_RESTORED    (languageTR ? "Sistem Geri Yuklendi" : "System Restored")
#define STR_SETTINGS        (languageTR ? "AYARLAR" : "SETTINGS")
#define STR_MUTE_ALARMS     (languageTR ? "ALARMLARI SUSTUR:" : "MUTE ALARMS: ")
#define STR_LANG_OPT        (languageTR ? "DIL / LANGUAGE:" : "LANGUAGE / DIL:")
#define STR_EXIT            (languageTR ? "CIKIS" : "EXIT")
#define STR_NAV_HELP        (languageTR ? "Kisa: Gez | Uzun: Sec" : "Short: Navigate | Hold: Select")
#define STR_DIAGNOSTICS     (languageTR ? "TANI" : "DIAGNOSTICS")
#define STR_SKIPPING        (languageTR ? "GECILIYOR >>" : "SKIPPING >>")
#define STR_WIFI_INIT       (languageTR ? "1. WiFi AP Baslat" : "1. WiFi AP Init")
#define STR_SENSOR_NODE     (languageTR ? "2. Sensor Node" : "2. Sensor Node")
#define STR_RADAR_MOD       (languageTR ? "3. Radar (RD-03D)" : "3. Radar (RD-03D)")
#define STR_AV_CHECK        (languageTR ? "4. Ses/Goruntu" : "4. Audio/Visual")
#define STR_SYS_DETAILS     (languageTR ? "SISTEM DETAYLARI" : "SYSTEM DETAILS")
#define STR_ON              (languageTR ? "ACIK " : "ON ")
#define STR_OFF             (languageTR ? "KAPALI" : "OFF")
#define STR_HEADER_TITLE    (languageTR ? "DUSME IZLEME" : "FALL MONITOR")

// Colors - Tactical Theme
#define BLACK   0x0000
#define WHITE   0xFFFF
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define YELLOW  0xFFE0
#define ORANGE  0xFD20
#define GRAY    0x7BEF
#define DGRAY   0x2124 // Darker gray for backgrounds
#define BLUE    0x001F
#define NAVY    0x000F

// ============== STATE MACHINE ==============
enum SystemState {
    STATE_BOOT,
    STATE_SELFTEST,
    STATE_MONITORING,
    STATE_SETTINGS, // New
    STATE_ALERT,
    STATE_DANGER,
    STATE_COOLDOWN
};

SystemState currentState = STATE_BOOT;
SystemState lastState = STATE_BOOT;
unsigned long stateStartTime = 0;

// ============== SETTINGS ==============
bool buzzersMuted = false;
int menuIndex = 0;
const int menuItems = 7; // Mute, Radar, MPU, Radar DBG, Theme, Language, Exit
bool radarEnabled = true;
bool mpuEnabled = true;
bool radarSerialDebug = false; // Serial output of radar values
int themeIndex = 0; // 0=CYAN, 1=GREEN, 2=MAGENTA

#define EEPROM_SIZE 512

void saveSettings() {
    EEPROM.write(0, buzzersMuted ? 1 : 0);
    EEPROM.write(1, radarEnabled ? 1 : 0);
    EEPROM.write(2, themeIndex);
    EEPROM.write(3, languageTR ? 1 : 0);
    EEPROM.write(4, mpuEnabled ? 1 : 0);
    EEPROM.write(5, radarSerialDebug ? 1 : 0);
    EEPROM.commit();
}

void loadSettings() {
    if (EEPROM.read(0) != 255) { // Check if initialized
        buzzersMuted = (EEPROM.read(0) == 1);
        radarEnabled = (EEPROM.read(1) == 1);
        themeIndex = EEPROM.read(2);
        if (themeIndex > 5) themeIndex = 0;
        languageTR = (EEPROM.read(3) == 1);
        mpuEnabled = (EEPROM.read(4) == 1);
        radarSerialDebug = (EEPROM.read(5) == 1);
    }
}

// Theme Colors
uint16_t getThemeColor() {
    switch (themeIndex) {
        case 0: return CYAN;
        case 1: return GREEN;
        case 2: return 0xF81F; // MAGENTA
        case 3: return YELLOW;
        case 4: return ORANGE;
        case 5: return 0x001F; // BLUE
        default: return CYAN;
    }
}

// ============== SENSOR DATA ==============
float mpu_ax = 0, mpu_ay = 0, mpu_az = 0;
float mpu_gx = 0, mpu_gy = 0, mpu_gz = 0;
int mpu_fall = 0;
float mpu_impact = 0;
float mpu_orient = 0;  // Peak orientation change from sensor node
uint8_t mpu_mlFall = 0;   // ML fall confidence 0-100%
uint8_t mpu_mlConf = 0;   // ML winner confidence 0-100%
uint8_t mpu_mlFlags = 0;  // bit0=SOS
volatile bool sensorConnected = false;
unsigned long lastSensorData = 0;

// ============== ESP-NOW PACKET (must match WatchNode) ==============
#pragma pack(push, 1)
struct SensorPacket {
    uint8_t  magic;      // 0xFA
    int16_t  ax, ay, az; // raw accel (4096 LSB/g)
    int16_t  gx, gy, gz; // raw gyro (65.5 LSB/dps)
    uint8_t  fall;       // 0=none, 1=possible, 2=confirmed
    uint8_t  impact;     // impactMagnitude * 10 (0-255)
    uint8_t  orient;     // peakOrient / 2 (0-255 = 0-510 dps)
    uint8_t  mlFallScore; // ML fall confidence 0-100%
    uint8_t  mlConf;     // ML winner confidence 0-100%
    uint8_t  mlFlags;    // bit0=SOS
    uint8_t  seq;        // packet counter
};
#pragma pack(pop)

// ESP-NOW receive callback — runs on WiFi task (Core 0)
void onESPNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (len < 20) return; // Accept both old (20) and new (24) packets
    const SensorPacket* pkt = (const SensorPacket*)data;
    if (pkt->magic != 0xFA) return;
    
    // Convert raw values to physical units (same as WatchNode encoding)
    mpu_ax = pkt->ax / 4096.0f;
    mpu_ay = pkt->ay / 4096.0f;
    mpu_az = pkt->az / 4096.0f;
    mpu_gx = pkt->gx / 65.5f;
    mpu_gy = pkt->gy / 65.5f;
    mpu_gz = pkt->gz / 65.5f;
    mpu_fall = pkt->fall;
    mpu_impact = pkt->impact / 10.0f;
    mpu_orient = pkt->orient * 2.0f;
    
    // Parse ML scores if present (new 24-byte packet)
    if (len >= (int)sizeof(SensorPacket)) {
        mpu_mlFall = pkt->mlFallScore;
        mpu_mlConf = pkt->mlConf;
        mpu_mlFlags = pkt->mlFlags;
    }
    
    lastSensorData = millis();
    sensorConnected = true;
}

// WiFi AP event tracking
volatile bool apClientJoined = false;   // Set by WiFi event callback
volatile int  apClientCount = 0;        // Live count from events
bool forceIconRedraw = true;            // Forward-declared for handleNetworking

char radarBuffer[64];
int radarBufIdx = 0;
String radarStatus = "---";
float radarDistance = 0;
bool radarPresence = false;
int radarMsgs = 0;
int radarSpeed = 0;
int radarTargetCount = 0;

// ============== RADAR HISTORY (Circular Buffer) ==============
#define RADAR_HISTORY_SIZE 50
struct RadarSample {
    float distance;
    bool presence;
    unsigned long timestamp;
};
RadarSample radarHistory[RADAR_HISTORY_SIZE];
int radarHistIdx = 0;
int radarHistCount = 0;

void addRadarSample(float dist, bool pres) {
    radarHistory[radarHistIdx] = {dist, pres, millis()};
    radarHistIdx = (radarHistIdx + 1) % RADAR_HISTORY_SIZE;
    if (radarHistCount < RADAR_HISTORY_SIZE) radarHistCount++;
}

// Analyze radar history for fall signature
// Returns 0.0-1.0 confidence that a fall occurred
float analyzeRadarFallSignature() {
    if (radarHistCount < 10) return 0.0; // Not enough data
    
    unsigned long now = millis();
    float maxDist = 0, minDist = 999;
    bool hadPresence = false;
    bool lostPresence = false;
    float distSpike = 0;
    int recentIdx = 0;
    
    // Analyze last 3 seconds of radar data
    for (int i = 0; i < radarHistCount; i++) {
        int idx = (radarHistIdx - 1 - i + RADAR_HISTORY_SIZE) % RADAR_HISTORY_SIZE;
        RadarSample& s = radarHistory[idx];
        
        if (now - s.timestamp > 3000) break; // Only look at last 3s
        recentIdx++;
        
        if (s.distance > maxDist) maxDist = s.distance;
        if (s.distance < minDist && s.distance > 0.01) minDist = s.distance;
        if (s.presence) hadPresence = true;
    }
    
    // Check current state
    if (!radarPresence && hadPresence) lostPresence = true;
    distSpike = maxDist - minDist;
    
    float score = 0.0;
    
    // Presence→Absence transition (person went down / out of view)
    if (lostPresence) score += 0.4;
    
    // Significant distance change (>0.5m in 3s = someone fell)
    if (distSpike > 0.5) score += 0.3;
    if (distSpike > 1.0) score += 0.2; // Extra for large change
    
    // Rapid distance increase (person moving away / falling)
    if (maxDist > minDist * 1.5 && minDist > 0.1) score += 0.1;
    
    return constrain(score, 0.0, 1.0);
}

// ============== SENSOR FUSION ==============
float fusionScore = 0.0;    // Overall fusion confidence (0.0 - 1.0)
float radarFallScore = 0.0; // Radar-only fall confidence

float computeFusionScore() {
    float mpuScore = 0.0;
    
    // ML-driven MPU contribution (replaces raw threshold-based scoring)
    if (mpu_mlFall > 0) {
        // Use ML confidence directly as the primary signal
        mpuScore = mpu_mlFall / 100.0f;
    } else {
        // Fallback to threshold-based scoring if no ML data
        if (mpu_fall >= 2) mpuScore = 0.8;
        else if (mpu_fall == 1) mpuScore = 0.4;
    }
    
    // Boost for high impact (still useful alongside ML)
    if (mpu_impact > 5.0) mpuScore += 0.1;
    else if (mpu_impact > 3.0) mpuScore += 0.05;
    
    // Boost for orientation change (body rotation during fall)
    if (mpu_orient > 150.0) mpuScore += 0.1;
    else if (mpu_orient > 80.0) mpuScore += 0.05;
    
    mpuScore = constrain(mpuScore, 0.0, 1.0);
    
    // Radar contribution
    radarFallScore = analyzeRadarFallSignature();
    
    // Fusion: Weighted combination with cross-validation bonus
    float fused = 0.0;
    
    if (mpuEnabled && radarEnabled) {
        // Both sensors active: cross-validate
        fused = (mpuScore * 0.55) + (radarFallScore * 0.35);
        
        // Cross-validation bonus: if BOTH detect fall, confidence jumps
        if (mpuScore > 0.5 && radarFallScore > 0.3) {
            fused += 0.15; // Agreement bonus
        }
    } else if (mpuEnabled) {
        fused = mpuScore;  // MPU only
    } else if (radarEnabled) {
        fused = radarFallScore;  // Radar only
    }
    
    return constrain(fused, 0.0, 1.0);
}

// ============== FALL DETECTION ==============
bool fallDetected = false;
float fallSeverity = 0;
unsigned long fallTime = 0;
int alertTimeout = 30000;

// ============== ALERT STATE ==============
int alertPhase = 0;
unsigned long lastBeep = 0;
int beepFreq = 500;

// ============== SELF TEST ==============
int selfTestStep = 0;
unsigned long stepStartTime = 0;
bool stepCompleted = false;
bool selfTestFailed = false;

// ============== LED CONTROL ==============
void updateLedStatus() {
    static unsigned long lastUpdate = 0;
    static int brightness = 5;
    static int direction = 1;
    
    if (currentState == STATE_MONITORING) {
        if (millis() - lastUpdate > 30) {
            lastUpdate = millis();
            // Breathing effect (5 to 40) - subtle
            brightness += direction;
            if (brightness >= 40) direction = -1;
            if (brightness <= 5) direction = 1;
            ledcWrite(PIN_LED_W, brightness);
        }
    } else if (currentState == STATE_SETTINGS) {
        ledcWrite(PIN_LED_W, 10); // Steady dim
    }
    // Alert/Danger handle their own flashing
}

// Wrapper for tone to handle mute
void playTone(int pin, int freq, int dur) {
    if (!buzzersMuted) tone(pin, freq, dur);
}

// ============== RD-03D Protocol Constants ==============
const uint8_t RD03D_FRAME_HEADER[] = {0xAA, 0xFF, 0x03, 0x00};
const uint8_t RD03D_FRAME_FOOTER[] = {0x55, 0xCC};

int16_t rd03d_decode_value(uint8_t low_byte, uint8_t high_byte) {
    int16_t value = ((high_byte & 0x7F) << 8) | low_byte;
    if ((high_byte & 0x80) == 0) value = -value;
    return value;
}

bool rd03d_is_speed_valid(int16_t speed) {
    int16_t abs_speed = speed < 0 ? -speed : speed;
    return abs_speed != 248 && abs_speed != 256;
}

// WiFi STA reconnect (non-blocking, AP-safe)
unsigned long lastWifiReconnect = 0;
bool staGaveUp = false;
int staAttempts = 0;

void handleWifiReconnect() {
    if (WiFi.status() == WL_CONNECTED) { staAttempts = 0; return; }
    if (staGaveUp) return;
    if (strlen(HOME_SSID) == 0 || strcmp(HOME_SSID, "YOUR_WIFI_NAME") == 0) return;
    unsigned long now = millis();
    if (now - lastWifiReconnect < 30000) return;  // Try every 30s
    lastWifiReconnect = now;
    staAttempts++;
    if (staAttempts > 10) {
        staGaveUp = true;
        Serial.println("[WiFi] STA gave up after 10 attempts, AP-only mode");
        return;  // Don't call WiFi.disconnect — it kills the AP!
    }
    // Just call begin() again — safe in AP_STA mode, won't disrupt AP
    WiFi.begin(HOME_SSID, HOME_PASS);
    Serial.printf("[WiFi] STA reconnect attempt %d/10\n", staAttempts);
}

// ===================== WIFI EVENT HANDLERS =====================
void onAPClientConnect(WiFiEvent_t event, WiFiEventInfo_t info) {
    apClientCount = WiFi.softAPgetStationNum();
    apClientJoined = true;
    Serial.printf("[AP] Client joined! Total: %d\n", apClientCount);
}

void onAPClientDisconnect(WiFiEvent_t event, WiFiEventInfo_t info) {
    apClientCount = WiFi.softAPgetStationNum();
    Serial.printf("[AP] Client left. Total: %d\n", apClientCount);
    // NOTE: Do NOT call sensorClient.stop() here — this callback runs on
    // Core 0 (WiFi task) while the main loop reads sensorClient on Core 1.
    // Let handleNetworking() detect the disconnect safely on the main loop.
}

void onSTAGotIP(WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.printf("[WiFi] STA connected! IP: %s\n", WiFi.localIP().toString().c_str());
    staAttempts = 0;
}

void onSTADisconnect(WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.printf("[WiFi] STA disconnected (reason=%d)\n", info.wifi_sta_disconnected.reason);
}

void drawHeaderIcons(); // Forward declaration for drawHeader usage

// ============== HELPERS ==============
void clearScreen() {
    tft.fillScreen(BLACK);
}

void drawHeader(const char* title, uint16_t color) {
    tft.fillRect(0, 0, W, 28, DGRAY);
    tft.drawFastHLine(0, 28, W, color);
    tft.setTextSize(2);
    tft.setTextColor(color, DGRAY);
    tft.setCursor(10, 6);
    tft.print(title);
    
    // Draw status icons on the right side
    drawHeaderIcons();
}

void drawStatusBar() {
    tft.fillRect(0, H - 20, W, 20, DGRAY);
    tft.drawFastHLine(0, H - 21, W, GRAY);
    
    tft.setTextSize(1);
    
    // Sensor
    int x = 10;
    tft.setCursor(x, H - 14);
    if (sensorConnected) {
        tft.setTextColor(BLACK, GREEN);
        tft.print(" MPU ");
    } else {
        tft.setTextColor(WHITE, RED);
        tft.print(" MPU ");
    }
    
    // Radar
    x += 40;
    tft.setCursor(x, H - 14);
    if (radarMsgs > 0) {
        tft.setTextColor(BLACK, CYAN);
        tft.print(" RAD ");
    } else {
        tft.setTextColor(WHITE, DGRAY);
        tft.print(" RAD ");
    }
    
    // WiFi Clients
    x += 40;
    tft.setCursor(x, H - 14);
    if (WiFi.softAPgetStationNum() > 0) {
       tft.setTextColor(BLACK, GREEN);
       tft.printf("WiFi:%d", WiFi.softAPgetStationNum());
    } else {
       tft.setTextColor(WHITE, DGRAY);
       tft.printf("WiFi:0");
    }
    
    // Time
    unsigned long sec = millis() / 1000;
    char buf[16];
    snprintf(buf, sizeof(buf), "%02lu:%02lu", sec / 60, sec % 60);
    tft.setTextColor(GRAY, DGRAY);
    tft.setCursor(W - 40, H - 14);
    tft.print(buf);
    
    // Fusion Score
    x += 55;
    tft.setCursor(x, H - 14);
    uint16_t fuseColor = (fusionScore > 0.7) ? GREEN : (fusionScore > 0.4) ? ORANGE : GRAY;
    tft.setTextColor(fuseColor, DGRAY);
    tft.printf("FUSE:%2d%%", (int)(fusionScore * 100));
}

// Forward declarations
void handleNetworking();
void handleRadar();
void parseSensorData(const char* data);
void drawHeaderIcons();
void cancelAlert();

// ============== REAL SELF TEST (Compact & Fast) ==============
void runSelfTestInternal() {
    unsigned long now = millis();
    static bool headersDrawn = false;
    static unsigned long stepPassedTime = 0;
    
    // Draw static list once
    if (!headersDrawn) {
        tft.setTextSize(1);
        tft.setTextColor(GRAY, BLACK);
        tft.setCursor(10, 45); tft.println(STR_WIFI_INIT);
        tft.setCursor(10, 60); tft.println(STR_SENSOR_NODE);
        tft.setCursor(10, 75); tft.println(STR_RADAR_MOD);
        tft.setCursor(10, 90); tft.println(STR_AV_CHECK);
        
        // System Info Header
        tft.drawFastHLine(0, 130, W, DGRAY);
        tft.setTextColor(getThemeColor(), BLACK);
        tft.setCursor(10, 140); tft.print(STR_SYS_DETAILS);
        
        tft.setTextColor(GRAY, BLACK);
        tft.setCursor(10, 160); tft.printf("FW: %s  Build: %s", VERSION, BUILD_DATE);
        tft.setCursor(10, 175); tft.printf("Chip: ESP32-S3 (Rev %d)", ESP.getChipRevision());
        tft.setCursor(10, 190); tft.printf("CPU: %dMHz  Flash: %dMB", ESP.getCpuFreqMHz(), ESP.getFlashChipSize()/(1024*1024));
        tft.setCursor(10, 205); tft.printf("Heap: %dKB   MAC: %s", ESP.getFreeHeap()/1024, WiFi.macAddress().c_str());

        headersDrawn = true;
        stepStartTime = now;
        stepPassedTime = 0;
    }

    // Skip
    if (digitalRead(PIN_BUTTON) == LOW) {
        tft.setTextColor(YELLOW, BLACK);
        tft.setCursor(220, 140); // Moved skipp text
        tft.print(STR_SKIPPING);
        delay(300);
        currentState = STATE_MONITORING;
        headersDrawn = false;
        stepPassedTime = 0;
        return;
    }

    bool passed = false;
    bool waiting = false;
    int y = 45 + (selfTestStep * 15);

    // Timeout logic (Sensors get 5s, others fast)
    unsigned long timeoutMs = (selfTestStep == 1) ? 5000 : 2000;
    if (now - stepStartTime > timeoutMs && stepPassedTime == 0) waiting = false; // logic fallthrough to fail

    switch (selfTestStep) {
        case 0: // WiFi
            if (WiFi.softAPIP() != IPAddress(0,0,0,0)) passed = true;
            break;
            
        case 1: // Sensor Node (with timeout skip)
            handleNetworking();
            if (WiFi.softAPgetStationNum() > 0) passed = true;
            else if (now - stepStartTime > 5000) passed = true; // Timeout: skip if no connection
            else waiting = true;
            break;
            
        case 2: // Radar
            handleRadar();
            if (radarMsgs > 0) passed = true;
            else waiting = true; // Wait for at least one packet
            break;
            
        case 3: // Audio/Visual
            if (stepPassedTime == 0) {
                playTone(PIN_BUZZER1, 2000, 15);
                ledcWrite(PIN_LED_W, 30);
                if (millis() % 100 > 50) ledcWrite(PIN_LED_W, 0); 
            }
            passed = true;
            break;
    }
    
    // Result handling
    tft.setCursor(220, y);
    if (passed) {
        if (stepPassedTime == 0) stepPassedTime = now;
        
        tft.setTextColor(GREEN, BLACK);
        tft.print("[OK]  ");
        
        // Turn off LED if it was on (Step 4)
        if (selfTestStep == 3 && now - stepPassedTime > 50) ledcWrite(PIN_LED_W, 0);

        // Wait before next step
        if (now - stepPassedTime > 500) {
            selfTestStep++;
            stepStartTime = now;
            stepPassedTime = 0;
        }
    } else if (!waiting || (now - stepStartTime > timeoutMs && stepPassedTime == 0)) {
        tft.setTextColor(RED, BLACK);
        tft.print("[FAIL]");
        selfTestFailed = true;
        
        // Fail also waits a bit to be readable? Or just moves on?
        // Let's wait on fail too
         if (stepPassedTime == 0) stepPassedTime = now;
         
         if (now - stepPassedTime > 500) {
            selfTestStep++;
            stepStartTime = now;
            stepPassedTime = 0;
         }
    } else {
        // Blinking indicator while waiting
        if ((now / 200) % 2) {
             tft.setTextColor(getThemeColor(), BLACK);
             tft.print("[..]  ");
        } else {
             tft.setTextColor(BLACK, BLACK); // blink off
             tft.print("[..]  ");
        }
    }
    
    // Completion
    if (selfTestStep > 3) {
        delay(500);
        currentState = STATE_MONITORING;
        headersDrawn = false;
        radarMsgs = 0; 
        stepPassedTime = 0;
    }
}

void runSelfTest() {
    static bool init = false;
    if (!init) {
        clearScreen();
        drawHeader(STR_DIAGNOSTICS, getThemeColor());
        init = true;
    }
    runSelfTestInternal();
    if (currentState != STATE_SELFTEST) {
        init = false;
    } else {
        lastState = STATE_SELFTEST;
    }
}

// ============== MONITORING GUI ==============
void drawMonitoringLayout() {
    clearScreen();
    drawHeader(STR_HEADER_TITLE, getThemeColor());
    
    // Panels
    // Top Left: Status
    tft.drawRect(5, 35, 150, 80, GRAY);
    tft.fillRect(6, 36, 148, 16, DGRAY);
    tft.setTextColor(WHITE, DGRAY);
    tft.setCursor(10, 40);
    tft.setTextSize(1);
    tft.print(STR_SYSTEM_STATUS);
    
    // Top Right: Radar
    tft.drawRect(165, 35, 150, 80, GRAY);
    tft.fillRect(166, 36, 148, 16, DGRAY);
    tft.setCursor(170, 40);
    tft.print(STR_RADAR_TRACKING);
    
    // Bottom: Accelerometer
    tft.drawRect(5, 125, 310, 80, GRAY);
    tft.fillRect(6, 126, 308, 16, DGRAY);
    tft.setCursor(10, 130);
    tft.print(STR_GRAVITY_SENSOR);
    
    drawStatusBar();
}

void drawMonitoringData() {
    static int frame = 0;
    frame++;
    
    // 1. System Status
    static String lastSysStatus = "";
    String sysStatus = STR_SECURE;
    uint16_t sysColor = BLACK;
    uint16_t sysBg = GREEN;
    
    // Check for fully disabled state
    if (!radarEnabled && !mpuEnabled) {
        sysStatus = "DISABLED";
        sysColor = WHITE;
        sysBg = DGRAY;
    } else if (!sensorConnected) { 
        sysStatus = STR_NO_LINK; 
        sysColor = WHITE; 
        sysBg = RED; 
    }
    
    if (sysStatus != lastSysStatus || frame % 30 == 0) {
        // Draw status box (Outline style)
        // Background
        uint16_t fillColor = (sysBg == GREEN) ? DGRAY : sysBg;
        uint16_t borderColor = (sysBg == GREEN) ? GREEN : ((sysBg == DGRAY) ? GRAY : WHITE);
        
        tft.fillRoundRect(20, 60, 130, 30, 4, fillColor);
        tft.drawRoundRect(20, 60, 130, 30, 4, borderColor);
        
        tft.setTextSize(2);
        // Text color
        if (sysBg == GREEN) tft.setTextColor(GREEN, DGRAY);
        else if (sysBg == DGRAY) tft.setTextColor(GRAY, DGRAY);
        else tft.setTextColor(WHITE, sysBg); // Red background for error
        
        // Center text
        int textW = sysStatus.length() * 12; 
        int x = 20 + (130 - textW) / 2;
        
        tft.setCursor(x, 67);
        tft.print(sysStatus);
        lastSysStatus = sysStatus;
    }
    
    // 2. Radar Data
    // 2. Radar Data
    if (frame % 20 == 0) {
        uint16_t accent = getThemeColor();
        
        if (!radarEnabled) {
             // Disabled State
            tft.fillRoundRect(180, 55, 120, 30, 4, DGRAY);
            tft.drawRoundRect(180, 55, 120, 30, 4, GRAY);
            
            tft.setTextColor(GRAY, DGRAY);
            tft.setTextSize(2);
            tft.setCursor(195, 62);
            tft.print("DISABLED");
            
            tft.fillRect(215, 90, 80, 10, BLACK); // Clear distance
        }
        else if (radarPresence) {
            // Target Detected Box
            tft.fillRoundRect(180, 55, 120, 30, 4, DGRAY);
            tft.drawRoundRect(180, 55, 120, 30, 4, accent);
            
            tft.setTextColor(accent, DGRAY);
            tft.setTextSize(2);
            
            const char* lbl = STR_TARGET;
            int tx = 180 + (120 - (strlen(lbl) * 12)) / 2;
            tft.setCursor(tx, 62);
            tft.print(lbl);
            
            // Distance below
            tft.setTextColor(accent, BLACK);
            tft.setTextSize(1);
            
            // Format distance string
            char distBuf[32];
            snprintf(distBuf, sizeof(distBuf), "DIST: %.1fm", radarDistance);
            int dx = 180 + (120 - (strlen(distBuf) * 6)) / 2;
            
            tft.setCursor(dx, 90);
            tft.print(distBuf);
        } else {
            // Scanning text
            tft.fillRoundRect(180, 55, 120, 30, 4, BLACK);
            tft.drawRoundRect(180, 55, 120, 30, 4, DGRAY);
            
            tft.setTextColor(DGRAY, BLACK);
            tft.setTextSize(2);
            
            const char* lbl = STR_SCANNING;
            int tx = 180 + (120 - (strlen(lbl) * 12)) / 2;
            tft.setCursor(tx, 62);
            tft.print(lbl);
            
            tft.setTextColor(GRAY, BLACK);
            tft.setTextSize(1);
            
            const char* sub = STR_AREA_CLEAR;
            int sx = 180 + (120 - (strlen(sub) * 6)) / 2;
            tft.setCursor(sx, 90);
            tft.print(sub);
        }
    }
    
    // 3. Accelerometer Data
    static bool lastMpuEnabled = true; // Track state changes
    
    if (!mpuEnabled) {
        // Draw DISABLED state ONCE when state changes
        if (lastMpuEnabled != mpuEnabled || frame % 60 == 0) {
            tft.fillRect(6, 143, 308, 61, BLACK); // Clear content area only
            tft.drawRect(5, 125, 310, 80, GRAY);
            tft.fillRect(6, 126, 308, 16, DGRAY);
            
            tft.setTextColor(WHITE, DGRAY);
            tft.setTextSize(1);
            tft.setCursor(10, 130);
            tft.print(STR_GRAVITY_SENSOR);
            
            tft.setTextColor(GRAY, BLACK);
            tft.setTextSize(2);
            tft.setCursor(110, 160);
            tft.print("DISABLED");
            
            lastMpuEnabled = mpuEnabled;
        }
    } else if (frame % 3 == 0) {
             // Normal operation
             uint16_t accent = getThemeColor();
             tft.setTextSize(1);
             tft.setTextColor(GRAY, BLACK);
             
             // Axis labels
             tft.setCursor(20, 155); tft.print("X-AXIS");
             tft.setCursor(120, 155); tft.print("Y-AXIS");
             tft.setCursor(220, 155); tft.print("Z-AXIS");
             
             auto drawBar = [&](int x, float g) {
                  int w = (int)(constrain(g, -2.0, 2.0) * 20) + 40; 
                  tft.drawRect(x, 170, 80, 8, DGRAY);
                  tft.fillRect(x+1, 171, 78, 6, BLACK); 
                  tft.fillRect(x + 40, 171, w - 40, 6, accent);
             };
             
             drawBar(20, mpu_ax);
             drawBar(120, mpu_ay);
             drawBar(220, mpu_az);
             
             tft.setTextColor(WHITE, BLACK);
             tft.setCursor(20, 185); tft.printf("%+.2fg  ", mpu_ax);
             tft.setCursor(120, 185); tft.printf("%+.2fg  ", mpu_ay);
             tft.setCursor(220, 185); tft.printf("%+.2fg  ", mpu_az);
             
             lastMpuEnabled = mpuEnabled;
    }
    
    if (frame % 20 == 0) {
        drawStatusBar();
        drawHeaderIcons();
    }
    
    // Drain TCP buffer between heavy TFT draws to prevent backpressure
    handleNetworking();
}

// ============== ALERT MODE (Tactical) ==============
// ============== ALERT MODE (Tactical) ==============
void drawAlertScreen() {
    static int lastRem = -1;
    unsigned long elapsed = millis() - stateStartTime;
    int remaining = (alertTimeout - elapsed) / 1000;
    if (remaining < 0) remaining = 0;
    
    // Draw static elements once
    if (lastState != STATE_ALERT) {
        tft.fillScreen(BLACK);
        
        // Header
        drawHeader(STR_WARNING, ORANGE);
        
        // Main Warning Box (Rounded & Modern)
        tft.drawRoundRect(20, 50, 280, 150, 8, ORANGE);
        tft.drawRoundRect(21, 51, 278, 148, 8, ORANGE); 
        
        tft.setTextColor(ORANGE);
        tft.setTextSize(2);
        const char* msg = STR_IMPACT_DETECTED;
        int tx = (320 - (strlen(msg)*12))/2;
        tft.setCursor(tx, 70);
        tft.print(msg);

        // Footer Instructions
        tft.setTextColor(GRAY);
        tft.setTextSize(1);
        int fx = (320 - (strlen(STR_CANCEL_MSG)*6))/2;
        tft.setCursor(fx, 220);
        tft.print(STR_CANCEL_MSG);
        
        lastState = STATE_ALERT;
        lastRem = -1; // Force number update
    }
    
    // Countdown Update
    if (remaining != lastRem) {
        // Clear number area inside the box
        tft.fillRect(100, 100, 120, 70, BLACK);
        
        tft.setTextSize(7);
        tft.setTextColor(ORANGE, BLACK);
        
        int nX = (W - (2 * 42)) / 2;
        if (remaining < 10) nX = (W - 42) / 2;
        
        tft.setCursor(nX, 105);
        tft.printf("%d", remaining); 
        
        lastRem = remaining;
    }
    
    // Beep logic (Unchanged)
    if (millis() - lastBeep > 300 - (elapsed / 200)) {
        lastBeep = millis();
        beepFreq = 500 + (elapsed / 30);
        if (beepFreq > 2500) beepFreq = 2500;
        playTone(PIN_BUZZER1, beepFreq, 100);
        playTone(PIN_BUZZER2, beepFreq + 200, 100);
        ledcWrite(PIN_LED_W, alertPhase % 2 ? 200 : 0);
        alertPhase++;
    }
    
    if (elapsed >= alertTimeout) {
        currentState = STATE_DANGER;
        stateStartTime = millis();
        // SMS is sent in Danger screen entry logic now or effectively immediately?
        // Original code called it here. Let's keep it.
        // SMS removed - using Telegram only
    }
}

// ============== DANGER MODE (Tactical) ==============
// ============== DANGER MODE (Tactical) ==============
void drawDangerScreen() {
    static int flash = 0;
    unsigned long elapsed = millis() - stateStartTime;
    
    // Initial Setup on State Entry
    if (lastState != STATE_DANGER) {
        tft.fillScreen(BLACK);
        flash = 0;
        
        // Static elements will be drawn in the flash loop or here?
        // Let's draw static base here to ensure it exists
        drawHeader(STR_CRITICAL, RED);
        
        tft.setTextColor(RED, BLACK);
        tft.setTextSize(3);
        const char* msg = STR_DANGER;
        int tx = (320 - (strlen(msg)*18))/2;
        tft.setCursor(tx, 80);
        tft.print(msg);
        
        tft.setTextColor(WHITE, BLACK);
        tft.setTextSize(2);
        tft.setCursor(55, 140);
        tft.print(STR_SMS_SENT);

        // Send ALL notifications (Telegram, Email, SMS) with severity
        // Done AFTER drawing screen to avoid blank gap
        sendAllNotifications(mpu_impact > 0 ? mpu_impact : 3.0);  // Default 3g if no data
        
        lastState = STATE_DANGER;
    }
    
    if (millis() - lastBeep > 200) {
        lastBeep = millis();
        flash++;
        playTone(PIN_BUZZER1, (flash%2) ? 2000 : 1500, 150);
        playTone(PIN_BUZZER2, (flash%2) ? 2200 : 1700, 150);
        ledcWrite(PIN_LED_W, (flash%2) ? 255 : 0);
        
        // Flashing Border (Thicker - 10px)
        uint16_t color = (flash%2) ? RED : BLACK;
        for (int i = 0; i < 12; i++) {
             tft.drawRect(i, 40 + i, W - (2*i), H - 40 - (2*i), color);
        }
    }
    
    if (elapsed >= DANGER_DURATION) {
        cancelAlert();
    }
}

// (SMS removed - using Telegram only)

// ============== CANCEL ALERT (Tactical Green) ==============
// ============== CANCEL ALERT (Tactical Green) ==============
void cancelAlert() {
    noTone(PIN_BUZZER1);
    noTone(PIN_BUZZER2);
    ledcWrite(PIN_LED_W, 0);
    fallDetected = false;
    
    tft.fillScreen(BLACK);
    
    // Header
    drawHeader(STR_STATUS, GREEN);
    
    // Main Status Box
    tft.drawRoundRect(20, 80, 280, 80, 8, GREEN);
    tft.drawRoundRect(21, 81, 278, 78, 8, GREEN);
    
    tft.setTextColor(GREEN, BLACK);
    tft.setTextSize(2);
    
    const char* line1 = STR_ALARM_CLEARED;
    int x1 = (320 - (strlen(line1) * 12)) / 2;
    tft.setCursor(x1, 100);
    tft.print(line1);
    
    tft.setTextSize(1);
    tft.setTextColor(WHITE, BLACK);
    
    const char* line2 = STR_SYS_RESTORED;
    int x2 = (320 - (strlen(line2) * 6)) / 2;
    tft.setCursor(x2, 130);
    tft.print(line2);
    
    delay(2000);
    
    currentState = STATE_MONITORING;
    lastState = STATE_COOLDOWN; 
}

// ============== FALL DETECTION (FUSION) ==============
void checkForFall() {
    static bool fallSignalActive = false;
    
    if (currentState != STATE_MONITORING) return;
    if (!mpuEnabled && !radarEnabled) return; // Both disabled
    
    // Compute fusion score every cycle
    fusionScore = computeFusionScore();
    
    // Primary trigger: MPU confirmed fall (mpu_fall >= 2)
    if (mpuEnabled && mpu_fall >= 2) {
        if (!fallSignalActive) {
            fallSignalActive = true;
            fallDetected = true;
            fallSeverity = constrain(mpu_impact / 5.0, 0.0, 1.0);
            
            // Fusion-adjusted timeout: higher fusion = shorter timeout
            if (fusionScore > 0.7) {
                // Both sensors agree: high confidence, short timeout
                alertTimeout = 5000;
            } else if (fusionScore > 0.5) {
                alertTimeout = 10000;
            } else {
                // MPU-only detection: standard timeout
                alertTimeout = MAX_ALERT_TIME - (fallSeverity * 25000);
                if (alertTimeout < 5000) alertTimeout = 5000;
            }
            
            currentState = STATE_ALERT;
            stateStartTime = millis();
            alertPhase = 0;
            
            Serial.printf("[FUSION] Fall detected! Score=%.2f Timeout=%dms\n", fusionScore, alertTimeout);
        }
    }
    // Secondary trigger: Radar-only fall detection (when MPU disabled)
    else if (!mpuEnabled && radarEnabled && radarFallScore > 0.6) {
        if (!fallSignalActive) {
            fallSignalActive = true;
            fallDetected = true;
            fallSeverity = radarFallScore;
            alertTimeout = 15000; // Longer timeout for radar-only (more false positives)
            currentState = STATE_ALERT;
            stateStartTime = millis();
            alertPhase = 0;
            
            Serial.printf("[RADAR-ONLY] Fall detected! RadarScore=%.2f\n", radarFallScore);
        }
    }
    
    if (mpu_fall < 2 && radarFallScore < 0.6) {
        fallSignalActive = false;
    }
}

// ============== DATA HANDLING ==============
void parseSensorData(const char* data) {
    char* p;
    if ((p = strstr(data, "\"ax\":")) != NULL) mpu_ax = atof(p + 5);
    if ((p = strstr(data, "\"ay\":")) != NULL) mpu_ay = atof(p + 5);
    if ((p = strstr(data, "\"az\":")) != NULL) mpu_az = atof(p + 5);
    if ((p = strstr(data, "\"gx\":")) != NULL) mpu_gx = atof(p + 5);
    if ((p = strstr(data, "\"gy\":")) != NULL) mpu_gy = atof(p + 5);
    if ((p = strstr(data, "\"gz\":")) != NULL) mpu_gz = atof(p + 5);
    if ((p = strstr(data, "\"fall\":")) != NULL) mpu_fall = atoi(p + 7);
    if ((p = strstr(data, "\"impact\":")) != NULL) mpu_impact = atof(p + 9);
    if ((p = strstr(data, "\"orient\":")) != NULL) mpu_orient = atof(p + 9);
    lastSensorData = millis();
    sensorConnected = true;
}

void handleNetworking() {
    // Check for new TCP client connections
    if (server.hasClient()) {
        if (sensorClient) sensorClient.stop();
        sensorClient = server.available();
        sensorClient.setNoDelay(true);   // Disable Nagle — send IMU data immediately
        sensorClient.setTimeout(50);     // 50ms read timeout
        sensorConnected = true;
        lastSensorData = millis();
        forceIconRedraw = true;
        Serial.println("[TCP] Sensor Node Connected!");
    }

    // Read incoming sensor data
    if (sensorClient && sensorClient.connected()) {
        static char buf[256];
        static int bufIdx = 0;
        int maxReads = 512;
        while (sensorClient.available() && maxReads-- > 0) {
            char c = sensorClient.read();
            if (c == '\n' || bufIdx >= 254) {
                buf[bufIdx] = 0;
                if (bufIdx > 5) parseSensorData(buf);
                bufIdx = 0;
            } else if (c >= 32) buf[bufIdx++] = c;
        }
    } else if (sensorConnected && !sensorClient.connected() && millis() - lastSensorData > 3000) {
        // TCP dropped AND no ESP-NOW data recently — truly disconnected
        sensorConnected = false;
        forceIconRedraw = true;
        Serial.println("[TCP] Sensor Node Disconnected");
    }

    // Fallback timeout — if no data for 5s, mark as disconnected
    if (millis() - lastSensorData > 5000 && sensorConnected) {
        sensorConnected = false;
        forceIconRedraw = true;
    }

    // Non-blocking WiFi STA reconnect
    handleWifiReconnect();
}

void handleRadar() {
    static uint8_t rdBuf[64];
    static int rdIdx = 0;

    while (radarSerial.available()) {
        uint8_t b = radarSerial.read();

        if (rdIdx < 4) {
            if (b == RD03D_FRAME_HEADER[rdIdx]) {
                rdBuf[rdIdx++] = b;
            } else if (b == RD03D_FRAME_HEADER[0]) {
                rdBuf[0] = b;
                rdIdx = 1;
            } else {
                rdIdx = 0;
            }
            continue;
        }

        rdBuf[rdIdx++] = b;

        if (rdIdx > 60) {
            rdIdx = 0;
            continue;
        }

        if (rdIdx >= 30 && rdBuf[rdIdx-2] == 0x55 && rdBuf[rdIdx-1] == 0xCC) {
            static uint8_t lastRdBuf[30];
            static int frozenCount = 0;

            if (memcmp(rdBuf, lastRdBuf, 30) == 0) {
                frozenCount++;
            } else {
                frozenCount = 0;
                memcpy(lastRdBuf, rdBuf, 30);
            }

            radarTargetCount = 0;
            float bestDist = 0;
            int16_t bestSpeed = 0;
            bool anyTarget = false;

            if (frozenCount < 10) {
                for (uint8_t i = 0; i < 3; i++) {
                    uint8_t offset = 4 + (i * 8);

                    int16_t x = rd03d_decode_value(rdBuf[offset + 0], rdBuf[offset + 1]);
                    int16_t y = rd03d_decode_value(rdBuf[offset + 2], rdBuf[offset + 3]);
                    int16_t speed = rd03d_decode_value(rdBuf[offset + 4], rdBuf[offset + 5]);
                    uint16_t resolution = (rdBuf[offset + 7] << 8) | rdBuf[offset + 6];

                    bool hasPosition = (x != 0 || y != 0);
                    bool hasValidSpeed = rd03d_is_speed_valid(speed);
                    bool targetPresent = hasPosition && hasValidSpeed;

                    if (radarSerialDebug) {
                        Serial.printf("T%d: x=%dmm y=%dmm spd=%dcm/s res=%dmm %s\n",
                            i+1, x, y, speed, resolution, targetPresent ? "VALID" : "---");
                    }

                    if (targetPresent) {
                        radarTargetCount++;
                        anyTarget = true;
                        float dist = sqrt(pow((float)x, 2) + pow((float)y, 2));
                        if (dist > bestDist) {
                            bestDist = dist;
                            bestSpeed = speed;
                        }
                    }
                }
            }

            radarPresence = anyTarget;
            radarDistance = bestDist / 1000.0f;
            radarSpeed = bestSpeed;
            radarMsgs++;

            addRadarSample(radarDistance, radarPresence);

            rdIdx = 0;
        }
    }
}

// ============== STATUS ICONS (BITMAP ICONS) ==============

// 12x12 Bitmaps (Data is top-to-bottom, MSB left)

// WiFi Icons (0=None, 1=Dot, 2=Low, 3=Med, 4=High)
static const unsigned char wifi_0[] PROGMEM = {
    0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x03,0x00, 0x03,0x00, 0x00,0x00
}; // Just dot
static const unsigned char wifi_1[] PROGMEM = {
   0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x07,0x80, 0x04,0x00, 0x03,0x00, 0x03,0x00, 0x00,0x00
}; // Low
static const unsigned char wifi_2[] PROGMEM = {
   0x00,0x00, 0x00,0x00, 0x00,0x00, 0x1F,0xE0, 0x10,0x20, 0x00,0x00, 0x07,0x80, 0x04,0x00, 0x03,0x00, 0x03,0x00, 0x00,0x00
}; // Med
static const unsigned char wifi_3[] PROGMEM = {
   0x3F,0xF8, 0x20,0x08, 0x00,0x00, 0x1F,0xE0, 0x10,0x20, 0x00,0x00, 0x07,0x80, 0x04,0x00, 0x03,0x00, 0x03,0x00, 0x00,0x00
}; // High

// SIM Icons (Signal Bars)
static const unsigned char sim_icon[] PROGMEM = {
   0x00,0x00, 0x00,0x00, 0x00,0x10, 0x00,0x10, 0x00,0x10, 0x00,0xD0, 0x00,0xD0, 0x00,0xD0, 0x07,0xD0, 0x07,0xD0, 0x07,0xD0, 0x00,0x00
}; // 4 Bars solid
static const unsigned char sim_fail[] PROGMEM = {
   0xC0,0x70, 0x60,0xE0, 0x31,0xC0, 0x1B,0x80, 0x0F,0x00, 0x1B,0x80, 0x31,0xC0, 0x60,0xE0, 0xC0,0x70, 0x00,0x00, 0x00,0x00, 0x00,0x00
}; // X icon

// Sensor Icon (Isometric Cube)
static const unsigned char sensor_icon[] PROGMEM = {
   0x03,0xC0, 0x07,0xE0, 0x0F,0xF0, 0x19,0x98, 0x31,0x8C, 0x61,0x86, 0x63,0xC6, 0x66,0x66, 0x6C,0x36, 0x38,0x1C, 0x10,0x08, 0x00,0x00
}; // Cube
static const unsigned char sensor_fail[] PROGMEM = {
   0x0C,0x30, 0x1E,0x78, 0x33,0xCC, 0x61,0x86, 0x61,0x86, 0x33,0xCC, 0x1C,0x38, 0x0C,0x30, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00
}; // Disconnected ? icon (Keep same)

// Radar Icon (Sector Fan) 
static const unsigned char radar_icon[] PROGMEM = {
   0x00,0x00, 0x01,0x80, 0x03,0xC0, 0x06,0x60, 0x0C,0x30, 0x19,0x98, 0x33,0xCC, 0x67,0xE6, 0x08,0x10, 0x10,0x08, 0x20,0x04, 0x00,0x00
}; // Fan Sector

// Track States for Redraw Optimization
int lastWifiState = -1;
bool lastSimState = false;
bool lastSensorState = false;
bool lastRadarState = false;
// forceIconRedraw declared earlier (line ~209)

// Icon Drawing Wrappers
void drawWiFiIcon(int x, int y, uint16_t color) {
    int clients = WiFi.softAPgetStationNum();
    bool activeAP = (WiFi.softAPIP() != IPAddress(0,0,0,0));
    
    const unsigned char* bmp;
    // Map status to bitmap
    if (clients > 1) bmp = wifi_3;       // >1 client = High
    else if (clients == 1) bmp = wifi_2; // 1 client = Med
    else if (activeAP) bmp = wifi_1;     // AP on, no clients = Low
    else bmp = wifi_0;                   // Off = Dot
    
    // Default color logic override if needed for specific states? 
    // Stick to passed 'color' which is calculated in drawHeaderIcons
    tft.drawBitmap(x+2, y+8, bmp, 12, 12, color);
}

// (SIM icon removed)

void drawSensorIcon(int x, int y, uint16_t color) {
    if (sensorConnected) {
        tft.drawBitmap(x+2, y+8, sensor_icon, 12, 12, color);
    } else {
        tft.drawBitmap(x+2, y+8, sensor_fail, 12, 12, RED); // Use X/Fail icon
    }
}

void drawRadarIcon(int x, int y, uint16_t color) {
    // Only draw sweep dot if active
    tft.drawBitmap(x+4, y+8, radar_icon, 12, 12, color);
    if (radarMsgs > 0 && radarEnabled) {
         tft.fillCircle(x+10, y+13, 2, color); // Active dot at bottom center
    }
}

// Draw all status icons in header area (text-based, clean)
void drawHeaderIcons() {
    int currWifi = WiFi.softAPgetStationNum();
    bool currSens = sensorConnected;
    bool currRad = (radarMsgs > 0);
    
    if (!forceIconRedraw && 
        currWifi == lastWifiState && 
        currSens == lastSensorState && 
        currRad == lastRadarState) {
        return;
    }

    int startX = W - 82; 
    tft.fillRect(startX, 1, 82, 26, DGRAY);
    
    tft.setTextSize(1);
    int x = startX + 4;
    int y = 10;
    
    // WiFi indicator
    uint16_t wifiC = (currWifi > 0) ? GREEN : 
                     (WiFi.softAPIP() != IPAddress(0,0,0,0)) ? YELLOW : RED;
    tft.setTextColor(wifiC, DGRAY);
    tft.setCursor(x, y);
    tft.printf("W:%d", currWifi);
    
    // Radar indicator
    x += 28;
    tft.setTextColor(currRad ? GREEN : RED, DGRAY);
    tft.setCursor(x, y);
    tft.print(currRad ? "R:OK" : "R:--");
    
    // Sensor indicator  
    x += 28;
    tft.setTextColor(currSens ? GREEN : RED, DGRAY);
    tft.setCursor(x, y);
    tft.print(currSens ? "S" : "X");
    
    lastWifiState = currWifi;
    lastSensorState = currSens;
    lastRadarState = currRad;
    forceIconRedraw = false;
}


// (SIM800C code removed — using Telegram only)


// (Removed duplicate handleButton)

// ============== NOTIFICATIONS ==============
// URL Encode helper for HTTP requests
String urlEncode(const String& str) {
    String encoded = "";
    char c;
    for (int i = 0; i < str.length(); i++) {
        c = str.charAt(i);
        if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
            encoded += c;
        } else if (c == ' ') {
            encoded += '+';
        } else {
            char buf[4];
            snprintf(buf, sizeof(buf), "%%%02X", (unsigned char)c);
            encoded += buf;
        }
    }
    return encoded;
}

// Send Telegram Message
bool sendTelegram(const char* message) {
    if (WiFi.status() != WL_CONNECTED) return false;
    if (strlen(TELEGRAM_TOKEN) == 0 || strlen(TELEGRAM_CHAT_ID) == 0) return false;
    
    HTTPClient http;
    String url = "https://api.telegram.org/bot" + String(TELEGRAM_TOKEN) 
               + "/sendMessage?chat_id=" + String(TELEGRAM_CHAT_ID) 
               + "&text=" + urlEncode(message);
    
    http.begin(url);
    http.setTimeout(3000);  // 3s max instead of default 5s
    int httpCode = http.GET();
    http.end();
    
    Serial.printf("Telegram: %s (HTTP %d)\n", (httpCode == 200) ? "Sent!" : "Failed", httpCode);
    return (httpCode == 200);
}

// Send Email (Simple SMTP - Note: For production, use ESP_Mail_Client library)
// This is a simplified version that works with some SMTP servers
bool sendEmail(const char* subject, const char* body) {
    if (WiFi.status() != WL_CONNECTED) return false;
    if (strlen(EMAIL_USER) == 0 || strlen(EMAIL_TO) == 0) return false;
    
    // For reliable email, ESP_Mail_Client library is recommended
    // This placeholder signals that email config exists but needs library
    Serial.println("Email: Would send to " + String(EMAIL_TO));
    Serial.println("Subject: " + String(subject));
    // Note: Full SMTP implementation requires SSL client and more code
    // Consider using Telegram which works out of the box
    return false; // Placeholder - add ESP_Mail_Client for real email
}

// Get formatted time string (with fallback)
String getTimeString() {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 100)) {  // Quick check, 100ms timeout
        char buf[32];
        strftime(buf, sizeof(buf), "%H:%M - %d/%m/%Y", &timeinfo);
        return String(buf);
    }
    // Fallback: show uptime
    unsigned long uptime = millis() / 1000;
    char buf[32];
    snprintf(buf, sizeof(buf), "Uptime: %lu sec", uptime);
    return String(buf);
}

// Build rich notification message
// Build rich notification message
String buildNotificationMessage(float severity) {
    String msg = "";
    String timeStr = getTimeString();
    unsigned long uptimeMin = millis() / 60000;
    
    if (languageTR) {
        msg += "🚨 <b>DÜŞME ALARMI!</b> 🚨\n\n";
        msg += "📅 <b>Zaman:</b> " + timeStr + "\n";
        msg += "📍 <b>Cihaz:</b> " + String(AP_SSID) + "\n";
        msg += "⏱️ <b>Çalışma Süresi:</b> " + String(uptimeMin) + " dk\n";
        
        String sevStr = "Düşük 🟡";
        if (severity >= 4.0) sevStr = "KRİTİK 🔴";
        else if (severity >= 2.5) sevStr = "Yüksek 🟠";
        
        msg += "📊 <b>Şiddet:</b> " + sevStr + " (" + String(severity, 1) + "g)\n\n";
        msg += "⚠️ <i>Bu bir acil durum uyarısıdır.\nLütfen kişiyi kontrol edin!</i>";
    } else {
        msg += "🚨 <b>FALL DETECTED!</b> 🚨\n\n";
        msg += "📅 <b>Time:</b> " + timeStr + "\n";
        msg += "📍 <b>Device:</b> " + String(AP_SSID) + "\n";
        msg += "⏱️ <b>Uptime:</b> " + String(uptimeMin) + " min\n";
        
        String sevStr = "Low 🟡";
        if (severity >= 4.0) sevStr = "CRITICAL 🔴";
        else if (severity >= 2.5) sevStr = "High 🟠";
        
        msg += "📊 <b>Severity:</b> " + sevStr + " (" + String(severity, 1) + "g)\n\n";
        msg += "⚠️ <i>This is an emergency alert.\nPlease check immediately!</i>";
    }
    
    return msg;
}

// Send All Notifications (with rich message)
void sendAllNotifications(float impactG) {
    Serial.println("Sending notifications...");
    
    // Build rich message with time and severity
    String richMessage = buildNotificationMessage(impactG);
    
    // Telegram (Supports HTML)
    // We need to enable HTML parse mode in the URL
    if (WiFi.status() == WL_CONNECTED && strlen(TELEGRAM_TOKEN) > 0) {
        HTTPClient http;
        String url = "https://api.telegram.org/bot" + String(TELEGRAM_TOKEN) 
                   + "/sendMessage?chat_id=" + String(TELEGRAM_CHAT_ID) 
                   + "&parse_mode=HTML&text=" + urlEncode(richMessage);
        http.begin(url);
        http.setTimeout(3000);  // 3s max
        http.GET();
        http.end();
        Serial.println("Telegram sent");
    }
    
    // Email
    String subject = languageTR ? "🚨 Düşme Uyarısı!" : "🚨 Fall Alert!";
    sendEmail(subject.c_str(), richMessage.c_str());
}



// ============== SETUP ==============
void setup() {
    Serial.begin(115200);
    EEPROM.begin(EEPROM_SIZE);
    loadSettings(); // Load persistence
    
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    ledcAttach(PIN_LED_W, 5000, 8);
    pinMode(PIN_BUZZER1, OUTPUT);
    pinMode(PIN_BUZZER2, OUTPUT);
    // Boot beep to verify buzzers
    tone(PIN_BUZZER1, 2000, 100);
    delay(120);
    tone(PIN_BUZZER2, 2500, 100);
    delay(120);
    noTone(PIN_BUZZER1);
    noTone(PIN_BUZZER2);
    
    SPI.begin(TFT_SCK, -1, TFT_MOSI, TFT_CS);
    tft.begin();
    tft.setRotation(1);
    
    // WiFi: Dual Mode (AP for watch + STA for internet)
    WiFi.mode(WIFI_AP_STA);
    
    // Register WiFi event handlers BEFORE starting AP/STA
    WiFi.onEvent(onAPClientConnect, WiFiEvent_t::ARDUINO_EVENT_WIFI_AP_STACONNECTED);
    WiFi.onEvent(onAPClientDisconnect, WiFiEvent_t::ARDUINO_EVENT_WIFI_AP_STADISCONNECTED);
    WiFi.onEvent(onSTAGotIP, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
    WiFi.onEvent(onSTADisconnect, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
    
    // Disable WiFi power saving
    esp_wifi_set_ps(WIFI_PS_NONE);
    
    // Start AP (channel may shift when STA connects to router)
    WiFi.softAP(AP_SSID, AP_PASS, 1);
    
    // Max TX power
    esp_wifi_set_max_tx_power(78);
    
    // Re-add STA for home WiFi (Telegram, NTP)
    if (strlen(HOME_SSID) > 0 && strcmp(HOME_SSID, "YOUR_WIFI_NAME") != 0) {
        WiFi.setAutoReconnect(true);
        WiFi.begin(HOME_SSID, HOME_PASS);
        Serial.println("WiFi STA connecting in background...");
        configTime(3 * 3600, 0, "pool.ntp.org", "time.google.com");
    }
    
    server.begin();
    
    // Init ESP-NOW receiver
    if (esp_now_init() == ESP_OK) {
        esp_now_register_recv_cb(onESPNowRecv);
        Serial.println("[ESP-NOW] Receiver ready");
    } else {
        Serial.println("[ESP-NOW] Init FAILED");
    }
    
    // Print actual channel (may differ from 1 if STA connected to router)
    uint8_t primaryCh;
    wifi_second_chan_t secondCh;
    esp_wifi_get_channel(&primaryCh, &secondCh);
    Serial.printf("[AP] SSID: %s  IP: %s  CH: %d\n", AP_SSID, WiFi.softAPIP().toString().c_str(), primaryCh);
    
    // RD-03D radar @ 256000 baud (binary protocol)
    radarSerial.begin(256000, SERIAL_8N1, PIN_RADAR_RX, PIN_RADAR_TX);
    
    currentState = STATE_SELFTEST;
}

// ============== BUTTON INPUT ==============
void handleButton() {
    static int lastState = HIGH;
    static int stableState = HIGH;
    static unsigned long lastDebounceTime = 0;
    static unsigned long pressStart = 0;
    static bool ignoreRelease = false;
    
    int reading = digitalRead(PIN_BUTTON);
    
    // 1. Debounce
    if (reading != lastState) {
        lastDebounceTime = millis();
    }
    lastState = reading;
    
    if ((millis() - lastDebounceTime) > 50) {
        // State is stable
        if (reading != stableState) {
            stableState = reading;
            
            // 2. Edge Detection
            if (stableState == LOW) {
                // Pressed
                pressStart = millis();
                ignoreRelease = false;
                
                // Instant Actions (Cancel Alert/Danger)
                if (currentState == STATE_ALERT || currentState == STATE_DANGER) {
                    cancelAlert();
                    ignoreRelease = true;
                }
            } else {
                // Released
                if (pressStart > 0 && !ignoreRelease) {
                    unsigned long duration = millis() - pressStart;
                    if (duration < 800) {
                        // Short Press
                         if (currentState == STATE_SETTINGS) {
                            menuIndex = (menuIndex + 1) % menuItems;
                            playTone(PIN_BUZZER1, 1500, 20); 
                        }
                    }
                }
                pressStart = 0;
            }
        }
    }
    
    // 3. Hold Logic (while pressed)
    if (stableState == LOW && pressStart > 0 && !ignoreRelease) {
        if (millis() - pressStart > 800) {
            // Long Hold Action
            if (currentState == STATE_MONITORING) {
                currentState = STATE_SETTINGS;
                menuIndex = 0;
                playTone(PIN_BUZZER1, 1000, 200);
            } else if (currentState == STATE_SETTINGS) {
                // Select/Toggle
                if (menuIndex == 0) {
                     buzzersMuted = !buzzersMuted;
                     saveSettings();
                } else if (menuIndex == 1) {
                     radarEnabled = !radarEnabled; // New Radar Toggle
                     saveSettings();
                } else if (menuIndex == 2) {
                     mpuEnabled = !mpuEnabled; // Toggle MPU
                     saveSettings(); // Save Immediately
                } else if (menuIndex == 3) {
                     radarSerialDebug = !radarSerialDebug;
                     saveSettings();
                } else if (menuIndex == 4) {
                     themeIndex = (themeIndex + 1) % 6; // Cycle 6 Themes
                     bool force = true;
                     lastState = STATE_MONITORING; 
                     saveSettings();
                } else if (menuIndex == 5) {
                     languageTR = !languageTR;
                     lastState = STATE_MONITORING; 
                     saveSettings();
                } else {
                     currentState = STATE_MONITORING;
                     saveSettings();
                     forceIconRedraw = true;
                }
                playTone(PIN_BUZZER1, 2000, 100);
            }
            ignoreRelease = true; // Handled, don't trigger release
        }
    }
}

// ============== SETTINGS MENU ==============
void drawSettingsMenu() {
    static int lastMenuIndex = -1;
    static bool lastMuted = !buzzersMuted; 
    static bool lastLang = !languageTR;    
    static bool lastRadar = !radarEnabled;
    static bool lastMpu = !mpuEnabled;
    static bool lastRadarDbg = !radarSerialDebug;
    static int lastTheme = -1;
    
    // Only clear screen if we just entered state
    if (lastState != STATE_SETTINGS) {
        tft.fillScreen(BLACK);
        // Header using consistent style
        drawHeader(STR_SETTINGS, getThemeColor());
        
        // Footer - Firmware Info
        tft.setTextSize(1);
        tft.setTextColor(DGRAY);
        char fwInfo[64];
        snprintf(fwInfo, sizeof(fwInfo), "FALL DETECT %s | %s", VERSION, BUILD_DATE);
        int fwX = (320 - strlen(fwInfo) * 6) / 2;
        tft.setCursor(fwX, 225);
        tft.print(fwInfo);
        
        // Instructions
        tft.setTextColor(GRAY);
        tft.setCursor(45, 210);
        tft.print(STR_NAV_HELP);
        
        lastState = STATE_SETTINGS;
        lastMenuIndex = -1; 
    }
    
    // Redraw ONLY if something changed
    if (menuIndex != lastMenuIndex || buzzersMuted != lastMuted || languageTR != lastLang || 
        radarEnabled != lastRadar || mpuEnabled != lastMpu || radarSerialDebug != lastRadarDbg || themeIndex != lastTheme) {
        
        // 7 items: use smaller boxes
        int boxH = 22;
        int startY = 35;
        int gap = 3;
        uint16_t accent = getThemeColor();
        
        // Helper to draw menu item
        auto drawItem = [&](int idx, const char* label, String val, uint16_t valColor) {
            int y = startY + (idx * (boxH + gap));
            bool selected = (menuIndex == idx);
            uint16_t border = selected ? accent : GRAY;
            
            // Draw - FILL FIRST then BORDER to avoid overwrite
            if (selected) tft.fillRoundRect(21, y+1, 278, boxH-2, 3, DGRAY);
            else tft.fillRoundRect(21, y+1, 278, boxH-2, 3, BLACK);
            tft.drawRoundRect(20, y, 280, boxH, 4, border);
            
            tft.setTextColor(WHITE, selected ? DGRAY : BLACK);
            tft.setCursor(35, y + 6);
            tft.print(label);
            
            if (val.length() > 0) {
                // Right aligned value with custom color
                tft.setTextColor(valColor, selected ? DGRAY : BLACK);
                int valW = val.length() * 12;
                tft.setCursor(280 - valW, y + 6);
                tft.print(val);
            }
        };
        
        // 1. Mute
        String muteVal = buzzersMuted ? "[ ON ]" : "[ OFF ]";
        drawItem(0, STR_MUTE_ALARMS, muteVal, buzzersMuted ? GREEN : RED);
        
        // 2. Radar
        String radarVal = radarEnabled ? "[ ON ]" : "[ OFF ]";
        drawItem(1, "RADAR", radarVal, radarEnabled ? GREEN : RED);
        
        // 3. MPU
        String mpuVal = mpuEnabled ? "[ ON ]" : "[ OFF ]";
        drawItem(2, "MPU SENSOR", mpuVal, mpuEnabled ? GREEN : RED);
        
        // 4. Radar Debug (serial output)
        String dbgVal = radarSerialDebug ? "[ ON ]" : "[ OFF ]";
        drawItem(3, "RADAR DBG", dbgVal, radarSerialDebug ? GREEN : RED);
        
        // 5. Theme
        String themeName = "[ CYAN ]";
        if (themeIndex == 0) themeName = languageTR ? "[ TURKUAZ ]" : "[ CYAN ]";
        if (themeIndex == 1) themeName = languageTR ? "[ YESIL ]" : "[ GREEN ]";
        if (themeIndex == 2) themeName = languageTR ? "[ MOR ]" : "[ MAGENTA ]";
        if (themeIndex == 3) themeName = languageTR ? "[ SARI ]" : "[ YELLOW ]";
        if (themeIndex == 4) themeName = languageTR ? "[ TURUNCU ]" : "[ ORANGE ]";
        if (themeIndex == 5) themeName = languageTR ? "[ MAVI ]" : "[ BLUE ]";
        drawItem(4, "THEME", themeName, accent);
        
        // 6. Language
        drawItem(5, STR_LANG_OPT, languageTR ? "[ TR ]" : "[ EN ]", accent);
        
        // 7. Exit
        drawItem(6, STR_EXIT, "", WHITE);
        
        // Update trackers
        bool needFullRedraw = (languageTR != lastLang) || (themeIndex != lastTheme);
        
        lastMenuIndex = menuIndex;
        lastMuted = buzzersMuted;
        lastLang = languageTR;
        lastRadar = radarEnabled;
        lastTheme = themeIndex;
        lastMpu = mpuEnabled;
        lastRadarDbg = radarSerialDebug;
        
        if (needFullRedraw) { 
           // Force full re-initialization next frame to update Header color/text
           lastState = STATE_MONITORING; 
        }
    }
}

// ============== LOOP ==============
void loop() {
    handleButton();
    updateLedStatus(); // New LED routine

    switch (currentState) {
        case STATE_BOOT:
            // Handled in setup
            break;
        case STATE_SELFTEST:
            runSelfTest();
            break;
        case STATE_MONITORING:
            // State Entry - Clear Screen & Draw Layout
            if (lastState != STATE_MONITORING) {
                drawMonitoringLayout();
                lastState = STATE_MONITORING;
                forceIconRedraw = true;
            }
            
            // Networking
            handleNetworking();
            handleRadar();
            
            // Logic
            checkForFall();
            
            // GUI
            drawMonitoringData();
            
            // Telemetry Output for HTML Dashboard (10Hz)
            {
                static unsigned long lastTelemetry = 0;
                if (millis() - lastTelemetry > 100) {
                    lastTelemetry = millis();
                    Serial.printf("TELEMETRY: ax=%.2f ay=%.2f az=%.2f gx=%.2f gy=%.2f gz=%.2f mlFall=%d mlConf=%d fusion=%.2f radar=%.2f\n",
                                  mpu_ax, mpu_ay, mpu_az, mpu_gx, mpu_gy, mpu_gz, 
                                  mpu_mlFall, mpu_mlConf, fusionScore, radarFallScore);
                }
            }
            break;
            
        case STATE_SETTINGS:
            drawSettingsMenu();
            break;
            
        case STATE_ALERT:
            drawAlertScreen();
            break;
        case STATE_DANGER:
            drawDangerScreen();
            break;
        case STATE_COOLDOWN:
            // Just wait
            break;
    }
}
