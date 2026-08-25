/**
 * RANGER — Bathroom Zone Fall Detector (ESP32-S3 SuperMini + HLK-LD1125H)
 * 
 * Hardware:
 *   - Microcontroller: ESP32-S3 SuperMini
 *   - Radar: HLK-LD1125H 24GHz mmWave Radar (UART @ 115200, RX=GPIO6, TX=GPIO5)
 *   - Actuators: Piezo Buzzer (GPIO 10), WS2812B NeoPixel (GPIO 48)
 *   - Radio: ESP-NOW broadcast to CrowPanel dashboard
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#define VERSION "3.4-BATH-NODE"

// Hardware Pin Configuration
#define PIN_RADAR_RX  6    // ESP RX <- Radar TX
#define PIN_RADAR_TX  5    // ESP TX -> Radar RX
#define PIN_BUZZER    10
#define PIN_NEOPIXEL  48   // Onboard RGB LED

void setNeoPixel(uint8_t r, uint8_t g, uint8_t b) {
    neopixelWrite(PIN_NEOPIXEL, r, g, b);
}

enum RadarState { RADAR_EMPTY, RADAR_MOVING, RADAR_OCCUPIED };
RadarState radarState = RADAR_EMPTY;
float radarDistance = 0;
unsigned long lastRadarMsg = 0;
bool radarOnline = false;

// Fall Detection via Radar Trajectory & Post-Event Stillness
#define FALL_DIST_DELTA     0.4f   // Minimum sudden displacement threshold (meters)
#define FALL_CONFIRM_MS     3000   // Required static occupation window (ms)
#define FALL_COOLDOWN_MS    30000  // Re-trigger suppression window

enum FallState { FALL_IDLE, FALL_DETECTED, FALL_POSSIBLE, FALL_CONFIRMED };
FallState fallState = FALL_IDLE;
unsigned long lastFallTrigger = 0;
unsigned long possibleFallStart = 0;
float preFallDist = 0;

float distHistory[10];
int historyIdx = 0;

void addDistSample(float d) {
    distHistory[historyIdx] = d;
    historyIdx = (historyIdx + 1) % 10;
}

void checkForFall() {
    if (radarState == RADAR_EMPTY) return;
    
    unsigned long now = millis();
    if (now - lastFallTrigger < FALL_COOLDOWN_MS) return;

    static float lastD = 0;
    float delta = abs(radarDistance - lastD);
    
    if (delta > FALL_DIST_DELTA && radarState == RADAR_MOVING && fallState == FALL_IDLE) {
        fallState = FALL_POSSIBLE;
        possibleFallStart = now;
        preFallDist = lastD;
    }
    
    if (fallState == FALL_POSSIBLE) {
        if (radarState == RADAR_OCCUPIED) {
            if (now - possibleFallStart > FALL_CONFIRM_MS) {
                fallState = FALL_CONFIRMED;
                lastFallTrigger = now;
            }
        } else if (radarState == RADAR_MOVING && (now - possibleFallStart > 1000)) {
            fallState = FALL_IDLE;
        }
    }
    
    lastD = radarDistance;
}

// ESP-NOW Telemetry Interface
typedef struct {
    uint8_t  magic;       // 0xBB
    uint8_t  presence;    // 0=Empty, 1=Moving, 2=Occupied
    uint16_t distance_cm; // Distance in cm
    uint8_t  fallState;   // 0=Idle, 1=Detected, 2=Possible, 3=Confirmed
    uint8_t  checksum;
} BathPacket;

bool espNowReady = false;

void initEspNow() {
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) return;

    esp_now_peer_info_t peerInfo = {};
    memset(peerInfo.peer_addr, 0xFF, 6); // Broadcast
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
        espNowReady = true;
    }
}

void sendBathPacket() {
    if (!espNowReady) return;

    BathPacket pkt;
    pkt.magic = 0xBB;
    pkt.presence = (uint8_t)radarState;
    pkt.distance_cm = (uint16_t)(radarDistance * 100);
    pkt.fallState = (uint8_t)fallState;

    uint8_t ck = 0;
    uint8_t* p = (uint8_t*)&pkt;
    for (size_t i = 0; i < sizeof(pkt) - 1; i++) ck ^= p[i];
    pkt.checksum = ck;

    uint8_t broadcast[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_send(broadcast, (uint8_t*)&pkt, sizeof(pkt));
}

// Radar UART Line Parser
char radarBuf[128];
int radarBufIdx = 0;

void parseRadarLine(const char* line) {
    String s = String(line);
    s.trim();

    if (s.startsWith("mov")) {
        radarState = RADAR_MOVING;
        int idx = s.indexOf("dis=");
        if (idx > 0) radarDistance = s.substring(idx + 4).toFloat();
        addDistSample(radarDistance);
        lastRadarMsg = millis();
        radarOnline = true;
    } else if (s.startsWith("occ")) {
        radarState = RADAR_OCCUPIED;
        int idx = s.indexOf("dis=");
        if (idx > 0) radarDistance = s.substring(idx + 4).toFloat();
        addDistSample(radarDistance);
        lastRadarMsg = millis();
        radarOnline = true;
    } else if (s.indexOf("empty") >= 0) {
        radarState = RADAR_EMPTY;
        radarDistance = 0;
        lastRadarMsg = millis();
        radarOnline = true;
    }
}

void processRadarUART() {
    while (Serial1.available()) {
        char c = Serial1.read();
        if (c == '\n' || c == '\r') {
            if (radarBufIdx > 2) {
                radarBuf[radarBufIdx] = '\0';
                parseRadarLine(radarBuf);
            }
            radarBufIdx = 0;
        } else {
            if (radarBufIdx < 126) radarBuf[radarBufIdx++] = c;
            else radarBufIdx = 0;
        }
    }
}

void buzzerTick() {
    if (fallState == FALL_CONFIRMED) {
        if ((millis() / 150) % 2 == 0) tone(PIN_BUZZER, 2500);
        else noTone(PIN_BUZZER);
    } else if (fallState == FALL_POSSIBLE) {
        if ((millis() / 500) % 2 == 0) tone(PIN_BUZZER, 1500);
        else noTone(PIN_BUZZER);
    } else {
        noTone(PIN_BUZZER);
    }
}

void updateLED() {
    if (fallState == FALL_CONFIRMED) {
        setNeoPixel(255, 0, 0); // Emergency Red
    } else if (fallState == FALL_POSSIBLE) {
        setNeoPixel(255, 165, 0); // Warning Orange
    } else if (radarState != RADAR_EMPTY) {
        setNeoPixel(0, 255, 0); // Active Green
    } else {
        setNeoPixel(0, 0, 50); // Standby Blue
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(PIN_BUZZER, OUTPUT);

    // Radar UART1
    Serial1.begin(115200, SERIAL_8N1, PIN_RADAR_RX, PIN_RADAR_TX);

    initEspNow();
}

void loop() {
    processRadarUART();
    checkForFall();
    buzzerTick();
    updateLED();

    static unsigned long lastPkt = 0;
    if (millis() - lastPkt > 500) {
        sendBathPacket();
        lastPkt = millis();
    }
}
