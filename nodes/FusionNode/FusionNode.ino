/**
 * RANGER — Central Fusion Node (Raspberry Pi Pico RP2040)
 * 
 * Hardware Interfaces:
 *   - UART0 (GP0/GP1 @ 115200): Ingests telemetry CSV from Base Node.
 *   - UART1 (GP4/GP5 @ 256000): Ingests 30-byte frames from RD-03D mmWave Radar.
 *   - I2C1 (GP2/GP3 @ 0x42 Slave): Exposes 24-byte telemetry struct to CrowPanel master.
 *   - PIO UART (GP8/GP9 @ 9600): Drives DFPlayer Mini audio annunciator.
 *   - GPIO16: Optoisolated relay control for ESP32-CAM power gating.
 */

#include <Arduino.h>
#include <Wire.h>
#include <DFRobotDFPlayerMini.h>

#define VERSION "4.0-FUSION-ML"

// Pin Assignments
#define PIN_BASE_TX     0
#define PIN_BASE_RX     1
#define PIN_OLED_SDA    2
#define PIN_OLED_SCL    3
#define PIN_RADAR_TX    4
#define PIN_RADAR_RX    5
#define PIN_DFP_TX      8
#define PIN_DFP_RX      9
#define PIN_LED_R       13
#define PIN_LED_G       14
#define PIN_LED_B       15
#define PIN_BUZZER      22
#define PIN_RELAY       16

#define RELAY_ON_TIME_MS 60000   // 60-second power cycle for ESP-CAM

// I2C Slave Telemetry Interface (Address 0x42)
#define FUSION_I2C_ADDR  0x42
#define TELEM_SIZE       24

#pragma pack(push, 1)
struct TelemetryPacket {
    uint8_t  magic;         // 0xAA preamble
    int16_t  ax, ay, az;    // Raw acceleration (÷4096 -> g)
    int16_t  gx, gy, gz;    // Raw angular velocity (÷65.5 -> dps)
    uint8_t  battery;       // 0–100%
    uint8_t  sos;           // 0: inactive, 1: triggered
    uint8_t  fallState;     // 0: normal, 1: checking stillness, 2: confirmed fall
    uint8_t  wearLink;      // 0: offline, 1: connected
    uint8_t  radarPresent;  // 0: clear, 1: target tracked
    int16_t  radarDist_mm;  // Radial distance (mm)
    int16_t  radarSpeed;    // Target velocity (cm/s)
    uint8_t  radarTargets;  // Active target count (0–3)
    uint8_t  checksum;      // Byte 0..22 XOR checksum
};
#pragma pack(pop)

// Double-buffered response array to guarantee atomic reads from I2C ISR
volatile TelemetryPacket telemOut;
static uint8_t telemReady[TELEM_SIZE];

void updateTelemChecksum() {
    uint8_t* p = (uint8_t*)&telemOut;
    uint8_t ck = 0;
    for (int i = 0; i < TELEM_SIZE - 1; i++) ck ^= p[i];
    telemOut.checksum = ck;
    memcpy(telemReady, (const void*)&telemOut, TELEM_SIZE);
}

volatile unsigned long lastI2cRequest = 0;

void onI2CRequest() {
    lastI2cRequest = millis();
    Wire1.write(telemReady, TELEM_SIZE);
}

// Kinematic State
float ax = 0, ay = 0, az = 1.0;
float gx = 0, gy = 0, gz = 0;
float totalAccel = 1.0;

// Fall Detection Parameters
#define ML_FALL_THRESHOLD       85    // Minimum classifier confidence percentage
#define STILLNESS_CHECK_MS      3000  // Post-impact stillness window
#define STILLNESS_G_THRESHOLD   0.25  // Absolute acceleration delta threshold (|a| - 1g)
#define STILLNESS_REQUIRED      25    // Required contiguous still samples
#define FALL_COOLDOWN_MS        30000 // Re-trigger suppression window

int fallDetected = 0;
unsigned long fallTriggerTime = 0;
unsigned long lastFallTime = 0;
int stillnessCount = 0;

uint8_t mlFall = 0, mlFalseAlarm = 0, mlIdle = 0, mlWalk = 0;
uint8_t mlWinnerIdx = 0, mlConfidence = 0, mlFlags = 0;

// Actuator & Peripherals State
bool relayActive = false;
unsigned long relayStartMs = 0;
bool wearableLinked = false;
int batteryPercent = 0;
int32_t currentPressurePa = 0;
uint32_t irVal = 0;
uint32_t redVal = 0;
bool sosActive = false;
unsigned long buzzerStartTime = 0;
bool buzzerActive = false;

SerialPIO dfpSerial(PIN_DFP_TX, PIN_DFP_RX);
DFRobotDFPlayerMini myDFPlayer;
bool dfpReady = false;

// Radar State
unsigned long lastRadarTime = 0;
bool radarPresence = false;
float radarDist_m = 0;
int16_t radarSpeed = 0;
int radarTargetCount = 0;
uint32_t imuPackets = 0;

// RD-03D Protocol Definitions
const uint8_t RD03D_FRAME_HEADER[] = {0xAA, 0xFF, 0x03, 0x00};
const uint8_t RD03D_FRAME_FOOTER[] = {0x55, 0xCC};
const uint8_t RD03D_CMD_HEADER[]   = {0xFD, 0xFC, 0xFB, 0xFA};
const uint8_t RD03D_CMD_FOOTER[]   = {0x04, 0x03, 0x02, 0x01};

static constexpr int16_t SPEED_SENTINEL_248 = 248;
static constexpr int16_t SPEED_SENTINEL_256 = 256;
static constexpr uint8_t RD03D_TARGET_DATA_SIZE = 8;
static constexpr uint8_t RD03D_MAX_TARGETS = 3;

// RD-03D sign-magnitude decoding (MSB=1 is positive, MSB=0 is negative)
int16_t rd03d_decode_value(uint8_t low_byte, uint8_t high_byte) {
    int16_t value = ((high_byte & 0x7F) << 8) | low_byte;
    if ((high_byte & 0x80) == 0) {
        value = -value;
    }
    return value;
}

bool rd03d_is_speed_valid(int16_t speed) {
    int16_t abs_speed = speed < 0 ? -speed : speed;
    return abs_speed != SPEED_SENTINEL_248 && abs_speed != SPEED_SENTINEL_256;
}

void setRGB(bool r, bool g, bool b) {
    digitalWrite(PIN_LED_R, r ? HIGH : LOW);
    digitalWrite(PIN_LED_G, g ? HIGH : LOW);
    digitalWrite(PIN_LED_B, b ? HIGH : LOW);
}

void updateLEDs() {
    unsigned long now = millis();
    bool pulse = (now % 1000) < 500;
    bool fastFlash = (now % 200) < 100;

    if (sosActive || fallDetected == 2) {
        setRGB(fastFlash, false, false);
    } else if (relayActive) {
        setRGB(false, pulse, pulse);
    } else if (!wearableLinked) {
        setRGB(false, false, pulse);
    } else if (radarPresence && (now - lastRadarTime < 1000)) {
        setRGB(true, true, false);
    } else {
        setRGB(false, true, false);
    }
}

void processActuators() {
    updateLEDs();
    
    static bool prevAlert = false;
    static bool prevLinked = false;
    bool currentAlert = (sosActive || fallDetected == 2);
    
    if (wearableLinked && !prevLinked && dfpReady) {
        myDFPlayer.playMp3Folder(2); // Link chime
    }
    if (!wearableLinked && prevLinked && dfpReady) {
        myDFPlayer.playMp3Folder(3); // Link lost chime
    }
    if (currentAlert && !prevAlert) {
        buzzerActive = true;
        buzzerStartTime = millis();
        tone(PIN_BUZZER, 2500);
        if (dfpReady) myDFPlayer.playMp3Folder(4); // Alarm siren
    }
    
    if (buzzerActive && millis() - buzzerStartTime > 1500) {
        noTone(PIN_BUZZER);
        buzzerActive = false;
    }
    
    prevAlert = currentAlert;
    prevLinked = wearableLinked;
}

void relayOn() {
    if (!relayActive) {
        digitalWrite(PIN_RELAY, HIGH);
        relayActive = true;
        relayStartMs = millis();
    }
}

void relayOff() {
    if (relayActive) {
        digitalWrite(PIN_RELAY, LOW);
        relayActive = false;
    }
}

void relayTick() {
    if (relayActive && (millis() - relayStartMs >= RELAY_ON_TIME_MS)) {
        relayOff();
    }
}

void updateTelemStruct();

void onI2CReceive(int howMany) {
    if (howMany > 0) {
        uint8_t cmd = Wire1.read();
        if (cmd == 0xEE) {
            // Cancel alarm command received from CrowPanel
            fallDetected = 0;
            sosActive = false;
            updateTelemStruct();
        }
        while (Wire1.available()) Wire1.read();
    }
}

void updateTelemStruct() {
    telemOut.magic = 0xAA;
    telemOut.ax = (int16_t)(ax * 4096.0f);
    telemOut.ay = (int16_t)(ay * 4096.0f);
    telemOut.az = (int16_t)(az * 4096.0f);
    telemOut.gx = (int16_t)(gx * 65.5f);
    telemOut.gy = (int16_t)(gy * 65.5f);
    telemOut.gz = (int16_t)(gz * 65.5f);
    telemOut.battery = batteryPercent;
    telemOut.sos = sosActive ? 1 : 0;
    telemOut.fallState = fallDetected;
    telemOut.wearLink = wearableLinked ? 1 : 0;
    telemOut.radarPresent = radarPresence ? 1 : 0;
    telemOut.radarDist_mm = (int16_t)(radarDist_m * 1000.0f);
    telemOut.radarSpeed = radarSpeed;
    telemOut.radarTargets = radarTargetCount;
    updateTelemChecksum();
}

void processMLResult(uint8_t fall, uint8_t falseAlarm, uint8_t idle, uint8_t walk,
                     uint8_t winnerIdx, uint8_t confidence, uint8_t flags) {
    unsigned long now = millis();

    mlFall = fall;
    mlFalseAlarm = falseAlarm;
    mlIdle = idle;
    mlWalk = walk;
    mlWinnerIdx = winnerIdx;
    mlConfidence = confidence;
    mlFlags = flags;

    if (flags & 0x01) {
        sosActive = true;
    }

    if (now - lastFallTime < FALL_COOLDOWN_MS && fallDetected == 0) return;

    if (fall >= ML_FALL_THRESHOLD && fallDetected == 0) {
        fallDetected = 1;
        fallTriggerTime = now;
        stillnessCount = 0;
        updateTelemStruct();
    }
}

void checkStillness() {
    unsigned long now = millis();

    if (fallDetected != 1) return;

    if (now - fallTriggerTime > STILLNESS_CHECK_MS) {
        // Verification window expired without sustained stillness
        fallDetected = 0;
        stillnessCount = 0;
        updateTelemStruct();
        return;
    }

    float movement = abs(totalAccel - 1.0f);
    if (movement < STILLNESS_G_THRESHOLD) {
        stillnessCount++;
    } else {
        stillnessCount = max(0, stillnessCount - 3);
    }

    if (stillnessCount >= STILLNESS_REQUIRED) {
        fallDetected = 2;
        lastFallTime = now;
        updateTelemStruct();
        relayOn();
    }
}

void fallAutoReset() {
    if (fallDetected == 2 && millis() - lastFallTime > 10000) {
        fallDetected = 0;
        stillnessCount = 0;
        updateTelemStruct();
    }
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); 
    
    Serial.begin(115200);
    
    pinMode(PIN_LED_R, OUTPUT);
    pinMode(PIN_LED_G, OUTPUT);
    pinMode(PIN_LED_B, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);
    setRGB(false, false, false);

    pinMode(PIN_RELAY, OUTPUT);
    digitalWrite(PIN_RELAY, LOW);

    // UART0: Base Station Bridge
    Serial1.setRX(PIN_BASE_RX);
    Serial1.setTX(PIN_BASE_TX);
    Serial1.begin(115200);
    Serial1.setTimeout(10);
    
    // UART1: RD-03D Radar interface
    Serial2.setTX(PIN_RADAR_TX);
    Serial2.setRX(PIN_RADAR_RX);
    Serial2.begin(256000);
    
    // I2C1 Slave interface for CrowPanel master
    Wire1.setSDA(PIN_OLED_SDA);
    Wire1.setSCL(PIN_OLED_SCL);
    Wire1.begin(FUSION_I2C_ADDR);
    Wire1.onRequest(onI2CRequest);
    Wire1.onReceive(onI2CReceive);

    // Audio synthesizer
    dfpSerial.begin(9600);
    delay(100);
    if (myDFPlayer.begin(dfpSerial, true, false)) {
        dfpReady = true;
        delay(100);
        myDFPlayer.volume(30);
    }

    memset((void*)&telemOut, 0, sizeof(telemOut));
    telemOut.magic = 0xAA;
    updateTelemChecksum();
}

void loop() {
    processActuators();
    relayTick();
    fallAutoReset();

    unsigned long now = millis();
    
    // 1. Ingest Base Node CSV Stream
    static char baseBuf[300];
    static int baseIdx = 0;
    static bool gotLine = false;
    
    int avail = Serial1.available();
    while (avail-- > 0) {
        char c = Serial1.read();
        if (c == '\n' || c == '\r') {
            if (baseIdx > 2) gotLine = true;
            break;
        } else {
            if (baseIdx < 299) baseBuf[baseIdx++] = c;
            else baseIdx = 0;
        }
    }
    
    if (gotLine) {
        baseBuf[baseIdx] = '\0';
        gotLine = false;
        baseIdx = 0;
        
        if (baseBuf[0] == 'I' && baseBuf[1] == ',') {
            int16_t x, y, z, gxi, gyi, gzi;
            int seq;
            if (sscanf(baseBuf, "I,%hd,%hd,%hd,%hd,%hd,%hd,%d", &x, &y, &z, &gxi, &gyi, &gzi, &seq) >= 6) {
                imuPackets++;
                wearableLinked = true;
                ax = (float)x / 4096.0f;
                ay = (float)y / 4096.0f;
                az = (float)z / 4096.0f;
                gx = (float)gxi / 65.5f;
                gy = (float)gyi / 65.5f;
                gz = (float)gzi / 65.5f;
                totalAccel = sqrt(ax*ax + ay*ay + az*az);

                checkStillness();
                updateTelemStruct();
            }
        } 
        else if (baseBuf[0] == 'E' && baseBuf[1] == ',') {
            long pa;
            unsigned long ir, red;
            int bat, sos;
            unsigned int seq;
            if (sscanf(baseBuf, "E,%ld,%lu,%lu,%d,%d,%u", &pa, &ir, &red, &bat, &sos, &seq) >= 5) {
                currentPressurePa = pa;
                batteryPercent = bat;
                sosActive = (sos != 0);
                updateTelemStruct();
            }
        }
        else if (baseBuf[0] == 'M' && baseBuf[1] == ',') {
            int f, fa, id, wk, wi, co, fl;
            if (sscanf(baseBuf, "M,%d,%d,%d,%d,%d,%d,%d", &f, &fa, &id, &wk, &wi, &co, &fl) >= 7) {
                processMLResult(f, fa, id, wk, wi, co, fl);
            }
        }
        else if (strncmp(baseBuf, "STAT,", 5) == 0) {
            const char* evt = baseBuf + 5;
            if (strcmp(evt, "BASE_LINK_OK") == 0 || strcmp(evt, "W_OK") == 0) {
                wearableLinked = true;
            } else if (strcmp(evt, "LINK_LOST") == 0 || strcmp(evt, "W_LOST") == 0) {
                wearableLinked = false;
            }
            updateTelemStruct();
        }
    }
    
    // 2. Ingest RD-03D mmWave Radar Frame Stream
    static uint8_t rdBuf[64];
    static int rdIdx = 0;
    while (Serial2.available() > 0) {
        uint8_t c = Serial2.read();
        
        if (rdIdx < 4) {
            if (c == RD03D_FRAME_HEADER[rdIdx]) {
                rdBuf[rdIdx++] = c;
            } else if (c == RD03D_FRAME_HEADER[0]) {
                rdBuf[0] = c;
                rdIdx = 1;
            } else {
                rdIdx = 0;
            }
            continue;
        }
        
        rdBuf[rdIdx++] = c;
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
            
            lastRadarTime = now;
            radarTargetCount = 0;
            float bestDist = 0;
            int16_t bestSpeed = 0;
            bool anyTarget = false;
            
            if (frozenCount < 10) {
                for (uint8_t i = 0; i < RD03D_MAX_TARGETS; i++) {
                    uint8_t offset = 4 + (i * RD03D_TARGET_DATA_SIZE);
                    
                    int16_t x = rd03d_decode_value(rdBuf[offset + 0], rdBuf[offset + 1]);
                    int16_t y = rd03d_decode_value(rdBuf[offset + 2], rdBuf[offset + 3]);
                    int16_t speed = rd03d_decode_value(rdBuf[offset + 4], rdBuf[offset + 5]);
                    
                    bool hasPosition = (x != 0 || y != 0);
                    bool hasValidSpeed = rd03d_is_speed_valid(speed);
                    
                    if (hasPosition && hasValidSpeed) {
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
            radarDist_m = bestDist / 1000.0f;
            radarSpeed = bestSpeed;
            
            updateTelemStruct();
            rdIdx = 0;
        }
    }
    
    // Inactivity timeout
    if (now - lastRadarTime > 3000 && radarPresence) {
        radarPresence = false;
        radarTargetCount = 0;
        radarDist_m = 0;
        radarSpeed = 0;
        updateTelemStruct();
    }

    // Heartbeat indicator
    static unsigned long lastBlink = 0;
    if (now - lastBlink > 500) {
        lastBlink = now;
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
}
