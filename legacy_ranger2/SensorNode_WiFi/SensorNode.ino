/**
 * Fall Detection Sensor Node - ESP32-C6
 * 
 * Reads MPU6050 accelerometer/gyroscope data
 * Detects falls and sends data to Base Station via WiFi TCP
 * 
 * Hardware:
 * - MPU6050: SDA=GPIO0, SCL=GPIO1
 * - Status LED: GPIO15
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_wifi.h>

// ============== CONFIGURATION ==============
#define VERSION "1.1-DEBUG"

// WiFi - Connect to Base Station AP
const char* WIFI_SSID = "FALLDETECT_BASE";
const char* WIFI_PASS = "falldetect1234";
const char* SERVER_IP = "192.168.4.1";  // ESP32-S3 AP default IP
const int SERVER_PORT = 8080;

// Pins
#define PIN_SDA 0
#define PIN_SCL 1
#define PIN_LED 15  // Status LED
#define PIN_SOS 9  // SOS Button

// MPU6050
#define MPU_ADDR 0x68
#define SAMPLE_RATE_MS 20  // 50Hz

// Fall detection thresholds (tuned to reduce false alarms)
#define IMPACT_THRESHOLD 3.0     // g - Significant impact (was 2.5)
#define FREEFALL_THRESHOLD 0.3   // g - Free fall detection
#define STILLNESS_THRESHOLD 0.25 // g - Post-impact stillness
#define STILLNESS_SAMPLES 25     // ~0.5 seconds at 50Hz
#define CONFIRMATION_WINDOW 3000 // ms - Window to confirm fall
#define COOLDOWN_PERIOD 10000    // ms - Prevent rapid retriggering
#define ORIENT_THRESHOLD 120.0   // deg/s - Orientation change rate for fall signature

// ============== GLOBALS ==============
WiFiClient client;
bool connected = false;
unsigned long lastSample = 0;
unsigned long lastReconnect = 0;
unsigned long lastDebugPrint = 0;

// SOS Button
unsigned long sosPressTime = 0;
bool sosTriggered = false;

// Accelerometer data
float ax, ay, az;
float gx, gy, gz;
float totalAccel;

// Fall detection
int fallDetected = 0;      // 0=none, 1=possible, 2=confirmed
float impactMagnitude = 0; 
unsigned long impactTime = 0;
int stillnessCount = 0;

// Orientation tracking (gyro integration)
float orientChange = 0;    // Integrated orientation change magnitude (deg/s)
float peakOrient = 0;      // Peak orientation change during fall event

void blink(int times, int delayMs) {
    for (int i=0; i<times; i++) {
        digitalWrite(PIN_LED, LOW); // LED ON (active low?) or HIGH? S3 Zero is RGB, C6 SuperMini is usually GPIO8 active low? 
        // Docs say GPIO8 is LED. Usually active LOW on these minis.
        digitalWrite(PIN_LED, LOW); 
        delay(delayMs);
        digitalWrite(PIN_LED, HIGH);
        delay(delayMs);
    }
}

// ============== MPU6050 ==============
bool initMPU6050() {
    Serial.print("Init MPU6050 on SDA=");
    Serial.print(PIN_SDA);
    Serial.print(" SCL=");
    Serial.println(PIN_SCL);

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);  // PWR_MGMT_1
    Wire.write(0x00);  // Wake up
    uint8_t err = Wire.endTransmission();
    
    if (err != 0) {
        Serial.printf("❌ I2C Error: %d\n", err);
        return false;
    }
    
    // Configure accelerometer ±8g
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1C);
    Wire.write(0x10);  // ±8g
    Wire.endTransmission();
    
    // Configure gyroscope ±500°/s
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1B);
    Wire.write(0x08);  // ±500°/s
    Wire.endTransmission();
    
    // Low-pass filter
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1A);
    Wire.write(0x03);  // ~43Hz bandwidth
    Wire.endTransmission();
    
    Serial.println("✅ MPU6050 Configured");
    return true;
}

void readMPU6050() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);  // Start at ACCEL_XOUT_H
    if (Wire.endTransmission(false) != 0) {
        Serial.println("I2C Read Error");
        return;
    }
    
    Wire.requestFrom(MPU_ADDR, 14, true);
    if (Wire.available() < 14) {
        // Serial.println("Incomplete I2C data");
        return;
    }
    
    int16_t raw_ax = (Wire.read() << 8) | Wire.read();
    int16_t raw_ay = (Wire.read() << 8) | Wire.read();
    int16_t raw_az = (Wire.read() << 8) | Wire.read();
    int16_t raw_temp = (Wire.read() << 8) | Wire.read();
    int16_t raw_gx = (Wire.read() << 8) | Wire.read();
    int16_t raw_gy = (Wire.read() << 8) | Wire.read();
    int16_t raw_gz = (Wire.read() << 8) | Wire.read();
    
    // Convert to physical units
    ax = raw_ax / 4096.0;
    ay = raw_ay / 4096.0;
    az = raw_az / 4096.0;
    
    gx = raw_gx / 65.5;
    gy = raw_gy / 65.5;
    gz = raw_gz / 65.5;
    
    totalAccel = sqrt(ax*ax + ay*ay + az*az);
    
    if (millis() - lastDebugPrint > 2000) {
         Serial.printf("MPU: ax=%.2f ay=%.2f az=%.2f Total=%.2fg\n", ax, ay, az, totalAccel);
         lastDebugPrint = millis();
    }
}

// ============== FALL DETECTION (Improved Algorithm + Gyro) ==============
void detectFall() {
    unsigned long now = millis();
    static float prevAccel = 1.0;
    static unsigned long lastFallTime = 0;
    static bool freeFallPhase = false;
    static unsigned long freeFallStart = 0;
    
    // Track orientation change from gyroscope (body rotation)
    // Magnitude of angular velocity vector (deg/s)
    float gyroMag = sqrt(gx*gx + gy*gy + gz*gz);
    
    // Exponential moving average of orientation change
    orientChange = orientChange * 0.7 + gyroMag * 0.3;
    
    // Track peak during a fall event
    if (fallDetected >= 1) {
        if (gyroMag > peakOrient) peakOrient = gyroMag;
    }
    
    // Cooldown: prevent rapid retriggering
    if (now - lastFallTime < COOLDOWN_PERIOD && fallDetected == 0) {
        return;
    }
    
    // Calculate acceleration change rate
    float accelChange = abs(totalAccel - prevAccel);
    prevAccel = totalAccel;
    
    // Phase 1: Detect free-fall (< 0.3g for brief moment)
    if (totalAccel < FREEFALL_THRESHOLD && !freeFallPhase && fallDetected == 0) {
        freeFallPhase = true;
        freeFallStart = now;
        Serial.println("⚠️ Free-fall phase started");
    }
    
    // Phase 2: Impact Analysis
    if (totalAccel > IMPACT_THRESHOLD && fallDetected == 0) {
        bool hadFreeFall = freeFallPhase && (now - freeFallStart < 500);
        bool isHardImpact = totalAccel > 5.0;
        bool hadRotation = orientChange > ORIENT_THRESHOLD; // Significant body rotation
        
        // Trigger if: (FreeFall + Impact) OR (Very Hard Impact) OR (Rotation + Impact)
        if (hadFreeFall || isHardImpact || hadRotation) {
            fallDetected = 1;
            impactMagnitude = totalAccel;
            impactTime = now;
            stillnessCount = 0;
            peakOrient = gyroMag; // Start tracking peak
            
            if (hadFreeFall && hadRotation) {
                Serial.printf("⚠️ Full Fall: FreeFall→Rotation(%.0f°/s)→Impact(%.2fg)\n", orientChange, totalAccel);
                impactMagnitude *= 1.4; // Highest confidence boost
            } else if (hadFreeFall) {
                Serial.printf("⚠️ Fall: FreeFall→Impact(%.2fg)\n", totalAccel);
                impactMagnitude *= 1.2;
            } else if (hadRotation) {
                Serial.printf("⚠️ Fall: Rotation(%.0f°/s)→Impact(%.2fg)\n", orientChange, totalAccel);
                impactMagnitude *= 1.1;
            } else {
                Serial.printf("⚠️ Hard Impact(%.2fg) no FreeFall/Rotation\n", totalAccel);
            }
        } else {
            Serial.printf("ℹ️ Impact(%.2fg) ignored - No FreeFall/Rotation\n", totalAccel);
        }
        freeFallPhase = false;
    }
    
    // Reset free-fall if time expires
    if (freeFallPhase && now - freeFallStart > 600) {
        freeFallPhase = false;
    }
    
    // Phase 3: Check for post-impact stillness (confirmation)
    if (fallDetected == 1) {
        float movement = abs(totalAccel - 1.0);
        
        if (movement < STILLNESS_THRESHOLD) {
            stillnessCount++;
        } else {
            stillnessCount = max(0, stillnessCount - 3);
        }
        
        // Confirm fall if still for ~0.5 seconds
        if (stillnessCount >= STILLNESS_SAMPLES) {
            fallDetected = 2;
            lastFallTime = now;
            Serial.printf("🚨 FALL CONFIRMED! (PeakRotation: %.0f°/s)\n", peakOrient);
        }
        
        // Cancel if no stillness within confirmation window
        if (now - impactTime > CONFIRMATION_WINDOW && fallDetected == 1) {
            Serial.println("✓ Impact cleared (no stillness)");
            fallDetected = 0;
            impactMagnitude = 0;
            stillnessCount = 0;
            peakOrient = 0;
        }
    }
    
    // Phase 4: Auto-clear confirmed fall after 5 seconds
    if (fallDetected == 2 && now - impactTime > 5000) {
        fallDetected = 0;
        impactMagnitude = 0;
        peakOrient = 0;
    }
}

// ============== NETWORKING ==============
void connectWiFi() {
    if (WiFi.status() == WL_CONNECTED) return;
    
    Serial.printf("\nConnecting to SSID: %s\n", WIFI_SSID);
    
    WiFi.mode(WIFI_STA);
    esp_wifi_set_ps(WIFI_PS_NONE);  // Disable power saving for fast response
    WiFi.setTxPower(WIFI_POWER_19_5dBm);  // Max TX power
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    
    // Non-blocking wait: max 5 seconds (was 15s)
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 10) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n✅ WiFi Connected!");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.printf("\n❌ WiFi Fail. Status: %d\n", WiFi.status());
    }
}

void connectServer() {
    if (client.connected()) return;
    
    if (WiFi.status() != WL_CONNECTED) {
        connected = false;
        return;
    }
    
    Serial.printf("Connecting to Base Station (%s:%d)... ", SERVER_IP, SERVER_PORT);
    client.setTimeout(2);  // 2s connect timeout (fast fail)
    if (client.connect(SERVER_IP, SERVER_PORT)) {
        client.setNoDelay(true);  // Disable Nagle — send IMU data immediately
        Serial.println("✅ OK");
        connected = true;
    } else {
        Serial.println("❌ Fail");
        connected = false;
    }
}

void sendData() {
    if (!connected) return;
    
    char json[256];
    snprintf(json, sizeof(json),
        "{\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,"
        "\"gx\":%.1f,\"gy\":%.1f,\"gz\":%.1f,"
        "\"fall\":%d,\"impact\":%.2f,\"orient\":%.1f}\n",
        ax, ay, az, gx, gy, gz, fallDetected, impactMagnitude, peakOrient);
    
    if (client.print(json) == 0) {
        Serial.println("❌ Send Failed");
        client.stop();
        connected = false;
    }
}

// ============== SETUP ==============
void setup() {
    Serial.begin(115200);
    delay(1000); // Shorter startup delay
    Serial.println("\n=== Fall Detection Sensor Node v" VERSION " ===");
    
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH); // Off
    
    // I2C
    Wire.begin(PIN_SDA, PIN_SCL);
    pinMode(PIN_SOS, INPUT_PULLUP);
    
    if (initMPU6050()) {
        blink(2, 100);
    } else {
        Serial.println("❌ MPU6050 FAIL - Check Wiring! Continuing to WiFi...");
    }
    
    connectWiFi();
    Serial.println("Ready loop...");
}

// ============== LOOP ==============
void loop() {
    unsigned long now = millis();
    
    // SOS Button Handling
    if (digitalRead(PIN_SOS) == LOW) {
        if (sosPressTime == 0) sosPressTime = now;
        
        if (now - sosPressTime > 1000 && !sosTriggered) {
             sosTriggered = true;
             // Force severe fall (High severity = lower timeout)
             // S3 logic: Timeout = 30s - (Severity * 25s)
             // We want 5s timeout -> Severity 1.0 -> Impact 5.0g
             fallDetected = 2; // Confirmed
             impactMagnitude = 6.0; 
             Serial.println("🚨 SOS TRIGGERED!");
             blink(5, 50);
        }
    } else {
        sosPressTime = 0;
        sosTriggered = false;
    }
    
    // Non-blocking reconnect: only try every 3 seconds (was 5s with full blocking scan)
    if (now - lastReconnect > 3000) {
        lastReconnect = now;
        if (WiFi.status() != WL_CONNECTED) {
            // Quick non-blocking reconnect attempt
            WiFi.disconnect(false);
            WiFi.begin(WIFI_SSID, WIFI_PASS);
        } else if (!client.connected()) {
            connectServer();
        }
    }
    
    if (now - lastSample >= SAMPLE_RATE_MS) {
        lastSample = now;
        readMPU6050();
        detectFall();
        sendData();
    }
    
    // LED Status Indication
    // Solid ON = Connected to server
    // Slow blink = WiFi OK, no server
    // Fast blink = No WiFi
    static unsigned long lastLedToggle = 0;
    if (connected) {
        digitalWrite(PIN_LED, LOW);  // Solid ON (active low)
    } else if (WiFi.status() == WL_CONNECTED) {
        // Slow blink - WiFi OK, no server
        if (now - lastLedToggle > 500) {
            digitalWrite(PIN_LED, !digitalRead(PIN_LED));
            lastLedToggle = now;
        }
    } else {
        // Fast blink - No WiFi
        if (now - lastLedToggle > 100) {
            digitalWrite(PIN_LED, !digitalRead(PIN_LED));
            lastLedToggle = now;
        }
    }
}
