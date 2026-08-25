/**
 * RANGER — Wearable Sensor Node (Seeed XIAO nRF52840)
 * 
 * Functions:
 *   1. Samples MPU6050 6-DoF IMU at 50Hz via hardware I2C (400kHz).
 *   2. Evaluates on-device Edge Impulse ML classifier window.
 *   3. Broadcasts raw kinematics and classification scores over BLE notifications.
 *   4. Monitors emergency SOS button hold (3-second debounce).
 */

#include <Arduino.h>
#include <Wire.h>
#include <bluefruit.h>

#define EIDSP_USE_CMSIS_DSP 0
#define EI_CLASSIFIER_TFLITE_ENABLE_CMSIS_NN 0
#include <falldetect_inferencing.h>

// Exception stubs for bare-metal nRF core
namespace std {
    void __throw_bad_function_call() { while(1); }
    void __throw_length_error(char const*) { while(1); }
    void __throw_bad_alloc() { while(1); }
}

#define VERSION "4.0-WEAR-ML"

// Pin Assignments
#define PIN_SDA       4
#define PIN_SCL       5
#define PIN_SOS       1

// Sensor Scaling
#define MPU6050_ADDR  0x68
#define ACCEL_SCALE   4096.0f   // ±8g range
#define GYRO_SCALE    65.5f     // ±500 deg/s range

#define BATTERY_CHECK_MS  30000

// BLE UUID Definitions
#define RANGER_SERVICE_UUID       "833d1814-9988-4e31-8db2-2c67699cd1c1"
#define SENSOR_IMU_CHAR_UUID      "833d2a01-9988-4e31-8db2-2c67699cd1c1"
#define SENSOR_ENV_CHAR_UUID      "833d2a02-9988-4e31-8db2-2c67699cd1c1"
#define ML_RESULT_CHAR_UUID       "833d2a04-9988-4e31-8db2-2c67699cd1c1"
#define DEVICE_INFO_CHAR_UUID     "833d2a03-9988-4e31-8db2-2c67699cd1c1"

BLEService        rangerService(RANGER_SERVICE_UUID);
BLECharacteristic imuChar(SENSOR_IMU_CHAR_UUID);
BLECharacteristic envChar(SENSOR_ENV_CHAR_UUID);
BLECharacteristic mlChar(ML_RESULT_CHAR_UUID);
BLECharacteristic deviceInfoChar(DEVICE_INFO_CHAR_UUID);

// Telemetry State
unsigned long lastBatteryCheck = 0;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx = 0, my = 0, mz = 0;
int32_t pressure_pa = 0;
uint32_t max_ir = 0;
uint32_t max_red = 0;

unsigned long sosPressTime = 0;
bool sosTriggered = false;
uint8_t batteryPercent = 100;

bool bleConnected = false;
uint16_t packetSeq = 0;

// Edge Impulse Rolling Inference Buffer
static float   ei_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };
static int     ei_buffer_ix = 0;
static bool    ei_buffer_full = false;

// ML packet: [fall%][false_alarm%][idle%][walk%][winnerIdx][confidence%][flags][seq]
static uint8_t mlPacket[8] = { 0 };

inline void ledOn()  { digitalWrite(PIN_LED, LOW); }
inline void ledOff() { digitalWrite(PIN_LED, HIGH); }

void blink(int times, int delayMs) {
    for (int i = 0; i < times; i++) {
        ledOn();
        delay(delayMs);
        ledOff();
        delay(delayMs);
    }
}

bool initMPU6050() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);  // PWR_MGMT_1
    Wire.write(0x00);  // Wake device
    if (Wire.endTransmission() != 0) return false;
    
    // Set Accelerometer to ±8g
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();

    // Set Gyroscope to ±500 deg/s
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1B);
    Wire.write(0x08);
    Wire.endTransmission();

    // Digital Low Pass Filter ~44Hz bandwidth
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1A);
    Wire.write(0x03);
    Wire.endTransmission();
    
    return true;
}

void readMPU6050() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (size_t)14);
    if (Wire.available() == 14) {
        ax = (Wire.read() << 8) | Wire.read();
        ay = (Wire.read() << 8) | Wire.read();
        az = (Wire.read() << 8) | Wire.read();
        
        Wire.read(); Wire.read(); // Discard die temperature
        
        gx = (Wire.read() << 8) | Wire.read();
        gy = (Wire.read() << 8) | Wire.read();
        gz = (Wire.read() << 8) | Wire.read();
    }
}

void setupBLE() {
    Bluefruit.begin();
    Bluefruit.setTxPower(4);
    Bluefruit.setName("RANGER-WEAR");

    rangerService.begin();
    
    // 20-byte 9-DoF IMU Characteristic
    imuChar.setProperties(CHR_PROPS_NOTIFY);
    imuChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    imuChar.setMaxLen(20);
    imuChar.begin();
    
    // 15-byte Environmental & Battery Characteristic
    envChar.setProperties(CHR_PROPS_NOTIFY);
    envChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    envChar.setMaxLen(15);
    envChar.begin();

    // 8-byte ML Classifier Characteristic
    mlChar.setProperties(CHR_PROPS_NOTIFY);
    mlChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    mlChar.setMaxLen(8);
    mlChar.begin();
    
    deviceInfoChar.setProperties(CHR_PROPS_READ);
    deviceInfoChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    deviceInfoChar.setFixedLen(21);
    deviceInfoChar.begin();
    
    char devInfo[22];
    snprintf(devInfo, sizeof(devInfo), "v4.0-ML");
    deviceInfoChar.write(devInfo, strlen(devInfo));

    Bluefruit.Periph.setConnectCallback([](uint16_t conn_handle) {
        bleConnected = true;
        blink(3, 100);
    });
    
    Bluefruit.Periph.setDisconnectCallback([](uint16_t conn_handle, uint8_t reason) {
        bleConnected = false;
    });

    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addService(rangerService);
    Bluefruit.Advertising.addName();
    
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 244);
    Bluefruit.Advertising.setFastTimeout(30);
    Bluefruit.Advertising.start(0);

    // Connection interval: 15ms - 30ms
    Bluefruit.Periph.setConnInterval(12, 24);
}

void readBattery() {
    int adc = analogRead(A0);
    float voltage = adc * (3.3f / 1024.0f) * 2.0f;
    if (voltage >= 4.2f) batteryPercent = 100;
    else if (voltage <= 3.3f) batteryPercent = 0;
    else batteryPercent = (uint8_t)((voltage - 3.3f) / (4.2f - 3.3f) * 100.0f);
}

void sendSensorData() {
    if (!bleConnected) return;
    
    packetSeq++;
    
    // Packet 1: Raw IMU (20 bytes)
    uint8_t pkt1[20];
    pkt1[0] = ax >> 8; pkt1[1] = ax & 0xFF;
    pkt1[2] = ay >> 8; pkt1[3] = ay & 0xFF;
    pkt1[4] = az >> 8; pkt1[5] = az & 0xFF;
    pkt1[6] = gx >> 8; pkt1[7] = gx & 0xFF;
    pkt1[8] = gy >> 8; pkt1[9] = gy & 0xFF;
    pkt1[10] = gz >> 8; pkt1[11] = gz & 0xFF;
    pkt1[12] = mx >> 8; pkt1[13] = mx & 0xFF;
    pkt1[14] = my >> 8; pkt1[15] = my & 0xFF;
    pkt1[16] = mz >> 8; pkt1[17] = mz & 0xFF;
    pkt1[18] = packetSeq >> 8; pkt1[19] = packetSeq & 0xFF;
    imuChar.notify(pkt1, 20);
    
    // Packet 2: Environmental & Status (15 bytes) at 10Hz
    static uint8_t envCounter = 0;
    if (++envCounter >= 5) {
        envCounter = 0;
        uint8_t pkt2[15];
        
        pkt2[0] = (pressure_pa >> 24) & 0xFF;
        pkt2[1] = (pressure_pa >> 16) & 0xFF;
        pkt2[2] = (pressure_pa >> 8) & 0xFF;
        pkt2[3] = pressure_pa & 0xFF;
        
        pkt2[4] = (max_ir >> 24) & 0xFF;
        pkt2[5] = (max_ir >> 16) & 0xFF;
        pkt2[6] = (max_ir >> 8) & 0xFF;
        pkt2[7] = max_ir & 0xFF;
        
        pkt2[8] = (max_red >> 24) & 0xFF;
        pkt2[9] = (max_red >> 16) & 0xFF;
        pkt2[10] = (max_red >> 8) & 0xFF;
        pkt2[11] = max_red & 0xFF;
        
        pkt2[12] = batteryPercent;
        pkt2[13] = sosTriggered ? 1 : 0;
        pkt2[14] = packetSeq & 0xFF;
        envChar.notify(pkt2, 15);
    }
}

void sendMLResult() {
    if (!bleConnected) return;
    mlChar.notify(mlPacket, 8);
}

void runInference() {
    signal_t signal;
    numpy::signal_from_buffer(ei_buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);

    ei_impulse_result_t result = { 0 };
    run_classifier(&signal, &result, false);

    float maxVal = 0;
    int   maxIdx = 0;
    uint8_t scores[4] = { 0 };

    for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        float val = result.classification[i].value;
        const char* label = result.classification[i].label;
        uint8_t pct = (uint8_t)(val * 100.0f);

        if (strstr(label, "fall") != NULL && strstr(label, "alarm") == NULL) {
            scores[0] = pct;
        } else if (strstr(label, "alarm") != NULL || strstr(label, "false") != NULL) {
            scores[1] = pct;
        } else if (strstr(label, "idle") != NULL) {
            scores[2] = pct;
        } else if (strstr(label, "walk") != NULL) {
            scores[3] = pct;
        }

        if (val > maxVal) {
            maxVal = val;
            maxIdx = i;
        }
    }

    mlPacket[0] = scores[0];           // Fall %
    mlPacket[1] = scores[1];           // False alarm %
    mlPacket[2] = scores[2];           // Idle %
    mlPacket[3] = scores[3];           // Walking %
    mlPacket[4] = (uint8_t)maxIdx;     // Highest score index
    mlPacket[5] = (uint8_t)(maxVal * 100.0f); // Confidence
    mlPacket[6] = sosTriggered ? 0x01 : 0x00;
    mlPacket[7] = packetSeq & 0xFF;

    sendMLResult();
}

void setup() {
    Serial.begin(115200);
    
    pinMode(PIN_LED, OUTPUT);
    ledOff();
    pinMode(PIN_SOS, INPUT_PULLUP);
    
    Wire.begin();
    Wire.setClock(400000);

    initMPU6050();
    setupBLE();
    readBattery();
}

void loop() {
    unsigned long now = millis();
    
    // SOS button 3-second hold detection
    if (digitalRead(PIN_SOS) == LOW) {
        if (sosPressTime == 0) sosPressTime = now;
        if (now - sosPressTime > 3000) {
            sosTriggered = true;
        }
    } else {
        sosPressTime = 0;
        sosTriggered = false;
    }
    
    // Fixed rate sensor acquisition
    static unsigned long lastSample = 0;
    unsigned long nowUs = micros();
    unsigned long interval_us = 1000000UL / EI_CLASSIFIER_FREQUENCY;

    if (nowUs - lastSample >= interval_us) {
        lastSample = nowUs;
        
        readMPU6050();
        sendSensorData();

        // Feed rolling DSP buffer
        ei_buffer[ei_buffer_ix + 0] = ax / ACCEL_SCALE;
        ei_buffer[ei_buffer_ix + 1] = ay / ACCEL_SCALE;
        ei_buffer[ei_buffer_ix + 2] = az / ACCEL_SCALE;
        ei_buffer[ei_buffer_ix + 3] = gx / GYRO_SCALE;
        ei_buffer[ei_buffer_ix + 4] = gy / GYRO_SCALE;
        ei_buffer[ei_buffer_ix + 5] = gz / GYRO_SCALE;
        ei_buffer_ix += EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME;

        if (ei_buffer_ix >= EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
            ei_buffer_ix = 0;
            ei_buffer_full = true;
        }

        if (ei_buffer_full) {
            ei_buffer_full = false;
            runInference();
        }
    }
    
    if (now - lastBatteryCheck >= BATTERY_CHECK_MS) {
        lastBatteryCheck = now;
        readBattery();
    }
    
    // Status LED logic
    static unsigned long lastLedToggle = 0;
    if (sosTriggered) {
        if (now - lastLedToggle > 100) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); lastLedToggle = now; }
    } else if (bleConnected) {
        ledOn();
    } else {
        if (now - lastLedToggle > 1000) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); lastLedToggle = now; }
    }
}
