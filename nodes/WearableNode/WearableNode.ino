/**
 * RANGER v4.1 — Wearable Node (nRF52840) + Edge Impulse ML
 *
 * ROLE: On-device AI fall detection + raw data relay
 *   1. Samples MPU6050 at 50Hz
 *   2. Runs Edge Impulse classifier on each full window
 *   3. Sends IMU data + ML classification scores over BLE
 *   4. SOS button (3s hold) still works as before
 *
 * BLE: Peripheral, advertises as "RANGER-WEAR"
 */

#include <Arduino.h>
#include <Wire.h>
#include <bluefruit.h>
// Bypass nRF54 core CMSIS conflicts by falling back to software DSP
#define EIDSP_USE_CMSIS_DSP 0
#define EI_CLASSIFIER_TFLITE_ENABLE_CMSIS_NN 0
#include <falldetect_inferencing.h>

// ============== VERSION ==============
#define VERSION "4.1-WEAR-RIGID-ML"

// ============== PIN CONFIGURATION ==============
#define PIN_SDA       4
#define PIN_SCL       5
#define PIN_SOS       1

// ============== SENSOR ==============
#define MPU6050_ADDR  0x68
#define ACCEL_SCALE   4096.0f   // ±8g
#define GYRO_SCALE    65.5f     // ±500°/s

// ============== TIMING ==============
#define BATTERY_CHECK_MS  30000

// ============== BLE UUIDs ==============
#define RANGER_SERVICE_UUID       "833d1814-9988-4e31-8db2-2c67699cd1c1"
// Char1: 9DOF (20 bytes)
#define SENSOR_IMU_CHAR_UUID      "833d2a01-9988-4e31-8db2-2c67699cd1c1"
// Char2: Baro + MAX3010 + Battery (15 bytes)
#define SENSOR_ENV_CHAR_UUID      "833d2a02-9988-4e31-8db2-2c67699cd1c1"
// Char3: ML Classification Result (8 bytes)
#define ML_RESULT_CHAR_UUID       "833d2a04-9988-4e31-8db2-2c67699cd1c1"
// Device Info
#define DEVICE_INFO_CHAR_UUID     "833d2a03-9988-4e31-8db2-2c67699cd1c1"

BLEService        rangerService(RANGER_SERVICE_UUID);
BLECharacteristic imuChar(SENSOR_IMU_CHAR_UUID);
BLECharacteristic envChar(SENSOR_ENV_CHAR_UUID);
BLECharacteristic mlChar(ML_RESULT_CHAR_UUID);
BLECharacteristic deviceInfoChar(DEVICE_INFO_CHAR_UUID);

// ============== GLOBALS ==============
unsigned long lastBatteryCheck = 0;

// Sensor data (Raw)
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;       // Unused but kept for packet compat
int32_t pressure_pa = 0;
uint32_t max_ir = 0;
uint32_t max_red = 0;

// SOS & Battery
unsigned long sosPressTime = 0;
bool sosTriggered = false;
uint8_t batteryPercent = 100;

// BLE
bool bleConnected = false;
uint16_t packetSeq = 0;

// ============== ML INFERENCE ==============
static float   ei_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };
static int     ei_buffer_ix = 0;
constexpr size_t INFERENCE_STRIDE_VALUES =
    (EI_CLASSIFIER_RAW_SAMPLE_COUNT / 2) * EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME;
constexpr size_t INFERENCE_KEEP_VALUES =
    EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - INFERENCE_STRIDE_VALUES;
bool mpuReady = false;
uint32_t mpuReadErrors = 0;

// Latest classification results (sent over BLE)
// Packet format: [fallScore:1][falseAlarmScore:1][idleScore:1][walkScore:1][winnerIdx:1][confidence:1][flags:1][seq:1]
// Scores are 0-100 (percentage * 100, clamped to uint8_t)
static uint8_t mlPacket[8] = { 0 };

// ============== LED HELPERS ==============
void ledOn()  { digitalWrite(PIN_LED, LOW); }
void ledOff() { digitalWrite(PIN_LED, HIGH); }

void blink(int times, int delayMs) {
    for (int i = 0; i < times; i++) {
        ledOn();
        delay(delayMs);
        ledOff();
        delay(delayMs);
    }
}

// ============== SENSOR INITS ==============
bool writeMpuRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool readMpuRegister(uint8_t reg, uint8_t &value) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(MPU6050_ADDR, 1) != 1 || !Wire.available()) return false;
    value = Wire.read();
    return true;
}

bool initMPU6050() {
    Serial.println("Init MPU6050...");
    uint8_t whoAmI = 0;
    if (!readMpuRegister(0x75, whoAmI) ||
        (whoAmI != 0x68 && whoAmI != 0x69)) {
        Serial.printf("[IMU] Unexpected WHO_AM_I: 0x%02X\n", whoAmI);
        return false;
    }
    // Wake, ±8g acceleration, ±500 dps gyro, ~44 Hz DLPF.
    return writeMpuRegister(0x6B, 0x00) &&
           writeMpuRegister(0x1C, 0x10) &&
           writeMpuRegister(0x1B, 0x08) &&
           writeMpuRegister(0x1A, 0x03);
}

bool readMPU6050() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0) return false;

    if (Wire.requestFrom(MPU6050_ADDR, 14) != 14) {
        while (Wire.available()) Wire.read();
        return false;
    }
    if (Wire.available() == 14) {
        ax = (Wire.read() << 8) | Wire.read();
        ay = (Wire.read() << 8) | Wire.read();
        az = (Wire.read() << 8) | Wire.read();

        // Temp
        Wire.read(); Wire.read();

        gx = (Wire.read() << 8) | Wire.read();
        gy = (Wire.read() << 8) | Wire.read();
        gz = (Wire.read() << 8) | Wire.read();
        return true;
    }
    return false;
}

// ============== BLE CONFIG ==============
void setupBLE() {
    Bluefruit.begin();
    Bluefruit.setTxPower(4);
    Bluefruit.setName("RANGER-WEAR");

    rangerService.begin();

    // IMU Characteristic
    imuChar.setProperties(CHR_PROPS_NOTIFY);
    imuChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    imuChar.setMaxLen(20);
    imuChar.begin();

    // ENV + Battery Characteristic
    envChar.setProperties(CHR_PROPS_NOTIFY);
    envChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    envChar.setMaxLen(15);
    envChar.begin();

    // ML Result Characteristic (NEW)
    mlChar.setProperties(CHR_PROPS_NOTIFY);
    mlChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    mlChar.setMaxLen(8);
    mlChar.begin();

    // Device Info
    deviceInfoChar.setProperties(CHR_PROPS_READ);
    deviceInfoChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    deviceInfoChar.setFixedLen(21);
    deviceInfoChar.begin();

    char devInfo[22];
    snprintf(devInfo, sizeof(devInfo), "v4.0-ML");
    deviceInfoChar.write(devInfo, strlen(devInfo));

    Bluefruit.Periph.setConnectCallback([](uint16_t conn_handle) {
        bleConnected = true;
        Serial.println("BLE Connected");
        blink(3, 100);
    });

    Bluefruit.Periph.setDisconnectCallback([](uint16_t conn_handle, uint8_t reason) {
        bleConnected = false;
        Serial.println("BLE Disconnected");
    });

    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addService(rangerService);
    Bluefruit.Advertising.addName();

    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 244);
    Bluefruit.Advertising.setFastTimeout(30);
    Bluefruit.Advertising.start(0);

    // Stable connection interval (15ms - 30ms)
    Bluefruit.Periph.setConnInterval(12, 24);
}

void readBattery() {
    int adc = analogRead(A0);
    float voltage = adc * (3.3f / 1024.0f) * 2.0f; // Voltage divider
    if (voltage >= 4.2f) batteryPercent = 100;
    else if (voltage <= 3.3f) batteryPercent = 0;
    else batteryPercent = (uint8_t)((voltage - 3.3f) / (4.2f - 3.3f) * 100.0f);
}

// ============== DATA TRANSMISSION ==============
void sendSensorData() {
    if (!bleConnected) return;

    // Packet 1: IMU (20 bytes) — raw values, no manipulation
    uint8_t pkt1[20];
    packetSeq++;

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

    // Packet 2: Baro + MAX + Battery (15 bytes) at 10Hz
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

        // Shared ENV contract with Base v4.1:
        // [pressure:4][ir:4][red:4][SOS|battery:1][sequence:2].
        pkt2[12] = (batteryPercent & 0x7F) | (sosTriggered ? 0x80 : 0x00);
        pkt2[13] = packetSeq >> 8;
        pkt2[14] = packetSeq & 0xFF;
        envChar.notify(pkt2, 15);
    }
}

void sendMLResult() {
    if (!bleConnected) return;
    mlChar.notify(mlPacket, 8);
}

// ============== ML INFERENCE ==============
void runInference() {
    signal_t signal;
    numpy::signal_from_buffer(ei_buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);

    ei_impulse_result_t result = { 0 };
    const EI_IMPULSE_ERROR inferenceResult = run_classifier(&signal, &result, false);
    if (inferenceResult != EI_IMPULSE_OK) {
        mlPacket[0] = mlPacket[1] = mlPacket[2] = mlPacket[3] = 0;
        mlPacket[4] = 0xFF;
        mlPacket[5] = 0;
        mlPacket[6] = sosTriggered ? 0x01 : 0x00;
        mlPacket[7] = packetSeq & 0xFF;
        Serial.printf("[ML] Inference failed: %d\n", (int)inferenceResult);
        sendMLResult();
        return;
    }

    // Find the semantic winner and pack scores into BLE packet.
    int semanticWinner = 0;

    // Map Edge Impulse labels to fixed packet positions:
    //   [0] = fall, [1] = false_alarm, [2] = idle, [3] = walking
    uint8_t scores[4] = { 0 };
    bool mapped[4] = { false, false, false, false };
    bool mappingValid = true;

    for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        float val = result.classification[i].value;
        const char* label = result.classification[i].label;

        // Convert to 0-100 scale
        int scaled = (int)lroundf(val * 100.0f);
        if (scaled < 0) scaled = 0;
        if (scaled > 100) scaled = 100;
        uint8_t pct = (uint8_t)scaled;

        // Map label to fixed index
        int semanticIndex = -1;
        if (strstr(label, "fall") != NULL && strstr(label, "alarm") == NULL) {
            semanticIndex = 0;
        } else if (strstr(label, "alarm") != NULL || strstr(label, "false") != NULL) {
            semanticIndex = 1;
        } else if (strstr(label, "idle") != NULL) {
            semanticIndex = 2;
        } else if (strstr(label, "walk") != NULL) {
            semanticIndex = 3;
        }
        if (semanticIndex < 0 || mapped[semanticIndex]) {
            mappingValid = false;
        } else {
            mapped[semanticIndex] = true;
            scores[semanticIndex] = pct;
        }

        Serial.printf("%s: %.2f  ", label, val);
    }
    Serial.println();
    for (int i = 0; i < 4; ++i) {
        if (!mapped[i]) mappingValid = false;
        if (scores[i] > scores[semanticWinner]) semanticWinner = i;
    }

    // Build ML BLE packet
    mlPacket[0] = scores[0];           // fall %
    mlPacket[1] = scores[1];           // false_alarm %
    mlPacket[2] = scores[2];           // idle %
    mlPacket[3] = scores[3];           // walking %
    mlPacket[4] = mappingValid ? (uint8_t)semanticWinner : 0xFF;
    mlPacket[5] = mappingValid ? scores[semanticWinner] : 0;
    mlPacket[6] = (sosTriggered ? 0x01 : 0x00) |
                  (mappingValid ? 0x02 : 0x00); // SOS + valid label map
    mlPacket[7] = packetSeq & 0xFF;    // sequence

    sendMLResult();
}

// ============== SETUP ==============
void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n=== RANGER Wearable ML " VERSION " ===");
    Serial.printf("  Model Freq:   %dHz\n", (int)EI_CLASSIFIER_FREQUENCY);
    Serial.printf("  Window:       %d samples\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT);
    Serial.printf("  Labels:       %d\n", EI_CLASSIFIER_LABEL_COUNT);
    Serial.println("======================================");

    pinMode(PIN_LED, OUTPUT);
    ledOff();

    pinMode(PIN_SOS, INPUT_PULLUP);

    Wire.begin();
    Wire.setClock(400000);

    mpuReady = initMPU6050();
    Serial.printf("MPU6050: %s\n", mpuReady ? "OK" : "FAIL - WILL RETRY");
    Serial.println("  Inference overlap: 50% (new decision every 1 second)");

    setupBLE();
    readBattery();
}

// ============== LOOP ==============
void loop() {
    unsigned long now = millis();

    if (!mpuReady) {
        static unsigned long lastMpuRetry = 0;
        if (now - lastMpuRetry >= 1000) {
            lastMpuRetry = now;
            mpuReady = initMPU6050();
            Serial.printf("[IMU] Retry: %s\n", mpuReady ? "OK" : "FAIL");
        }
    }

    // SOS Button: 3s hold = SOS
    if (digitalRead(PIN_SOS) == LOW) {
        if (sosPressTime == 0) sosPressTime = now;
        if (now - sosPressTime > 3000) {
            sosTriggered = true;
        }
    } else {
        sosPressTime = 0;
        sosTriggered = false;
    }

    // Sampling at exact model frequency using micros()
    static unsigned long lastSample = 0;
    unsigned long nowUs = micros();
    unsigned long interval_us = 1000000UL / EI_CLASSIFIER_FREQUENCY;

    if (nowUs - lastSample >= interval_us) {
        lastSample = nowUs;

        if (!mpuReady || !readMPU6050()) {
            ++mpuReadErrors;
            if ((mpuReadErrors % 50) == 1)
                Serial.printf("[IMU] Read failed (%lu); sample discarded\n", mpuReadErrors);
            mpuReady = false;
            // A gap invalidates the model's evenly sampled two-second window.
            ei_buffer_ix = 0;
        } else {
            sendSensorData();

            // Feed into Edge Impulse rolling buffer.
            ei_buffer[ei_buffer_ix + 0] = ax / ACCEL_SCALE;
            ei_buffer[ei_buffer_ix + 1] = ay / ACCEL_SCALE;
            ei_buffer[ei_buffer_ix + 2] = az / ACCEL_SCALE;
            ei_buffer[ei_buffer_ix + 3] = gx / GYRO_SCALE;
            ei_buffer[ei_buffer_ix + 4] = gy / GYRO_SCALE;
            ei_buffer[ei_buffer_ix + 5] = gz / GYRO_SCALE;
            ei_buffer_ix += EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME;

            if (ei_buffer_ix >= EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
                runInference();
                // Keep the newest half-window, so a fall crossing a 2-second
                // boundary is still evaluated in the next overlapped window.
                memmove(ei_buffer, ei_buffer + INFERENCE_STRIDE_VALUES,
                        INFERENCE_KEEP_VALUES * sizeof(float));
                ei_buffer_ix = INFERENCE_KEEP_VALUES;
            }
        }
    }

    // Battery & Info
    if (now - lastBatteryCheck >= BATTERY_CHECK_MS) {
        lastBatteryCheck = now;
        readBattery();
    }

    // Status LED
    static unsigned long lastLedToggle = 0;
    if (sosTriggered) {
        if (now - lastLedToggle > 100) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); lastLedToggle = now; }
    } else if (bleConnected) {
        ledOn();
    } else {
        if (now - lastLedToggle > 1000) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); lastLedToggle = now; }
    }
}
