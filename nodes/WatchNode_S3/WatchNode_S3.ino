/**
 * RANGER — WatchNode S3 (ESP32-S3 SuperMini)
 * 
 * Hardware:
 *   - Microcontroller: ESP32-S3 SuperMini
 *   - IMU: MPU9250 (0x68) + AK8963 Magnetometer (0x0C)
 *   - Barometer: BMP280 (0x76)
 *   - Actuators: WS2812B NeoPixel (GPIO 48), Onboard Status LED (GPIO 2)
 *   - Input: Boot / SOS Button (GPIO 0, 3-second hold)
 *   - Radio: Connectionless ESP-NOW broadcasting with dynamic base station channel discovery
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>
#include <Adafruit_NeoPixel.h>
#include <_m_rd_me_inferencing.h>

#define VERSION "6.0-WATCH-S3-ESPNOW"

// Hardware Pin Configuration
#define PIN_SDA         8
#define PIN_SCL         9
#define PIN_SOS         0    // BOOT button
#define PIN_LED         2    // Status LED
#define PIN_NEOPIXEL    48   // Onboard RGB LED

// Sensor I2C Addresses
#define MPU9250_ADDR    0x68
#define AK8963_ADDR     0x0C
#define BMP280_ADDR     0x76

// Lightweight Hardware BMP280 Driver
class BMP280_Mini {
public:
    bool begin(uint8_t addr = 0x76) {
        _addr = addr;
        uint8_t id = readReg8(0xD0);
        if (id != 0x58) return false;

        dig_T1 = readReg16_LE(0x88);
        dig_T2 = (int16_t)readReg16_LE(0x8A);
        dig_T3 = (int16_t)readReg16_LE(0x8C);
        dig_P1 = readReg16_LE(0x8E);
        dig_P2 = (int16_t)readReg16_LE(0x90);
        dig_P3 = (int16_t)readReg16_LE(0x92);
        dig_P4 = (int16_t)readReg16_LE(0x94);
        dig_P5 = (int16_t)readReg16_LE(0x96);
        dig_P6 = (int16_t)readReg16_LE(0x98);
        dig_P7 = (int16_t)readReg16_LE(0x9A);
        dig_P8 = (int16_t)readReg16_LE(0x9C);
        dig_P9 = (int16_t)readReg16_LE(0x9E);

        writeReg8(0xF5, (0x04 << 5) | (0x02 << 2));
        writeReg8(0xF4, (0x02 << 5) | (0x05 << 2) | 0x03);
        return true;
    }

    int32_t readPressure() {
        int32_t adc_T = readReg20(0xFA);
        int32_t adc_P = readReg20(0xF7);
        int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
        int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
        int32_t t_fine = var1 + var2;
        int64_t v1 = ((int64_t)t_fine) - 128000;
        int64_t v2 = v1 * v1 * (int64_t)dig_P6;
        v2 = v2 + ((v1 * (int64_t)dig_P5) << 17);
        v2 = v2 + (((int64_t)dig_P4) << 35);
        v1 = ((v1 * v1 * (int64_t)dig_P3) >> 8) + ((v1 * (int64_t)dig_P2) << 12);
        v1 = (((((int64_t)1) << 47) + v1)) * ((int64_t)dig_P1) >> 33;
        if (v1 == 0) return 0;
        int64_t p = 1048576 - adc_P;
        p = (((p << 31) - v2) * 3125) / v1;
        v1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
        v2 = (((int64_t)dig_P8) * p) >> 19;
        p = ((p + v1 + v2) >> 8) + (((int64_t)dig_P7) << 4);
        return (int32_t)(p >> 8);
    }

private:
    uint8_t _addr;
    uint16_t dig_T1, dig_P1;
    int16_t dig_T2, dig_T3;
    int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

    void writeReg8(uint8_t reg, uint8_t val) {
        Wire.beginTransmission(_addr);
        Wire.write(reg); Wire.write(val);
        Wire.endTransmission();
    }
    uint8_t readReg8(uint8_t reg) {
        Wire.beginTransmission(_addr);
        Wire.write(reg);
        Wire.endTransmission();
        Wire.requestFrom(_addr, (uint8_t)1);
        return Wire.read();
    }
    uint16_t readReg16_LE(uint8_t reg) {
        Wire.beginTransmission(_addr);
        Wire.write(reg);
        Wire.endTransmission();
        Wire.requestFrom(_addr, (uint8_t)2);
        uint8_t lo = Wire.read();
        uint8_t hi = Wire.read();
        return (hi << 8) | lo;
    }
    int32_t readReg20(uint8_t reg) {
        Wire.beginTransmission(_addr);
        Wire.write(reg);
        Wire.endTransmission();
        Wire.requestFrom(_addr, (uint8_t)3);
        uint8_t msb = Wire.read();
        uint8_t lsb = Wire.read();
        uint8_t xlsb = Wire.read();
        return ((int32_t)msb << 12) | ((int32_t)lsb << 4) | (xlsb >> 4);
    }
};

#define SAMPLE_RATE_MS    20     // 50Hz acquisition rate
#define BARO_RATE_MS      1000   // 1Hz environmental rate

// ESP-NOW Protocol
uint8_t broadcastMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
#define ESPNOW_CHANNEL 1

#pragma pack(push, 1)
struct SensorPacket {
    uint8_t  magic;       // 0xFA
    int16_t  ax, ay, az;  // Raw accel (4096 LSB/g)
    int16_t  gx, gy, gz;  // Raw gyro (65.5 LSB/dps)
    uint8_t  fall;        // 0: none, 1: verifying, 2: confirmed
    uint8_t  impact;      // Impact magnitude * 10
    uint8_t  orient;      // Peak orientation change / 2
    uint8_t  mlFallScore; // Classifier fall score (0–100%)
    uint8_t  mlConf;      // Highest class confidence (0–100%)
    uint8_t  mlFlags;     // Bit 0 = SOS
    uint8_t  seq;         // Packet sequence counter
};
#pragma pack(pop)

int16_t raw_ax, raw_ay, raw_az;
int16_t raw_gx, raw_gy, raw_gz;
int16_t raw_mx, raw_my, raw_mz;

float ax, ay, az;
float gx, gy, gz;
float totalAccel = 1.0;
int32_t pressure_pa = 0;

// Fall Detection Parameters
#define ML_FALL_THRESHOLD    85
#define STILLNESS_THRESHOLD  0.25
#define STILLNESS_SAMPLES    25
#define STILLNESS_CHECK_MS   3000
#define COOLDOWN_PERIOD      30000

static float   ei_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };
static int     ei_buffer_ix   = 0;
static bool    ei_buffer_full = false;

uint8_t mlFallScore = 0;
uint8_t mlConf = 0;
int     mlWinnerIdx = 0;

int   fallDetected = 0;
float impactMagnitude = 0;
unsigned long impactTime = 0;
int   stillnessCount = 0;
float peakOrient = 0;
unsigned long lastFallTime = 0;

bool          sosTriggered = false;
unsigned long sosPressTime = 0;
unsigned long lastSample = 0;
unsigned long lastBaroRead = 0;

volatile bool espNowReady = false;
volatile bool lastSendOk = false;
int sendCounter = 0;
uint8_t packetSeq = 0;

BMP280_Mini bmp;
Adafruit_NeoPixel pixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

void updateNeoPixel() {
    unsigned long t = millis();
    if (sosTriggered) {
        if ((t % 200) < 100) pixel.setPixelColor(0, 255, 0, 0);
        else pixel.setPixelColor(0, 0, 0, 0);
    } else if (fallDetected > 0) {
        if ((t % 300) < 150) pixel.setPixelColor(0, 255, 0, 255);
        else pixel.setPixelColor(0, 0, 0, 0);
    } else if (espNowReady && lastSendOk) {
        float brightness = (sin(t / 400.0) + 1.0) * 127.0;
        pixel.setPixelColor(0, 0, (int)brightness, 0);
    } else if (espNowReady) {
        pixel.setPixelColor(0, 0, 0, 60);
    } else {
        if ((t % 2000) < 100) pixel.setPixelColor(0, 60, 0, 0);
        else pixel.setPixelColor(0, 0, 0, 0);
    }
    pixel.show();
}

inline void ledOn()  { digitalWrite(PIN_LED, HIGH); }
inline void ledOff() { digitalWrite(PIN_LED, LOW); }

void blink(int times, int ms) {
    for (int i = 0; i < times; i++) {
        ledOn(); delay(ms);
        ledOff(); delay(ms);
    }
}

bool initMPU9250() {
    Wire.beginTransmission(MPU9250_ADDR);
    Wire.write(0x6B); Wire.write(0x00);
    if (Wire.endTransmission() != 0) return false;
    delay(10);

    Wire.beginTransmission(MPU9250_ADDR);
    Wire.write(0x1C); Wire.write(0x10); // ±8g
    Wire.endTransmission();

    Wire.beginTransmission(MPU9250_ADDR);
    Wire.write(0x1B); Wire.write(0x08); // ±500 dps
    Wire.endTransmission();

    Wire.beginTransmission(MPU9250_ADDR);
    Wire.write(0x1A); Wire.write(0x03); // DLPF 44Hz
    Wire.endTransmission();

    Wire.beginTransmission(MPU9250_ADDR);
    Wire.write(0x37); Wire.write(0x02); // Enable I2C bypass for AK8963
    Wire.endTransmission();
    delay(10);

    return true;
}

bool initMagnetometer() {
    Wire.beginTransmission(AK8963_ADDR);
    Wire.write(0x0A); Wire.write(0x16);
    return (Wire.endTransmission() == 0);
}

void readMPU9250() {
    Wire.beginTransmission(MPU9250_ADDR);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0) return;

    Wire.requestFrom((uint8_t)MPU9250_ADDR, (uint8_t)14);
    if (Wire.available() < 14) return;

    raw_ax = (Wire.read() << 8) | Wire.read();
    raw_ay = (Wire.read() << 8) | Wire.read();
    raw_az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read();
    raw_gx = (Wire.read() << 8) | Wire.read();
    raw_gy = (Wire.read() << 8) | Wire.read();
    raw_gz = (Wire.read() << 8) | Wire.read();

    ax = raw_ax / 4096.0f;
    ay = raw_ay / 4096.0f;
    az = raw_az / 4096.0f;
    gx = raw_gx / 65.5f;
    gy = raw_gy / 65.5f;
    gz = raw_gz / 65.5f;
    totalAccel = sqrt(ax * ax + ay * ay + az * az);
}

void readMagnetometer() {
    Wire.beginTransmission(AK8963_ADDR);
    Wire.write(0x03);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)AK8963_ADDR, (uint8_t)7);
    if (Wire.available() < 7) return;
    raw_mx = Wire.read() | (Wire.read() << 8);
    raw_my = Wire.read() | (Wire.read() << 8);
    raw_mz = Wire.read() | (Wire.read() << 8);
    Wire.read();
}

void runMLInference() {
    if (!ei_buffer_full) return;
    ei_buffer_full = false;

    signal_t signal;
    numpy::signal_from_buffer(ei_buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);

    ei_impulse_result_t result = { 0 };
    if (run_classifier(&signal, &result, false) != EI_IMPULSE_OK) return;

    float maxVal = 0;
    int maxIdx = 0;
    float fallScore = 0;
    for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        if (result.classification[i].value > maxVal) {
            maxVal = result.classification[i].value;
            maxIdx = i;
        }
        if (strstr(result.classification[i].label, "fall") != NULL &&
            strstr(result.classification[i].label, "alarm") == NULL) {
            fallScore = result.classification[i].value;
        }
    }

    mlFallScore = (uint8_t)(fallScore * 100);
    mlConf = (uint8_t)(maxVal * 100);
    mlWinnerIdx = maxIdx;

    unsigned long now = millis();
    if (now - lastFallTime < COOLDOWN_PERIOD && fallDetected == 0) return;

    if (mlFallScore >= ML_FALL_THRESHOLD && fallDetected == 0) {
        fallDetected = 1;
        impactTime = now;
        impactMagnitude = totalAccel;
        stillnessCount = 0;
        peakOrient = sqrt(gx*gx + gy*gy + gz*gz);
    }
}

void checkStillness() {
    unsigned long now = millis();
    if (fallDetected != 1) return;

    float gyroMag = sqrt(gx*gx + gy*gy + gz*gz);
    if (gyroMag > peakOrient) peakOrient = gyroMag;

    if (now - impactTime > STILLNESS_CHECK_MS) {
        fallDetected = 0;
        stillnessCount = 0;
        return;
    }

    float movement = abs(totalAccel - 1.0f);
    if (movement < STILLNESS_THRESHOLD) {
        stillnessCount++;
    } else {
        stillnessCount = max(0, stillnessCount - 3);
    }

    if (stillnessCount >= STILLNESS_SAMPLES) {
        fallDetected = 2;
        lastFallTime = now;
        blink(5, 50);
    }
}

void fallAutoReset() {
    if (fallDetected == 2 && millis() - lastFallTime > 5000) {
        fallDetected = 0;
        impactMagnitude = 0;
        peakOrient = 0;
        stillnessCount = 0;
    }
}

void onSendDone(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
    lastSendOk = (status == ESP_NOW_SEND_SUCCESS);
}

int currentChannel = 1;

int findBaseChannel() {
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
    int n = WiFi.scanNetworks(false, false, false, 1000);
    int found = -1;
    for (int i = 0; i < n; i++) {
        if (WiFi.SSID(i) == "FALLDETECT_BASE") {
            found = WiFi.channel(i);
        }
    }
    WiFi.scanDelete();
    return (found > 0) ? found : 1;
}

void setChannel(int ch) {
    currentChannel = ch;
    esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
}

void initESPNow() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    
    int ch = findBaseChannel();
    setChannel(ch);
    
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_max_tx_power(34); // 8.5dBm for LiPo battery safety
    
    if (esp_now_init() != ESP_OK) return;
    
    esp_now_register_send_cb(onSendDone);
    
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, broadcastMAC, 6);
    peer.channel = 0;
    peer.encrypt = false;
    
    if (esp_now_add_peer(&peer) == ESP_OK) {
        espNowReady = true;
    }
}

void checkChannelRecovery() {
    static unsigned long lastGoodSend = 0;
    if (lastSendOk) lastGoodSend = millis();
    
    if (millis() - lastGoodSend > 10000 && espNowReady) {
        esp_now_deinit();
        int ch = findBaseChannel();
        setChannel(ch);
        esp_now_init();
        esp_now_register_send_cb(onSendDone);
        
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, broadcastMAC, 6);
        peer.channel = 0;
        peer.encrypt = false;
        esp_now_add_peer(&peer);
        
        lastGoodSend = millis();
    }
}

void sendData() {
    if (!espNowReady) return;
    
    sendCounter++;
    if (sendCounter < 5 && fallDetected == 0) return;
    sendCounter = 0;
    
    SensorPacket pkt;
    pkt.magic = 0xFA;
    pkt.ax = raw_ax;
    pkt.ay = raw_ay;
    pkt.az = raw_az;
    pkt.gx = raw_gx;
    pkt.gy = raw_gy;
    pkt.gz = raw_gz;
    pkt.fall = fallDetected;
    pkt.impact = (uint8_t)constrain(impactMagnitude * 10, 0, 255);
    pkt.orient = (uint8_t)constrain(peakOrient / 2, 0, 255);
    pkt.mlFallScore = mlFallScore;
    pkt.mlConf = mlConf;
    pkt.mlFlags = sosTriggered ? 0x01 : 0x00;
    pkt.seq = packetSeq++;
    
    esp_now_send(broadcastMAC, (uint8_t*)&pkt, sizeof(pkt));
}

void setup() {
    Serial.begin(115200);
    
    pinMode(PIN_LED, OUTPUT);
    ledOff();
    pinMode(PIN_SOS, INPUT_PULLUP);

    pixel.begin();
    pixel.setBrightness(100);
    pixel.setPixelColor(0, 255, 255, 255);
    pixel.show();

    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);

    initMPU9250();
    initMagnetometer();
    bmp.begin(BMP280_ADDR);

    initESPNow();
}

void loop() {
    unsigned long now = millis();

    if (digitalRead(PIN_SOS) == LOW) {
        if (sosPressTime == 0) sosPressTime = now;
        if (now - sosPressTime > 3000 && !sosTriggered) {
            sosTriggered = true;
            fallDetected = 2;
            impactMagnitude = 6.0f;
        }
    } else {
        sosPressTime = 0;
        sosTriggered = false;
    }

    if (now - lastSample >= SAMPLE_RATE_MS) {
        lastSample = now;
        readMPU9250();

        ei_buffer[ei_buffer_ix + 0] = ax;
        ei_buffer[ei_buffer_ix + 1] = ay;
        ei_buffer[ei_buffer_ix + 2] = az;
        ei_buffer[ei_buffer_ix + 3] = gx;
        ei_buffer[ei_buffer_ix + 4] = gy;
        ei_buffer[ei_buffer_ix + 5] = gz;
        ei_buffer_ix += EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME;
        if (ei_buffer_ix >= EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
            ei_buffer_ix = 0;
            ei_buffer_full = true;
        }

        runMLInference();
        checkStillness();
        fallAutoReset();
        sendData();
    }

    if (now - lastBaroRead >= BARO_RATE_MS) {
        lastBaroRead = now;
        readMagnetometer();
        pressure_pa = bmp.readPressure();
    }

    checkChannelRecovery();
    updateNeoPixel();

    static unsigned long lastLedToggle = 0;
    if (sosTriggered) {
        if (now - lastLedToggle > 100) {
            digitalWrite(PIN_LED, !digitalRead(PIN_LED));
            lastLedToggle = now;
        }
    } else if (espNowReady && lastSendOk) {
        ledOn();
    } else {
        if (now - lastLedToggle > 1000) {
            digitalWrite(PIN_LED, !digitalRead(PIN_LED));
            lastLedToggle = now;
        }
    }
}
