/**
 * RANGER — Edge Impulse Data Collector (CSV Upload Version)
 * Board: Seeed XIAO nRF52840 + MPU6050
 * 
 * PURPOSE:
 *   Streams raw MPU6050 data over Serial at 50Hz as CSV.
 *   Data is captured to a .csv file on your computer, then
 *   uploaded directly to Edge Impulse via the website.
 *
 * USAGE:
 *   1. Flash this sketch
 *   2. Open a terminal and run the capture script:
 *        cd /Users/gorkem/Ranger/EI_DataCollector
 *        bash capture.sh idle        (for idle recording)
 *        bash capture.sh walking     (for walking recording)
 *        bash capture.sh fall        (for fall recording)
 *        bash capture.sh false_alarm (for false alarm recording)
 *   3. Press Ctrl+C to stop recording
 *   4. Upload the .csv files at studio.edgeimpulse.com
 *      -> Data Acquisition -> Upload Data -> Choose .csv files
 *
 * OUTPUT FORMAT (CSV over Serial @ 115200 baud):
 *   timestamp,accX,accY,accZ,gyrX,gyrY,gyrZ
 *
 * PROTOCOL:
 *   Send 's' over Serial to START recording (header is printed)
 *   Send 'x' over Serial to STOP recording
 *   This prevents garbage data from corrupting the CSV file.
 */

#include <Arduino.h>
#include <Wire.h>

// ============== CONFIG ==============
#define MPU6050_ADDR    0x68
#define SAMPLE_RATE_HZ  50
#define SAMPLE_PERIOD_US (1000000 / SAMPLE_RATE_HZ)

// Scale factors: Accel ±8g, Gyro ±500°/s
#define ACCEL_SCALE  4096.0f
#define GYRO_SCALE   65.5f

// ============== STATE ==============
int16_t raw_ax, raw_ay, raw_az;
int16_t raw_gx, raw_gy, raw_gz;
bool recording = false;
unsigned long recordStartTime = 0;

// ============== MPU6050 DRIVER ==============
bool initMPU6050() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B); Wire.write(0x00);  // Wake up
    if (Wire.endTransmission() != 0) return false;
    delay(10);

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1C); Wire.write(0x10);  // Accel ±8g
    Wire.endTransmission();

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1B); Wire.write(0x08);  // Gyro ±500°/s
    Wire.endTransmission();

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1A); Wire.write(0x03);  // DLPF ~44Hz
    Wire.endTransmission();

    return true;
}

void readMPU6050() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0) return;

    Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14);
    if (Wire.available() < 14) return;

    raw_ax = (Wire.read() << 8) | Wire.read();
    raw_ay = (Wire.read() << 8) | Wire.read();
    raw_az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read();
    raw_gx = (Wire.read() << 8) | Wire.read();
    raw_gy = (Wire.read() << 8) | Wire.read();
    raw_gz = (Wire.read() << 8) | Wire.read();
}

// ============== SETUP ==============
void setup() {
    Serial.begin(115200);
    delay(2000);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);  // Off

    Wire.begin();
    Wire.setClock(400000);

    if (initMPU6050()) {
        for (int i = 0; i < 3; i++) {
            digitalWrite(LED_BUILTIN, LOW); delay(100);
            digitalWrite(LED_BUILTIN, HIGH); delay(100);
        }
        Serial.println("READY: Send 's' to start recording, 'x' to stop.");
    } else {
        Serial.println("ERROR: MPU6050 not found!");
        while (true) {
            digitalWrite(LED_BUILTIN, LOW); delay(50);
            digitalWrite(LED_BUILTIN, HIGH); delay(50);
        }
    }
}

// ============== LOOP ==============
void loop() {
    // Check for serial commands
    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 's' && !recording) {
            recording = true;
            recordStartTime = millis();
            digitalWrite(LED_BUILTIN, LOW);  // LED on = recording
            // Print CSV header
            Serial.println("timestamp,accX,accY,accZ,gyrX,gyrY,gyrZ");
        } else if (cmd == 'x' && recording) {
            recording = false;
            digitalWrite(LED_BUILTIN, HIGH);  // LED off
            Serial.println("STOPPED");
        }
    }

    if (!recording) return;

    static unsigned long lastSampleTime = 0;
    unsigned long now = micros();

    if (now - lastSampleTime >= SAMPLE_PERIOD_US) {
        lastSampleTime = now;
        readMPU6050();

        // Timestamp in milliseconds since recording started
        unsigned long ts = millis() - recordStartTime;

        float accX = raw_ax / ACCEL_SCALE;
        float accY = raw_ay / ACCEL_SCALE;
        float accZ = raw_az / ACCEL_SCALE;
        float gyrX = raw_gx / GYRO_SCALE;
        float gyrY = raw_gy / GYRO_SCALE;
        float gyrZ = raw_gz / GYRO_SCALE;

        // Edge Impulse CSV format
        Serial.print(ts); Serial.print(',');
        Serial.print(accX, 4); Serial.print(',');
        Serial.print(accY, 4); Serial.print(',');
        Serial.print(accZ, 4); Serial.print(',');
        Serial.print(gyrX, 4); Serial.print(',');
        Serial.print(gyrY, 4); Serial.print(',');
        Serial.println(gyrZ, 4);
    }
}
