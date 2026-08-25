/**
 * RANGER — Next-Gen Wearable Hardware Validation (Seeed XIAO nRF54L15)
 * 
 * Hardware Target: Nordic Semiconductor nRF54L15 (Arm Cortex-M33 @ 128MHz, BLE 5.4)
 * 
 * Verification Scope:
 *   1. System Clock & High-Speed USB Serial CDC (115200 baud).
 *   2. Onboard Status LED GPIO output toggling.
 *   3. Fast-Mode I2C Bus Scan (400 kHz) for MPU9250 / MPU6050, BMP280 / BMP388, and MAX30102.
 */

#include <Arduino.h>
#include <Wire.h>

#ifndef LED_BUILTIN
#define LED_BUILTIN 26
#endif

#define I2C_CLOCK_SPEED 400000

void scanI2CBus() {
    Serial.println(F("[I2C] Scanning bus at 400 kHz..."));
    uint8_t devicesFound = 0;

    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        uint8_t error = Wire.endTransmission();

        if (error == 0) {
            Serial.printf("  -> Device detected at 0x%02X: ", addr);
            switch (addr) {
                case 0x68:
                    Serial.println(F("MPU6050 / MPU9250 6/9-DoF IMU"));
                    break;
                case 0x0C:
                    Serial.println(F("AK8963 Magnetometer (MPU9250 auxiliary)"));
                    break;
                case 0x76:
                case 0x77:
                    Serial.println(F("BMP280 / BMP388 Barometric Altimeter"));
                    break;
                case 0x57:
                    Serial.println(F("MAX30102 / MAX30100 Biometric PPG Sensor"));
                    break;
                default:
                    Serial.println(F("Generic I2C Peripheral"));
                    break;
            }
            devicesFound++;
        }
    }

    if (devicesFound == 0) {
        Serial.println(F("[I2C] Warning: No I2C devices acknowledged. Check pull-ups & wiring."));
    } else {
        Serial.printf("[I2C] Scan complete: %u device(s) online.\n", devicesFound);
    }
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.begin(115200);
    uint32_t startMs = millis();
    while (!Serial && (millis() - startMs < 3000)) {
        delay(10);
    }

    Serial.println();
    Serial.println(F("=================================================="));
    Serial.println(F("  RANGER — XIAO nRF54L15 Hardware Diagnostics    "));
    Serial.println(F("  Core: Arm Cortex-M33 | Protocol: BLE 5.4 Ready  "));
    Serial.println(F("=================================================="));

    Wire.begin();
    Wire.setClock(I2C_CLOCK_SPEED);

    scanI2CBus();
    Serial.println(F("[SYS] Diagnostics complete. Entering heartbeat loop.\n"));
}

void loop() {
    // Heartbeat LED Blink (1 Hz)
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
}
