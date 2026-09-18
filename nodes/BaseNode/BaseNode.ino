/**
 * RANGER v4.1 — Base Station Node (nRF52840)
 *
 * ROLE: Pure BLE to UART Bridge for the RP2040 Fusion Node.
 * - Connects to RANGER-WEAR over BLE.
 * - Receives IMU and ENV characteristics.
 * - Forwards data directly over Hardware Serial1 (D6/D7) as JSON.
 * - Stripped of all logic, OLED, and sensors.
 *
 * UART TX: D6 (to RP2040 RX)
 * UART RX: D7 (from RP2040 TX)
 */

#include <Arduino.h>
#include <bluefruit.h>

#define VERSION "4.1-BASE-RIGID-BRIDGE"

#define HEARTBEAT_TIMEOUT_MS  10000
#define W_LOST_DEBOUNCE_MS     3000   // Suppress rapid W_LOST/W_OK chatter

// ============== BLE UUIDs ==============
#define RANGER_SERVICE_UUID       "833d1814-9988-4e31-8db2-2c67699cd1c1"
#define SENSOR_IMU_CHAR_UUID      "833d2a01-9988-4e31-8db2-2c67699cd1c1"
#define SENSOR_ENV_CHAR_UUID      "833d2a02-9988-4e31-8db2-2c67699cd1c1"
#define ML_RESULT_CHAR_UUID       "833d2a04-9988-4e31-8db2-2c67699cd1c1"
#define DEVICE_INFO_CHAR_UUID     "833d2a03-9988-4e31-8db2-2c67699cd1c1"

BLEClientService        rangerService(RANGER_SERVICE_UUID);
BLEClientCharacteristic imuChar(SENSOR_IMU_CHAR_UUID);
BLEClientCharacteristic envChar(SENSOR_ENV_CHAR_UUID);
BLEClientCharacteristic mlChar(ML_RESULT_CHAR_UUID);
BLEClientCharacteristic deviceInfoChar(DEVICE_INFO_CHAR_UUID);

// ============== GLOBALS ==============
volatile bool bleConnected = false;
volatile unsigned long lastDataReceived = 0;
volatile bool wearableLost = false;

volatile uint32_t packetsReceived = 0;
volatile uint32_t malformedPackets = 0;
unsigned long lastStatusPrint = 0;

void ledOn()  { digitalWrite(PIN_LED, LOW); }
void ledOff() { digitalWrite(PIN_LED, HIGH); }

void uartPrint(const char* str) {
    Serial1.print(str);
}
void uartPrintln(const char* str) {
    Serial1.println(str);
}

uint16_t readBe16(const uint8_t* data) {
    return (static_cast<uint16_t>(data[0]) << 8) |
           static_cast<uint16_t>(data[1]);
}

uint32_t readBe32(const uint8_t* data) {
    return (static_cast<uint32_t>(data[0]) << 24) |
           (static_cast<uint32_t>(data[1]) << 16) |
           (static_cast<uint32_t>(data[2]) << 8) |
           static_cast<uint32_t>(data[3]);
}

// ============== BLE CALLBACKS ==============
void scanCallback(ble_gap_evt_adv_report_t* report) {
    if (Bluefruit.Scanner.checkReportForService(report, rangerService)) {
        Serial.println("Found RANGER-WEAR! Connecting...");
        Bluefruit.Central.connect(report);
    } else {
        Bluefruit.Scanner.resume();
    }
}

void connectCallback(uint16_t conn_handle) {
    Serial.println("BLE Connected! Discovering services...");

    // Request stable connection interval for 20Hz streaming (50ms target)
    // 40 = 50ms. Using 30ms (24) caused W_LOST/W_OK flicker.
    Bluefruit.Connection(conn_handle)->requestConnectionParameter(40);

    if (!rangerService.discover(conn_handle)) {
        Serial.println("  FAIL: RANGER service not found");
        Bluefruit.disconnect(conn_handle);
        return;
    }

    if (!imuChar.discover()) {
        Serial.println("  FAIL: IMU char not found");
        Bluefruit.disconnect(conn_handle);
        return;
    }

    if (!envChar.discover()) {
        Serial.println("  FAIL: ENV char not found");
        Bluefruit.disconnect(conn_handle);
        return;
    }

    if (!mlChar.discover()) {
        Serial.println("  WARN: ML char not found (old firmware?)");
    }

    deviceInfoChar.discover();

    // Enable notifications
    imuChar.setNotifyCallback(imuNotifyCallback);
    const bool imuNotifyOk = imuChar.enableNotify();

    envChar.setNotifyCallback(envNotifyCallback);
    const bool envNotifyOk = envChar.enableNotify();

    if (!imuNotifyOk || !envNotifyOk) {
        Serial.println("  FAIL: mandatory notification subscription failed");
        Bluefruit.disconnect(conn_handle);
        return;
    }

    if (mlChar.discovered()) {
        mlChar.setNotifyCallback(mlNotifyCallback);
        mlChar.enableNotify();
        Serial.println("ML notifications enabled.");
    }

    bleConnected = true;
    wearableLost = false;
    lastDataReceived = millis();
    Serial.println("Wearable link established. Bridge ready.\n");
    uartPrintln("STAT,BASE_LINK_OK");
}

void disconnectCallback(uint16_t conn_handle, uint8_t reason) {
    bleConnected = false;
    Serial.printf("Disconnected (reason: 0x%02X). Scanning...\n", reason);
    uartPrintln("STAT,LINK_LOST");
}

volatile bool pendingImu = false;
volatile bool pendingEnv = false;
volatile bool pendingMl = false;
char imuJson[100];
char envJson[100];
char mlJson[80];

void imuNotifyCallback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len) {
    if (len != 20) { malformedPackets++; return; }
    lastDataReceived = millis();
    packetsReceived++;

    // Decode [ax:2][ay:2][az:2][gx:2][gy:2][gz:2][mx:2][my:2][mz:2][seq:2]
    int16_t ax = static_cast<int16_t>(readBe16(data));
    int16_t ay = static_cast<int16_t>(readBe16(data + 2));
    int16_t az = static_cast<int16_t>(readBe16(data + 4));
    int16_t gx = static_cast<int16_t>(readBe16(data + 6));
    int16_t gy = static_cast<int16_t>(readBe16(data + 8));
    int16_t gz = static_cast<int16_t>(readBe16(data + 10));
    uint16_t seq = readBe16(data + 18);

    // Compact CSV format
    snprintf(imuJson, sizeof(imuJson),
        "I,%d,%d,%d,%d,%d,%d,%d\n",
        ax, ay, az, gx, gy, gz, seq);

    pendingImu = true;
    if (wearableLost) {
        wearableLost = false;
        uartPrintln("STAT,W_OK");
    }
}

void envNotifyCallback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len) {
    if (len != 15) { malformedPackets++; return; }
    lastDataReceived = millis();
    packetsReceived++;

    // Decode [press:4][ir:4][red:4][sos_bat:1][seq:2]
    int32_t press = static_cast<int32_t>(readBe32(data));
    uint32_t ir   = readBe32(data + 4);
    uint32_t red  = readBe32(data + 8);

    uint8_t sos_bat = data[12];
    uint8_t bat = sos_bat & 0x7F;     // Bits 0-6: battery
    bool sos = (sos_bat & 0x80) != 0; // Bit 7: SOS

    uint16_t seq = readBe16(data + 13);

    snprintf(envJson, sizeof(envJson),
        "E,%ld,%lu,%lu,%d,%d,%u\n",
        press, ir, red, bat, sos, seq);

    pendingEnv = true;

    if (wearableLost) {
        wearableLost = false;
        uartPrintln("STAT,W_OK");
    }
}

void mlNotifyCallback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len) {
    if (len != 8) { malformedPackets++; return; }
    lastDataReceived = millis();

    // Decode: [fall:1][false_alarm:1][idle:1][walk:1][winnerIdx:1][confidence:1][flags:1][seq:1]
    snprintf(mlJson, sizeof(mlJson),
        "M,%d,%d,%d,%d,%d,%d,%d\n",
        data[0], data[1], data[2], data[3],
        data[4], data[5], data[6]);

    pendingMl = true;
    if (wearableLost) {
        wearableLost = false;
        uartPrintln("STAT,W_OK");
    }
}

// ============== SETUP ==============
void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n=== RANGER BASE BRIDGE " VERSION " ===");

    pinMode(PIN_LED, OUTPUT);
    ledOff();

    // UART to FusionNode (Hardware Serial1 on D6/D7)
    Serial1.begin(115200);
    uartPrintln("STAT,BASE_BOOT");

    Bluefruit.begin(0, 1);
    Bluefruit.setName("RANGER-BASE");
    Bluefruit.setTxPower(4);

    rangerService.begin();
    imuChar.begin();
    envChar.begin();
    mlChar.begin();
    deviceInfoChar.begin();

    Bluefruit.Central.setConnectCallback(connectCallback);
    Bluefruit.Central.setDisconnectCallback(disconnectCallback);

    Bluefruit.Scanner.setRxCallback(scanCallback);
    Bluefruit.Scanner.restartOnDisconnect(true);
    Bluefruit.Scanner.setInterval(160, 80);
    Bluefruit.Scanner.filterUuid(rangerService.uuid);
    Bluefruit.Scanner.useActiveScan(false);
    Bluefruit.Scanner.start(0);
}

// ============== LOOP ==============
void loop() {
    unsigned long now = millis();

    if (pendingImu) {
        pendingImu = false;
        uartPrint(imuJson);
    }

    if (pendingEnv) {
        pendingEnv = false;
        uartPrint(envJson);
    }

    if (pendingMl) {
        pendingMl = false;
        uartPrint(mlJson);
    }

    // Heartbeat check with debounce (prevents rapid W_LOST/W_OK chatter)
    unsigned long lst = lastDataReceived;
    static unsigned long wLostSince = 0;
    if (bleConnected && (now - lst > HEARTBEAT_TIMEOUT_MS)) {
        if (!wearableLost) {
            // First detection — start debounce timer
            if (wLostSince == 0) {
                wLostSince = now;
            } else if (now - wLostSince > W_LOST_DEBOUNCE_MS) {
                // Confirmed loss — not just a hiccup
                wearableLost = true;
                wLostSince = 0;
                Serial.println("WEARABLE TIMEOUT (confirmed)");
                uartPrintln("STAT,W_LOST");
            }
        }
    } else {
        wLostSince = 0;  // Reset debounce if data arrives
    }

    if (now - lastStatusPrint > 2000) {
        lastStatusPrint = now;
        if (bleConnected && !wearableLost) {
            Serial.printf("Active. Pkts recv: %lu malformed: %lu\n",
                          packetsReceived, malformedPackets);
        } else if (bleConnected && wearableLost) {
            Serial.println("Wearable silent...");
        } else {
            Serial.println("Scanning...");
        }
    }

    // LED Status
    static unsigned long lastLedToggle = 0;
    if (wearableLost) {
        if (now - lastLedToggle > 100) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); lastLedToggle = now; }
    } else if (bleConnected) {
        ledOn();
    } else {
        if (now - lastLedToggle > 500) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); lastLedToggle = now; }
    }
}
