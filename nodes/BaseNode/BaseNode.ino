/**
 * RANGER — Base Station Node (Seeed XIAO nRF52840)
 * 
 * Functions:
 *   1. Operates as BLE Central; scans and connects to "RANGER-WEAR".
 *   2. Subscribes to IMU, Environmental, and ML classification notification streams.
 *   3. Serializes decoded BLE payloads into high-throughput CSV over UART (115200 baud) to the Fusion Node.
 *   4. Implements debounced link-loss heartbeat detection.
 */

#include <Arduino.h>
#include <bluefruit.h>

#define VERSION "4.0-BASE-BRIDGE-ML"

#define HEARTBEAT_TIMEOUT_MS  10000
#define W_LOST_DEBOUNCE_MS     3000   // Filter transient connection blips

// BLE UUID Definitions
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

volatile bool bleConnected = false;
volatile unsigned long lastDataReceived = 0;
bool wearableLost = false;

volatile uint32_t packetsReceived = 0;
unsigned long lastStatusPrint = 0;

inline void ledOn()  { digitalWrite(PIN_LED, LOW); }
inline void ledOff() { digitalWrite(PIN_LED, HIGH); }

inline void uartPrint(const char* str) {
    Serial1.print(str);
}
inline void uartPrintln(const char* str) {
    Serial1.println(str);
}

void imuNotifyCallback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len);
void envNotifyCallback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len);
void mlNotifyCallback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len);

void scanCallback(ble_gap_evt_adv_report_t* report) {
    if (Bluefruit.Scanner.checkReportForService(report, rangerService)) {
        Bluefruit.Central.connect(report);
    } else {
        Bluefruit.Scanner.resume();
    }
}

void connectCallback(uint16_t conn_handle) {
    // 50ms connection interval (40 * 1.25ms) for stable 20Hz throughput
    Bluefruit.Connection(conn_handle)->requestConnectionParameter(40);
    
    if (!rangerService.discover(conn_handle)) {
        Bluefruit.disconnect(conn_handle);
        return;
    }
    
    if (!imuChar.discover() || !envChar.discover()) {
        Bluefruit.disconnect(conn_handle);
        return;
    }
    
    mlChar.discover();
    deviceInfoChar.discover();
    
    imuChar.setNotifyCallback(imuNotifyCallback);
    imuChar.enableNotify();
    
    envChar.setNotifyCallback(envNotifyCallback);
    envChar.enableNotify();

    if (mlChar.discovered()) {
        mlChar.setNotifyCallback(mlNotifyCallback);
        mlChar.enableNotify();
    }
    
    bleConnected = true;
    wearableLost = false;
    lastDataReceived = millis();
    uartPrintln("STAT,BASE_LINK_OK");
}

void disconnectCallback(uint16_t conn_handle, uint8_t reason) {
    bleConnected = false;
    uartPrintln("STAT,LINK_LOST");
}

volatile bool pendingImu = false;
volatile bool pendingEnv = false;
volatile bool pendingMl = false;
char imuJson[100];
char envJson[100];
char mlJson[80];

void imuNotifyCallback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len) {
    if (len < 20) return;
    lastDataReceived = millis();
    packetsReceived++;
    
    int16_t ax = (data[0] << 8)  | data[1];
    int16_t ay = (data[2] << 8)  | data[3];
    int16_t az = (data[4] << 8)  | data[5];
    int16_t gx = (data[6] << 8)  | data[7];
    int16_t gy = (data[8] << 8)  | data[9];
    int16_t gz = (data[10] << 8) | data[11];
    uint16_t seq = (data[18] << 8) | data[19];
    
    snprintf(imuJson, sizeof(imuJson),
        "I,%d,%d,%d,%d,%d,%d,%d\n",
        ax, ay, az, gx, gy, gz, seq);
    
    pendingImu = true;
}

void envNotifyCallback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len) {
    if (len < 15) return;
    lastDataReceived = millis();
    packetsReceived++;
    
    int32_t press = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
    uint32_t ir   = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7];
    uint32_t red  = (data[8] << 24) | (data[9] << 16) | (data[10] << 8) | data[11];
    
    uint8_t sos_bat = data[12];
    uint8_t bat = sos_bat & 0x7F;
    bool sos = (sos_bat & 0x80) != 0;
    uint16_t seq = (data[13] << 8) | data[14];
    
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
    if (len < 8) return;
    lastDataReceived = millis();

    snprintf(mlJson, sizeof(mlJson),
        "M,%d,%d,%d,%d,%d,%d,%d\n",
        data[0], data[1], data[2], data[3],
        data[4], data[5], data[6]);

    pendingMl = true;
}

void setup() {
    Serial.begin(115200);
    
    pinMode(PIN_LED, OUTPUT);
    ledOff();
    
    // Hardware Serial1 to Fusion Node (Pins D6/D7)
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
    
    // Heartbeat monitoring with debounce filtering
    unsigned long lst = lastDataReceived;
    static unsigned long wLostSince = 0;
    if (bleConnected && (now - lst > HEARTBEAT_TIMEOUT_MS)) {
        if (!wearableLost) {
            if (wLostSince == 0) {
                wLostSince = now;
            } else if (now - wLostSince > W_LOST_DEBOUNCE_MS) {
                wearableLost = true;
                wLostSince = 0;
                uartPrintln("STAT,W_LOST");
            }
        }
    } else {
        wLostSince = 0;
    }
    
    // Status LED logic
    static unsigned long lastLedToggle = 0;
    if (wearableLost) {
        if (now - lastLedToggle > 100) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); lastLedToggle = now; }
    } else if (bleConnected) {
        ledOn();
    } else {
        if (now - lastLedToggle > 500) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); lastLedToggle = now; }
    }
}
