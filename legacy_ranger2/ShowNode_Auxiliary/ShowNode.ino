/*
 * ShowNode - Test Code for Deneyap 1A v2 (ESP32-S3)
 * 
 * Components:
 *   - OLED Display (I2C): SDA=D10, SCL=D11
 *   - DHT11 Sensor: Signal=D1
 *   - Radar Module: RX=D3, TX=D2
 *   - Button: D9
 * 
 * Libraries Required:
 *   - Adafruit SSD1306
 *   - Adafruit GFX
 *   - DHT sensor library
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <HardwareSerial.h>

// ============== PIN DEFINITIONS (Deneyap D notation) ==============
#define PIN_SDA       D10
#define PIN_SCL       D11
#define PIN_DHT       D1
#define PIN_RADAR_RX  D3    // ESP RX <- Radar TX
#define PIN_RADAR_TX  D2    // ESP TX -> Radar RX
#define PIN_BUTTON    D9    // Button
#define PIN_LED       D8    // LED
#define PIN_BUZZER    D4    // Active Buzzer (PWM)
#define PIN_MIC       A0    // Microphone Analog (unused)

// Buzzer PWM channel
#define BUZZER_CH     0

// ============== OLED CONFIG ==============
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_ADDR     0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ============== DHT CONFIG ==============
#define DHT_TYPE DHT11
DHT dht(PIN_DHT, DHT_TYPE);

// ============== RADAR SERIAL ==============
HardwareSerial radarSerial(1);  // UART1

// ============== DATA VARIABLES ==============
float temperature = 0;
float humidity = 0;
bool radarPresence = false;
float radarDistance = 0;
String radarStatus = "Initializing...";
bool buttonPressed = false;
int micLevel = 0;           // Microphone reading (0-4095)

// ============== TIMING ==============
unsigned long lastDHTRead = 0;
unsigned long lastDisplayUpdate = 0;
#define DHT_INTERVAL 2000     // Read DHT every 2 seconds
#define DISPLAY_INTERVAL 100  // Update display 10 times/sec

// ============== RADAR BUFFER ==============
char radarBuffer[64];
int radarBufIdx = 0;

void setup() {
    Serial.begin(115200);
    Serial.println("\n=== ShowNode Test ===");
    Serial.println("Deneyap 1A v2 (ESP32-S3)");
    
    // Initialize I2C with custom pins
    Wire.begin(PIN_SDA, PIN_SCL);
    Serial.printf("I2C: SDA=%d, SCL=%d\n", PIN_SDA, PIN_SCL);
    
    // Initialize OLED
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("OLED: FAILED!");
    } else {
        Serial.println("OLED: OK");
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println("ShowNode");
        display.println("Initializing...");
        display.display();
    }
    
    // Initialize DHT
    dht.begin();
    Serial.printf("DHT11: Pin %d\n", PIN_DHT);
    
    // Initialize Radar Serial
    radarSerial.begin(115200, SERIAL_8N1, PIN_RADAR_RX, PIN_RADAR_TX);
    Serial.printf("Radar: RX=%d, TX=%d\n", PIN_RADAR_RX, PIN_RADAR_TX);
    
    // Initialize Button
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    Serial.printf("Button: Pin %d\n", PIN_BUTTON);
    
    // Initialize LED
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);
    Serial.printf("LED: Pin %d\n", PIN_LED);
    
    // Initialize Buzzer (PWM for tone generation)
    ledcSetup(BUZZER_CH, 2000, 8);       // Channel 0, 2kHz, 8-bit
    ledcAttachPin(PIN_BUZZER, BUZZER_CH);
    ledcWriteTone(BUZZER_CH, 0);          // Start silent
    Serial.printf("Buzzer (PWM): Pin %d\n", PIN_BUZZER);
    
    
    delay(1000);
    Serial.println("=== Ready ===\n");
}

void loop() {
    unsigned long now = millis();
    
    // Read button (active HIGH - pressed = HIGH)
    static bool lastButtonState = false;
    buttonPressed = (digitalRead(PIN_BUTTON) == HIGH);
    
    // LED control: On when button pressed
    digitalWrite(PIN_LED, buttonPressed ? HIGH : LOW);
    
    // Play tune on button press (edge detection)
    if (buttonPressed && !lastButtonState) {
        playTune();
    }
    lastButtonState = buttonPressed;
    
    // Buzzer: Quick beep on radar presence change
    static bool lastRadarState = false;
    if (radarPresence && !lastRadarState) {
        ledcWriteTone(BUZZER_CH, 1000);  // 1kHz beep
        delay(50);
        ledcWriteTone(BUZZER_CH, 0);
    }
    lastRadarState = radarPresence;
    
    // Read DHT sensor periodically
    if (now - lastDHTRead >= DHT_INTERVAL) {
        lastDHTRead = now;
        readDHT();
    }
    
    // Process Radar data continuously
    readRadar();
    
    // Update display periodically
    if (now - lastDisplayUpdate >= DISPLAY_INTERVAL) {
        lastDisplayUpdate = now;
        updateDisplay();
    }
}

// ============== BUZZER TUNE (PWM) ==============
void playTune() {
    // 🔥 HARD DROP - Bass-heavy aggressive pattern 🔥
    // Lower frequencies hit harder, fast staccato rhythm
    
    // Intro build-up
    for (int f = 200; f < 800; f += 100) {
        ledcWriteTone(BUZZER_CH, f);
        delay(30);
    }
    ledcWriteTone(BUZZER_CH, 0);
    delay(50);
    
    // DROP! Fast punchy hits
    int drops[] = {150, 150, 200, 150, 100, 150, 200, 300};
    int times[] = {60, 40, 60, 40, 60, 40, 80, 150};
    
    for (int i = 0; i < 8; i++) {
        ledcWriteTone(BUZZER_CH, drops[i]);
        delay(times[i]);
        ledcWriteTone(BUZZER_CH, 0);
        delay(20);
    }
    
    // Final bass hit
    ledcWriteTone(BUZZER_CH, 100);
    delay(200);
    ledcWriteTone(BUZZER_CH, 0);
}

// ============== DHT READING ==============
void readDHT() {
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    
    if (!isnan(h) && !isnan(t)) {
        humidity = h;
        temperature = t;
        Serial.printf("DHT: %.1f°C, %.1f%%\n", temperature, humidity);
    } else {
        Serial.println("DHT: Read error");
    }
}

// ============== RADAR READING ==============
void readRadar() {
    while (radarSerial.available()) {
        char c = radarSerial.read();
        
        if (c == '\n' || radarBufIdx >= 62) {
            radarBuffer[radarBufIdx] = 0;
            if (radarBufIdx > 2) {
                parseRadarData(radarBuffer);
            }
            radarBufIdx = 0;
        } else if (c >= 32) {
            radarBuffer[radarBufIdx++] = c;
        }
    }
}

void parseRadarData(const char* data) {
    String s = String(data);
    s.trim();
    
    if (s.startsWith("mov") || s.startsWith("occ")) {
        radarPresence = true;
        int idx = s.indexOf("dis=");
        if (idx > 0) {
            radarDistance = s.substring(idx + 4).toFloat();
        }
        radarStatus = "Detected";
        Serial.printf("Radar: %s, Distance=%.2fm\n", s.c_str(), radarDistance);
    } else if (s.indexOf("empty") >= 0) {
        radarPresence = false;
        radarDistance = 0;
        radarStatus = "Empty";
        Serial.println("Radar: Empty");
    }
}

// ============== DISPLAY UPDATE ==============
void updateDisplay() {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    
    // ===== TOP BAR: Status Icons (row 0-9) =====
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("SHOWNODE");
    
    // Status indicators on right
    display.setCursor(75, 0);
    display.print(buttonPressed ? "[BTN]" : "     ");
    display.setCursor(110, 0);
    display.print(radarPresence ? "*" : " ");
    
    // Separator line
    display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
    
    // ===== LEFT COLUMN: Environment (x: 0-63) =====
    display.setCursor(0, 14);
    display.print("TEMP");
    display.setTextSize(2);
    display.setCursor(0, 24);
    display.printf("%.0f", temperature);
    display.setTextSize(1);
    display.print("C");
    
    display.setCursor(0, 44);
    display.print("HUM ");
    display.printf("%.0f%%", humidity);
    
    // Vertical separator
    display.drawLine(64, 11, 64, 55, SSD1306_WHITE);
    
    // ===== RIGHT COLUMN: Radar (x: 66-127) =====
    display.setCursor(68, 14);
    display.print("RADAR");
    display.setTextSize(2);
    display.setCursor(68, 26);
    if (radarPresence) {
        display.printf("%.1f", radarDistance);
        display.setTextSize(1);
        display.print("m");
    } else {
        display.print("--");
    }
    display.setTextSize(1);
    
    // Status text
    display.setCursor(68, 46);
    display.print(radarPresence ? "DETECTED" : "EMPTY");
    
    // ===== BOTTOM BAR: Uptime =====
    display.drawLine(0, 55, 127, 55, SSD1306_WHITE);
    display.setCursor(0, 58);
    unsigned long upSec = millis() / 1000;
    display.printf("UP: %02lu:%02lu:%02lu", upSec/3600, (upSec/60)%60, upSec%60);
    
    display.display();
}
