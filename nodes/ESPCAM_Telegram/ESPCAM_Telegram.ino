/**
 * RANGER — Emergency Visual Verification Node (ESP32-CAM)
 * 
 * Flow:
 *   1. Hardware Power-On (Optoisolated relay closed by Fusion Node on confirmed fall).
 *   2. Connects to local 2.4GHz Wi-Fi network.
 *   3. Initializes OV2640/GC2145 image sensor in QVGA (320x240) mode.
 *   4. Captures 3 visual snapshots spaced 2 seconds apart.
 *   5. Dispatches photos directly to Telegram bot chat via HTTPS multipart/form-data.
 *   6. Enters deep sleep until power is cut by the Fusion Node timeout.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "esp_camera.h"

// Configuration
#define WIFI_SSID           "YOUR_WIFI_SSID"
#define WIFI_PASS           "YOUR_WIFI_PASSWORD"
#define TELEGRAM_BOT_TOKEN  "YOUR_TELEGRAM_BOT_TOKEN"
#define TELEGRAM_CHAT_ID    "YOUR_TELEGRAM_CHAT_ID"

const int NUM_SHOTS = 3;
const int SHOT_DELAY_MS = 2000;

// Camera Model: AI-THINKER ESP32-CAM Pin Mapping
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

bool initCamera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;
    config.pin_d0       = Y2_GPIO_NUM;
    config.pin_d1       = Y3_GPIO_NUM;
    config.pin_d2       = Y4_GPIO_NUM;
    config.pin_d3       = Y5_GPIO_NUM;
    config.pin_d4       = Y6_GPIO_NUM;
    config.pin_d5       = Y7_GPIO_NUM;
    config.pin_d6       = Y8_GPIO_NUM;
    config.pin_d7       = Y9_GPIO_NUM;
    config.pin_xclk     = XCLK_GPIO_NUM;
    config.pin_pclk     = PCLK_GPIO_NUM;
    config.pin_vsync    = VSYNC_GPIO_NUM;
    config.pin_href     = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn     = PWDN_GPIO_NUM;
    config.pin_reset    = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    
    // Capture in RGB565 and compress to JPEG in software to maintain compatibility across sensor variants
    config.pixel_format = PIXFORMAT_RGB565;
    config.frame_size   = FRAMESIZE_QVGA;
    config.fb_count     = 1;

    return (esp_camera_init(&config) == ESP_OK);
}

String sendPhotoTelegram() {
    const char* myDomain = "api.telegram.org";
    String getAll = "", getBody = "";

    camera_fb_t * fb = esp_camera_fb_get();  
    if (!fb) return "Capture failed";

    uint8_t *jpg_buf = NULL;
    size_t jpg_len = 0;
    bool converted = fmt2jpg(fb->buf, fb->len, fb->width, fb->height, PIXFORMAT_RGB565, 80, &jpg_buf, &jpg_len);
    
    if (!converted) {
        esp_camera_fb_return(fb);
        return "JPEG conversion failed";
    }

    WiFiClientSecure client;
    client.setInsecure(); // Telegram API HTTPS certificate verification bypass
    
    if (client.connect(myDomain, 443)) {
        String head = "--RangerBoundary\r\nContent-Disposition: form-data; name=\"chat_id\"; \r\n\r\n" 
                    + String(TELEGRAM_CHAT_ID) 
                    + "\r\n--RangerBoundary\r\nContent-Disposition: form-data; name=\"photo\"; filename=\"fall.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
        String tail = "\r\n--RangerBoundary--\r\n";

        uint32_t imageLen = jpg_len;
        uint32_t extraLen = head.length() + tail.length();
        uint32_t totalLen = imageLen + extraLen;
    
        client.println("POST /bot" + String(TELEGRAM_BOT_TOKEN) + "/sendPhoto HTTP/1.1");
        client.println("Host: " + String(myDomain));
        client.println("Connection: close");
        client.println("Content-Length: " + String(totalLen));
        client.println("Content-Type: multipart/form-data; boundary=RangerBoundary");
        client.println();
        client.print(head);
    
        // Write payload in 1KB chunks to adhere to TLS record fragmentation limits
        uint8_t *fbBuf = jpg_buf;
        size_t fbLen = jpg_len;
        for (size_t n = 0; n < fbLen; n += 1024) {
            if (n + 1024 < fbLen) {
                client.write(fbBuf, 1024);
                fbBuf += 1024;
            } else if (fbLen % 1024 > 0) {
                size_t remainder = fbLen % 1024;
                client.write(fbBuf, remainder);
            }
        }
        
        client.print(tail);
        
        int waitTime = 10000;
        long startTimer = millis();
        boolean state = false;
        
        while ((startTimer + waitTime) > millis()) {
            if (client.available()) {
                char c = client.read();
                if (c == '\n') {
                    if (getAll.length() == 0) state = true; 
                    getAll = "";
                } else if (c != '\r') {
                    getAll += String(c);
                }
                if (state) getBody += String(c);
                startTimer = millis();
            }
            if (!client.connected() && client.available() == 0) break;
        }
        client.stop();
    }
    
    if (jpg_buf != NULL) free(jpg_buf);
    esp_camera_fb_return(fb);
    return getBody;
}

void setup() {
    Serial.begin(115200);
    
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }

    if (!initCamera()) return;

    for (int i = 0; i < NUM_SHOTS; i++) {
        sendPhotoTelegram();
        if (i < NUM_SHOTS - 1) delay(SHOT_DELAY_MS);
    }

    esp_deep_sleep_start();
}

void loop() {
    // Deep sleep active
}
