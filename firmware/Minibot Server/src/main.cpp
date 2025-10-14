#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <sys/time.h>

#define MAX_CLIENTS 18
#define LED_PIN 2 // Onboard LED

// Structure for G-code commands
typedef struct {
    uint8_t client_id;
    char gcode[32]; // Store G1 or G2 command
} GCodeMessage;

// Structure for Sync Message
typedef struct {
    char type[4]; // "SYNC"
    uint32_t timestamp; // Timestamp in milliseconds
} SyncMessage;

// Client MAC Addresses
uint8_t client_mac[MAX_CLIENTS][6] = { /* Populate with MAC addresses */ };

uint32_t getTimestamp() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
}

void sendGCode(uint8_t client_id, const char* command) {
    GCodeMessage message;
    message.client_id = client_id;
    strncpy(message.gcode, command, sizeof(message.gcode));
    esp_now_send(client_mac[client_id], (uint8_t*)&message, sizeof(message));
    Serial.printf("Sent GCode to Client %d: %s\n", client_id, command);
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
}

void sendSync() {
    SyncMessage sync;
    strncpy(sync.type, "SYNC", sizeof(sync.type));
    sync.timestamp = getTimestamp();
    for (int i = 0; i < MAX_CLIENTS; i++) {
        esp_now_send(client_mac[i], (uint8_t*)&sync, sizeof(sync));
    }
    Serial.printf("Sent Sync Message to all clients with timestamp: %u\n", sync.timestamp);
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
}

void setup() {
    WiFi.mode(WIFI_STA);
    pinMode(LED_PIN, OUTPUT);
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Init Failed");
        return;
    }
    Serial.println("Broadcaster Ready");
    // Add peers here
}

void loop() {
    sendGCode(0, "G1 X10 Y10"); // Example command to client 0
    delay(1000);
    sendSync();
    delay(5000);
}