#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <sys/time.h>

#define CLIENT_ID 0 // Unique per client
#define LED_PIN 2 // Onboard LED

// Structure for incoming G-code commands
typedef struct {
    uint8_t client_id;
    char gcode[32];
} GCodeMessage;

// Structure for sync message
typedef struct {
    char type[4];
    uint32_t timestamp;
} SyncMessage;

// Structure for position response
typedef struct {
    uint8_t client_id;
    float x;
    float y;
    uint32_t sync_timestamp;
    uint32_t received_timestamp;
} PositionMessage;

GCodeMessage commandQueue[10]; // Simple queue
int queueSize = 0;

uint8_t broadcaster_mac[6] = { /* Populate with broadcaster MAC */ };

uint32_t getTimestamp() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
}

void sendPosition(float x, float y, uint32_t sync_timestamp, uint32_t received_timestamp) {
    PositionMessage pos;
    pos.client_id = CLIENT_ID;
    pos.x = x;
    pos.y = y;
    pos.sync_timestamp = sync_timestamp;
    pos.received_timestamp = received_timestamp;
    esp_now_send(broadcaster_mac, (uint8_t*)&pos, sizeof(pos));
    Serial.printf("Client %d: Sent Position X: %.2f Y: %.2f, Sync TS: %u, Received TS: %u\n", CLIENT_ID, x, y, sync_timestamp, received_timestamp);
}

void onDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    digitalWrite(LED_PIN, HIGH);
    uint32_t received_timestamp = getTimestamp();
    
    if (len == sizeof(GCodeMessage)) {
        GCodeMessage msg;
        memcpy(&msg, incomingData, sizeof(msg));
        if (msg.client_id == CLIENT_ID && queueSize < 10) {
            commandQueue[queueSize++] = msg;
            Serial.printf("Client %d: Received GCode: %s\n", CLIENT_ID, msg.gcode);
        }
    } else if (len == sizeof(SyncMessage)) {
        SyncMessage sync;
        memcpy(&sync, incomingData, sizeof(sync));
        if (strncmp(sync.type, "SYNC", 4) == 0) {
            Serial.printf("Client %d: Received Sync Message with timestamp: %u\n", CLIENT_ID, sync.timestamp);
            if (queueSize > 0) {
                // Execute stored G-code (to be implemented)
            } else {
                // Send current position via ESP-NOW
                sendPosition(0.0, 0.0, sync.timestamp, received_timestamp); // Placeholder position
            }
        }
    }
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
    esp_now_register_recv_cb(onDataRecv);
    Serial.printf("Client %d Ready\n", CLIENT_ID);
}

void loop() {
    // Execution logic will be implemented here
}
