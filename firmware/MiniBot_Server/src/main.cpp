#include <Arduino.h>
#include <WiFi.h>
#include "GUITask.h"
#include "CommunicatorTask.h"
#include "QueueStructs.h"

// WiFi Configuration
const char* ssid = "ChessBot-Server";
const char* password = "qwertyuiop";
const int wifiChannel = 6;
IPAddress local_IP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

void setup() {
  Serial.begin(115200);
  Serial.println("\n\nMiniBot Server Starting...");
  
  // Create FreeRTOS Queues
  initQueues();
  
  // Initialize WiFi
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(ssid, password, wifiChannel);
  // WiFi.begin(ssid, password);

  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());
  
  // Serial.print("Connecting to WiFi");
  // int attempts = 0;
  // while (WiFi.status() != WL_CONNECTED && attempts < 20) {
  //   delay(500);
  //   Serial.print(".");
  //   attempts++;
  // }
  
  // if (WiFi.status() == WL_CONNECTED) {
  //   Serial.println("\nWiFi connected!");
  //   Serial.print("IP Address: ");
  //   Serial.println(WiFi.localIP());
  // } else {
  //   Serial.println("\nWiFi connection failed, starting AP mode");
  //   WiFi.softAP("MiniBot-Server", "12345678");
  //   Serial.print("AP IP Address: ");
  //   Serial.println(WiFi.softAPIP());
  // }
  
  // Initialize ESP-NOW
  initESPNow();
  
  // Initialize GUI
  initGUI();
  
  // Create FreeRTOS Tasks
  xTaskCreatePinnedToCore(
    guiTask,           // Task function
    "GUI Task",        // Task name
    8192,              // Stack size (bytes)
    NULL,              // Parameter
    2,                 // Priority
    &guiTaskHandle,    // Task handle
    1                  // Core (0 or 1)
  );
  
  xTaskCreatePinnedToCore(
    communicatorTask,  // Task function
    "Comm Task",       // Task name
    4096,              // Stack size (bytes)
    NULL,              // Parameter
    2,                 // Priority
    &commTaskHandle,   // Task handle
    0                  // Core (0 or 1)
  );
  
  Serial.println("FreeRTOS tasks created");
  Serial.println("Setup complete!");
}

// ============================================
// Loop (unused - FreeRTOS tasks handle everything)
// ============================================
void loop() {
  // Empty - all work done in FreeRTOS tasks
  vTaskDelay(pdMS_TO_TICKS(1000));
}