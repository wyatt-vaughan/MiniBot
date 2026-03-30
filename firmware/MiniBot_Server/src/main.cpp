#include <Arduino.h>
#include <WiFi.h>
#include "config.h"
#if ENABLE_JOYSTICK_MODE
  #include "JoystickTask.h"
#else
  #include "GUITask.h"
#endif
#include "CommunicatorTask.h"
#include "QueueStructs.h"
#include "ElectromagnetTask.h"
#include "PythonCommTask.h"
#include "I2CCommTask.h"
#include "LEDStatusTask.h"

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

  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());
  
  // Initialize ESP-NOW
  initESPNow();
  
#if ENABLE_JOYSTICK_MODE
  // Initialize Joystick mode
  initJoystick();
  Serial.println("Running in JOYSTICK MODE");
#else
  // Initialize GUI mode
  initGUI();
  Serial.println("Running in GUI MODE");
#endif
  
  // Initialize Electromagnets
  initElectromagnets();

  // Initialize LED status indicator
  initLEDStatus();
  
  // Initialize Python Serial communication
  // initPythonComm();
  
  // Initialize I2C communication
  // initI2CComm();
  
  // Create FreeRTOS Tasks
#if ENABLE_JOYSTICK_MODE
  xTaskCreatePinnedToCore(
    joystickTask,      // Task function
    "Joystick Task",   // Task name
    4096,              // Stack size (bytes)
    NULL,              // Parameter
    2,                 // Priority
    &joystickTaskHandle, // Task handle
    1                  // Core (0 or 1)
  );
#else
  xTaskCreatePinnedToCore(
    guiTask,           // Task function
    "GUI Task",        // Task name
    8192,              // Stack size (bytes)
    NULL,              // Parameter
    2,                 // Priority
    &guiTaskHandle,    // Task handle
    0                  // Core (0 or 1)
  );
#endif
  
  xTaskCreatePinnedToCore(
    communicatorTask,  // Task function
    "Comm Task",       // Task name
    4096,              // Stack size (bytes)
    NULL,              // Parameter
    2,                 // Priority
    &commTaskHandle,   // Task handle
    0                  // Core (0 or 1)
  );
  
  xTaskCreatePinnedToCore(
    electromagnetTask, // Task function
    "Emag Task",       // Task name
    2048,              // Stack size (bytes)
    NULL,              // Parameter
    1,                 // Priority (lower than comm/gui)
    &emagTaskHandle,   // Task handle
    1                  // Core (0 or 1)
  );

  xTaskCreatePinnedToCore(
    ledStatusTask,        // Task function
    "LED Task",           // Task name
    2048,                 // Stack size (bytes)
    NULL,                 // Parameter
    1,                    // Priority
    &ledStatusTaskHandle, // Task handle
    0                     // Core (0 or 1)
  );
  
  // xTaskCreatePinnedToCore(
  //   pythonCommTask,    // Task function
  //   "Python Task",     // Task name
  //   4096,              // Stack size (bytes)
  //   NULL,              // Parameter
  //   2,                 // Priority
  //   &pythonCommTaskHandle, // Task handle
  //   1                  // Core (0 or 1)
  // );
  
  // xTaskCreatePinnedToCore(
  //   i2cCommTask,       // Task function
  //   "I2C Task",        // Task name
  //   4096,              // Stack size (bytes)
  //   NULL,              // Parameter
  //   2,                 // Priority
  //   &i2cCommTaskHandle, // Task handle
  //   0                  // Core (0 or 1)
  // );
  
  Serial.println("Setup complete!");
}

// ============================================
// Loop (unused - FreeRTOS tasks handle everything)
// ============================================
void loop() {
  // Empty - all work done in FreeRTOS tasks
  vTaskDelay(pdMS_TO_TICKS(1000));
}