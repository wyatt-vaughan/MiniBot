#include "PythonCommTask.h"
#include "QueueStructs.h"
#include "ElectromagnetTask.h"

// Task handle
TaskHandle_t pythonCommTaskHandle = NULL;

// Serial2 for Python communication
#define PYTHON_SERIAL Serial2
#define PYTHON_BAUD 115200
#define PYTHON_RX_PIN 16
#define PYTHON_TX_PIN 17

// Buffer for incoming serial data
static char serialBuffer[256];
static int bufferIndex = 0;

// Initialize Python communication
void initPythonComm() {
  PYTHON_SERIAL.begin(PYTHON_BAUD, SERIAL_8N1, PYTHON_RX_PIN, PYTHON_TX_PIN);
  Serial.println("Python Serial initialized");
}

// Parse and process incoming command
// Format: "cmd,<targetID>,<x>,<y>,<angle>,<duration>"
// Format: "req,<targetID>"
// Format: "emag,<0|1>"
static void processSerialCommand(const char* command) {
  if (strncmp(command, "cmd,", 4) == 0) {
    // Position command
    GUICommand cmd;
    cmd.isPositionRequest = false;
    
    // Parse: cmd,<id>,<x>,<y>,<angle>,<duration>
    char* ptr = (char*)command + 4;
    cmd.targetID = (uint8_t)strtol(ptr, &ptr, 16);  // Parse hex ID
    if (*ptr == ',') ptr++;
    cmd.x = strtof(ptr, &ptr);
    if (*ptr == ',') ptr++;
    cmd.y = strtof(ptr, &ptr);
    if (*ptr == ',') ptr++;
    cmd.angle = strtof(ptr, &ptr);
    if (*ptr == ',') ptr++;
    cmd.duration = strtof(ptr, NULL);
    
    if (xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(100)) == pdPASS) {
      PYTHON_SERIAL.printf("OK,cmd,%02X\n", cmd.targetID);
    } else {
      PYTHON_SERIAL.printf("ERR,queue_full\n");
    }
    
  } else if (strncmp(command, "req,", 4) == 0) {
    // Position request
    GUICommand cmd;
    cmd.isPositionRequest = true;
    cmd.targetID = (uint8_t)strtol(command + 4, NULL, 16);
    cmd.x = 0;
    cmd.y = 0;
    cmd.angle = 0;
    cmd.duration = 0;
    
    if (xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(100)) == pdPASS) {
      PYTHON_SERIAL.printf("OK,req,%02X\n", cmd.targetID);
    } else {
      PYTHON_SERIAL.printf("ERR,queue_full\n");
    }
    
  } else if (strncmp(command, "emag,", 5) == 0) {
    // Electromagnet control
    bool enable = (command[5] == '1');
    setElectromagnetEnabled(enable);
    PYTHON_SERIAL.printf("OK,emag,%d\n", enable ? 1 : 0);
    
  } else if (strcmp(command, "ping") == 0) {
    // Simple ping/pong for connection check
    PYTHON_SERIAL.println("pong");
    
  } else if (strcmp(command, "status") == 0) {
    // Return electromagnet status
    PYTHON_SERIAL.printf("emag,%d\n", getElectromagnetEnabled() ? 1 : 0);
    
  } else {
    PYTHON_SERIAL.printf("ERR,unknown_cmd\n");
  }
}

// FreeRTOS Python communication task
void pythonCommTask(void *parameter) {
  Serial.println("Python Comm Task started");
  
  while (1) {
    // Check for incoming serial data
    while (PYTHON_SERIAL.available()) {
      char c = PYTHON_SERIAL.read();
      
      if (c == '\n' || c == '\r') {
        if (bufferIndex > 0) {
          serialBuffer[bufferIndex] = '\0';
          processSerialCommand(serialBuffer);
          bufferIndex = 0;
        }
      } else if (bufferIndex < sizeof(serialBuffer) - 1) {
        serialBuffer[bufferIndex++] = c;
      }
    }
    
    // Check for status updates from communicator
    GUIStatus status;
    while (xQueueReceive(pythonStatusQueue, &status, 0) == pdPASS) {
      // Forward status to Python
      if (status.ackReceived) {
        PYTHON_SERIAL.printf("status,%02X,%.3f,%.3f,%.4f,%u,%.2f\n",
          status.targetID,
          status.currentX,
          status.currentY,
          status.currentAngle,
          status.timestamp,
          status.batteryVoltage);
      } else {
        PYTHON_SERIAL.printf("noack,%02X\n", status.targetID);
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));  // 10ms cycle
  }
}
