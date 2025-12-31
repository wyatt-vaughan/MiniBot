#include "CommunicatorTask.h"
#include "QueueStructs.h"

// Task handle
TaskHandle_t commTaskHandle = NULL;

// Store peer MAC addresses (broadcast by default)
uint8_t broadcastAddress[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ACK tracking
static QueueHandle_t ackQueue = NULL;

// ESP-NOW callback when data is sent
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Broadcast Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// ESP-NOW callback when data is received
void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  if (len == sizeof(AckMessage)) {
    AckMessage ack;
    memcpy(&ack, incomingData, sizeof(ack));
    
    // Check if it's an ACK message
    if (ack.msg_type == MSG_TYPE_ACK_MESSAGE) {
      // Send to ACK queue for timeout handling
      if (ackQueue != NULL) {
        xQueueSend(ackQueue, &ack, 0);
      }
      
      // Also send to status queue for GUI update
      GUIStatus status;
      status.targetID = ack.responderID;
      status.ackReceived = true;
      status.currentX = ack.x;
      status.currentY = ack.y;
      status.currentAngle = ack.orientation_rad;
      status.timestamp = ack.timestamp;
      status.batteryVoltage = ack.battery_voltage;
      
      if (xQueueSend(statusQueue, &status, 0) != pdPASS) {
        Serial.println("Status queue full!");
      }
    }
  }
}

// Initialize ESP-NOW
void initESPNow() {
  // Create ACK queue
  ackQueue = xQueueCreate(10, sizeof(AckMessage));
  
  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialization failed");
    return;
  }
  
  Serial.println("ESP-NOW initialized");
  
  // Register callbacks
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);
  
  // Add broadcast peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
  } else {
    Serial.println("Broadcast peer added successfully");
  }
}

// FreeRTOS Communicator Task
void communicatorTask(void *parameter) {
  Serial.println("Communicator Task started");
  
  GUICommand cmd;
  
  while (1) {
    // Check for commands from GUI
    if (xQueueReceive(commandQueue, &cmd, pdMS_TO_TICKS(10)) == pdPASS) {
      
      if (cmd.isPositionRequest) {
        // Send position request
        PositionRequest req;
        req.targetID = cmd.targetID;
        req.msg_type = MSG_TYPE_POSITION_REQUEST;
        req.timestamp = millis();
        
        Serial.printf("Sending position request to robot 0x%02X\n", cmd.targetID);
        
        // Clear any pending ACKs for this target
        AckMessage tempAck;
        while (xQueueReceive(ackQueue, &tempAck, 0) == pdPASS) {
          // Clear queue
        }
        
        // Send broadcast
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&req, sizeof(req));
        
        if (result == ESP_OK) {
          Serial.println("Position request broadcast sent");
          
          // Wait for ACK with 500ms timeout
          AckMessage ack;
          bool ackReceived = false;
          uint32_t startTime = millis();
          
          while ((millis() - startTime) < 500) {
            if (xQueueReceive(ackQueue, &ack, pdMS_TO_TICKS(10)) == pdPASS) {
              if (ack.responderID == cmd.targetID && ack.msg_type == MSG_TYPE_ACK_MESSAGE) {
                ackReceived = true;
                Serial.printf("ACK received from robot 0x%02X\n", cmd.targetID);
                break;
              }
            }
          }
          
          if (!ackReceived) {
            Serial.printf("No ACK received from robot 0x%02X (timeout)\n", cmd.targetID);
            // Send timeout notification to GUI
            GUIStatus status;
            status.targetID = cmd.targetID;
            status.ackReceived = false;
            status.currentX = 0;
            status.currentY = 0;
            status.currentAngle = 0;
            status.timestamp = 0;
            status.batteryVoltage = 0;
            xQueueSend(statusQueue, &status, 0);
          }
        } else {
          Serial.printf("Error sending position request: %d\n", result);
        }
        
      } else {
        // Send position command
        PositionCommand posCmd;
        posCmd.targetID = cmd.targetID;
        posCmd.msg_type = MSG_TYPE_POSITION_COMMAND;
        posCmd.timestamp = millis();
        posCmd.target_x_mm = cmd.x;
        posCmd.target_y_mm = cmd.y;
        posCmd.target_a_rad = cmd.angle;
        posCmd.move_duration_ms = cmd.duration;
        
        Serial.printf("Sending position command to robot 0x%02X: (%.2f, %.2f) %.2frad %.2fms\n",
                     cmd.targetID, cmd.x, cmd.y, cmd.angle, cmd.duration);
        
        // Clear any pending ACKs for this target
        AckMessage tempAck;
        while (xQueueReceive(ackQueue, &tempAck, 0) == pdPASS) {
          // Clear queue
        }
        
        // Send broadcast
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&posCmd, sizeof(posCmd));
        
        if (result == ESP_OK) {
          Serial.println("Position command broadcast sent");
          
          // Wait for ACK with 500ms timeout
          AckMessage ack;
          bool ackReceived = false;
          uint32_t startTime = millis();
          
          while ((millis() - startTime) < 500) {
            if (xQueueReceive(ackQueue, &ack, pdMS_TO_TICKS(10)) == pdPASS) {
              if (ack.responderID == cmd.targetID && ack.msg_type == MSG_TYPE_ACK_MESSAGE) {
                ackReceived = true;
                Serial.printf("ACK received from robot 0x%02X\n", cmd.targetID);
                break;
              }
            }
          }
          
          if (!ackReceived) {
            Serial.printf("No ACK received from robot 0x%02X (timeout)\n", cmd.targetID);
            // Send timeout notification to GUI
            GUIStatus status;
            status.targetID = cmd.targetID;
            status.ackReceived = false;
            status.currentX = 0;
            status.currentY = 0;
            status.currentAngle = 0;
            status.timestamp = 0;
            status.batteryVoltage = 0;
            xQueueSend(statusQueue, &status, 0);
          }
        } else {
          Serial.printf("Error sending position command: %d\n", result);
        }
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
