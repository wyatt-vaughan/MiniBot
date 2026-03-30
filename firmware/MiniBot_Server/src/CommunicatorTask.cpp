#include "CommunicatorTask.h"
#include "QueueStructs.h"
#include "ElectromagnetTask.h"
#include "config.h"
#include <esp_timer.h>

// Task handle
TaskHandle_t commTaskHandle = NULL;

// Store peer MAC addresses (broadcast by default)
uint8_t broadcastAddress[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ACK tracking
static QueueHandle_t ackQueue = NULL;
static QueueHandle_t nackQueue = NULL;
static QueueHandle_t magFieldQueue = NULL;

// ESP-NOW callback when data is sent
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Broadcast Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// ESP-NOW callback when data is received
void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  Serial.printf("ESP-NOW data received: %d bytes\n", len);
  
  // Try to handle as AckMessage first
  if (len == sizeof(AckMessage)) {
    AckMessage ack;
    memcpy(&ack, incomingData, sizeof(ack));
    
    Serial.printf("Received message - Type: %d, ResponderID: 0x%02X\n", ack.msg_type, ack.responderID);
    
    // Check if it's an ACK message
    if (ack.msg_type == MSG_TYPE_ACK_MESSAGE) {
      Serial.printf("ACK message detected from robot 0x%02X\n", ack.responderID);
      Serial.printf("  Position: (%.2f, %.2f) Angle: %.3f rad\n", ack.x, ack.y, ack.orientation_rad);
      Serial.printf("  Timestamp: %u Battery: %.2fV\n", ack.timestamp, ack.battery_voltage);
      
      // Send to ACK queue for timeout handling
      if (ackQueue != NULL) {
        if (xQueueSend(ackQueue, &ack, 0) == pdPASS) {
          Serial.println("  -> Sent to ACK queue");
        } else {
          Serial.println("  -> ACK queue full!");
        }
      } else {
        Serial.println("  -> ACK queue is NULL!");
      }
      
      // Also send to all status queues for GUI/Python/I2C update
      GUIStatus status;
      status.targetID = ack.responderID;
      status.ackReceived = true;
      status.currentX = ack.x;
      status.currentY = ack.y;
      status.currentAngle = ack.orientation_rad;
      status.timestamp = ack.timestamp;
      status.batteryVoltage = ack.battery_voltage;
      status.magnetFieldValid = false;
      
      broadcastStatus(status);
      Serial.println("  -> Broadcast to all status queues");
      return;
    }
  }
  
  // Try to handle as MagneticFieldResponse
  if (len == sizeof(MagneticFieldResponse)) {
    MagneticFieldResponse magField;
    memcpy(&magField, incomingData, sizeof(magField));
    
    Serial.printf("Received message - Type: %d, ResponderID: 0x%02X\n", magField.msg_type, magField.responderID);
    
    if (magField.msg_type == MSG_TYPE_MAG_REQUEST) {
      Serial.printf("Magnetic field response detected from robot 0x%02X\n", magField.responderID);
      Serial.printf("  Field: [%.2f, %.2f, %.2f] gauss\n", magField.field_x_gauss, magField.field_y_gauss, magField.field_z_gauss);
      Serial.printf("  Timestamp: %u\n", magField.timestamp);
      
      // Send to magField queue for timeout handling
      if (magFieldQueue != NULL) {
        if (xQueueSend(magFieldQueue, &magField, 0) == pdPASS) {
          Serial.println("  -> Sent to magField queue");
        } else {
          Serial.println("  -> magField queue full!");
        }
      } else {
        Serial.println("  -> magField queue is NULL!");
      }
      return;
    }
  }

  // Try to handle as NackMessage
  if (len == sizeof(NackMessage)) {
    NackMessage nack;
    memcpy(&nack, incomingData, sizeof(nack));
    if (nack.msg_type == MSG_TYPE_NACK_MESSAGE) {
      Serial.printf("NACK received from robot 0x%02X, err_type=%d\n", nack.responderID, nack.err_type);
      if (nackQueue != NULL) {
        if (xQueueSend(nackQueue, &nack, 0) != pdPASS) {
          Serial.println("  -> NACK queue full!");
        }
      }
      return;
    }
  }

  Serial.printf("Warning: Unhandled message type or size mismatch (len=%d, AckMsg=%d, NackMsg=%d, MagField=%d)\n",
               len, sizeof(AckMessage), sizeof(NackMessage), sizeof(MagneticFieldResponse));
}

// Initialize ESP-NOW
void initESPNow() {
  // Create ACK queue
  ackQueue = xQueueCreate(10, sizeof(AckMessage));
  nackQueue = xQueueCreate(10, sizeof(NackMessage));
  magFieldQueue = xQueueCreate(10, sizeof(MagneticFieldResponse));
  
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
  
  CommandMessage msg;
  
  while (1) {
    // Check for commands from GUI or Joystick
    if (xQueueReceive(commandQueue, &msg, pdMS_TO_TICKS(10)) == pdPASS) {
      
      if (msg.commandType == CMD_TYPE_MOT_TEST) {
        // Handle MotTestCommand (from Joystick or other sources)
        MotTestCommand *motCmd = &msg.data.motCmd;
        Serial.printf("Sending MotTestCommand to 0x%02X: M0=%d, M1=%d\n", 
                     motCmd->targetID, motCmd->m0_vel, motCmd->m1_vel);
        
        // Send broadcast
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)motCmd, sizeof(MotTestCommand));
        
        if (result == ESP_OK) {
          Serial.println("MotTestCommand broadcast sent");
        } else {
          Serial.printf("MotTestCommand send failed: %d\n", result);
        }
      } else if (msg.commandType == CMD_TYPE_GUI) {
        // Handle GUICommand (from GUI)
        GUICommand *cmd = &msg.data.guiCmd;
      
        if (cmd->requestType == 1) {
          // Send position request
          PositionRequest req;
          req.targetID = cmd->targetID;
          req.msg_type = MSG_TYPE_POSITION_REQUEST;
          req.timestamp = millis();
          
          Serial.printf("Sending position request to robot 0x%02X\n", cmd->targetID);
          
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
                if (ack.responderID == cmd->targetID && ack.msg_type == MSG_TYPE_ACK_MESSAGE) {
                  ackReceived = true;
                  Serial.printf("ACK received from robot 0x%02X\n", cmd->targetID);
                  break;
                }
              }
            }
            
            if (!ackReceived) {
              Serial.printf("No ACK received from robot 0x%02X (timeout)\n", cmd->targetID);
              // Send timeout notification to all tasks
              GUIStatus status;
              status.targetID = cmd->targetID;
              status.ackReceived = false;
              status.currentX = 0;
              status.currentY = 0;
              status.currentAngle = 0;
            status.timestamp = 0;
            status.batteryVoltage = 0;
            status.magnetFieldValid = false;
            broadcastStatus(status);
          }
        } else {
          Serial.printf("Error sending position request: %d\n", result);
        }
        
      } else if (cmd->requestType == 2) {
        // Send magnetic field request
        MagneticFieldRequest magReq;
        magReq.targetID = cmd->targetID;
        magReq.msg_type = MSG_TYPE_MAG_REQUEST;
        magReq.timestamp = millis();
        
        Serial.printf("Sending magnetic field request to robot 0x%02X\n", cmd->targetID);
        
        // Clear any pending magField responses for this target
        MagneticFieldResponse tempMag;
        while (xQueueReceive(magFieldQueue, &tempMag, 0) == pdPASS) {
          // Clear queue
        }
        
        // Send broadcast
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&magReq, sizeof(magReq));
        
        if (result == ESP_OK) {
          Serial.println("Magnetic field request broadcast sent");
          
          // Wait for response with 500ms timeout
          MagneticFieldResponse magResp;
          bool respReceived = false;
          uint32_t startTime = millis();
          
          while ((millis() - startTime) < 500) {
            if (xQueueReceive(magFieldQueue, &magResp, pdMS_TO_TICKS(10)) == pdPASS) {
              if (magResp.responderID == cmd->targetID && magResp.msg_type == MSG_TYPE_MAG_REQUEST) {
                respReceived = true;
                Serial.printf("Magnetic field response received from robot 0x%02X\n", cmd->targetID);
                
                // Update status with magnet field values (don't overwrite battery voltage)
                GUIStatus status;
                status.targetID = cmd->targetID;
                status.ackReceived = true;
                status.currentX = 0;
                status.currentY = 0;
                status.currentAngle = 0;
                status.timestamp = magResp.timestamp;
                status.batteryVoltage = -1.0f;  // Use sentinel value to indicate "don't update"
                status.magnetX_gauss = magResp.field_x_gauss;
                status.magnetY_gauss = magResp.field_y_gauss;
                status.magnetZ_gauss = magResp.field_z_gauss;
                status.magnetFieldValid = true;
                broadcastStatus(status);
                break;
              }
            }
          }
          
          if (!respReceived) {
            Serial.printf("No magnetic field response from robot 0x%02X (timeout)\n", cmd->targetID);
            // Send timeout notification
            GUIStatus status;
            status.targetID = cmd->targetID;
            status.ackReceived = false;
            status.currentX = 0;
            status.currentY = 0;
            status.currentAngle = 0;
            status.timestamp = 0;
            status.batteryVoltage = 0;
            status.magnetFieldValid = false;
            broadcastStatus(status);
          }
        } else {
          Serial.printf("Error sending magnetic field request: %d\n", result);
        }
        
      } else {
        // Send position command (requestType == 0)
        PositionCommand posCmd;
        posCmd.targetID = cmd->targetID;
        posCmd.msg_type = MSG_TYPE_POSITION_COMMAND;
        posCmd.timestamp = millis();
        posCmd.target_x_mm = cmd->x;
        posCmd.target_y_mm = cmd->y;
        posCmd.target_a_rad = cmd->angle;
        posCmd.move_duration_ms = cmd->duration;
        
        Serial.printf("Sending position command to robot 0x%02X: (%.2f, %.2f) %.2frad %.2fms\n",
                     cmd->targetID, cmd->x, cmd->y, cmd->angle, cmd->duration);
        
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
              if (ack.responderID == cmd->targetID && ack.msg_type == MSG_TYPE_ACK_MESSAGE) {
                ackReceived = true;
                Serial.printf("ACK received from robot 0x%02X\n", cmd->targetID);
                break;
              }
            }
          }
          
          if (!ackReceived) {
            Serial.printf("No ACK received from robot 0x%02X (timeout)\n", cmd->targetID);
            // Send timeout notification to all tasks
            GUIStatus status;
            status.targetID = cmd->targetID;
            status.ackReceived = false;
            status.currentX = 0;
            status.currentY = 0;
            status.currentAngle = 0;
            status.timestamp = 0;
            status.batteryVoltage = 0;
            status.magnetFieldValid = false;
            broadcastStatus(status);
          }
        } else {
          Serial.printf("Error sending position command: %d\n", result);
        }
        }  // End of GUI command handling
      } else if (msg.commandType == CMD_TYPE_POS_SYNC) {
        // Broadcast PosSyncCommand to all units
        PosSyncCommand syncCmd;
        syncCmd.targetID = 0xFF;
        syncCmd.msg_type = MSG_TYPE_POS_SYNC_COMMAND;
        syncCmd.timestamp = (uint32_t)esp_timer_get_time();
        syncCmd.timeout_ms = 2 * EMAG_FRAME_LEN_MS;

        Serial.printf("Sending PosSyncCommand to all units (timeout=%dms)\n", syncCmd.timeout_ms);

        // Clear any stale ACKs and NACKs from previous transactions
        AckMessage tempAck;
        while (xQueueReceive(ackQueue, &tempAck, 0) == pdPASS) {}
        NackMessage tempNack;
        while (xQueueReceive(nackQueue, &tempNack, 0) == pdPASS) {}

        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&syncCmd, sizeof(syncCmd));
        if (result == ESP_OK) {
          Serial.println("PosSyncCommand broadcast sent");

          // Trigger local sync pulse after waiting 20ms to ensure command delivery
          vTaskDelay(pdMS_TO_TICKS(20));
          triggerSyncPulse();

          // Collect ACK/NACK responses for 5 * EMAG_FRAME_LEN_MS
          uint32_t collectStart = millis();
          while ((millis() - collectStart) < (5 * EMAG_FRAME_LEN_MS)) {
            AckMessage ack;
            if (xQueueReceive(ackQueue, &ack, pdMS_TO_TICKS(5)) == pdPASS) {
              if (ack.msg_type == MSG_TYPE_ACK_MESSAGE) {
                Serial.printf("Sync ACK from robot 0x%02X\n", ack.responderID);
                GUIStatus syncStatus = {};
                syncStatus.targetID = ack.responderID;
                syncStatus.ackReceived = true;
                syncStatus.batteryVoltage = -1.0f;
                syncStatus.syncStatus = 1;
                broadcastStatus(syncStatus);
              }
            }
            NackMessage nack;
            if (xQueueReceive(nackQueue, &nack, 0) == pdPASS) {
              if (nack.msg_type == MSG_TYPE_NACK_MESSAGE) {
                Serial.printf("Sync NACK from robot 0x%02X (err=%d)\n", nack.responderID, nack.err_type);
                GUIStatus syncStatus = {};
                syncStatus.targetID = nack.responderID;
                syncStatus.ackReceived = false;
                syncStatus.batteryVoltage = -1.0f;
                syncStatus.syncStatus = 2;
                broadcastStatus(syncStatus);
              }
            }
          }
          Serial.println("Sync response collection window closed");
        } else {
          Serial.printf("PosSyncCommand send failed: %d\n", result);
        }
      }  // End of command type check
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
