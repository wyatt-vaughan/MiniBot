#include "I2CCommTask.h"
#include "QueueStructs.h"
#include "ElectromagnetTask.h"

// Task handle
TaskHandle_t i2cCommTaskHandle = NULL;

// I2C message types
enum I2CMessageType : uint8_t {
  I2C_MSG_POSITION_CMD = 0x01,
  I2C_MSG_POSITION_REQ = 0x02,
  I2C_MSG_EMAG_CTRL = 0x03,
  I2C_MSG_STATUS_RESP = 0x10,
  I2C_MSG_NOACK_RESP = 0x11,
  I2C_MSG_ACK = 0x20,
  I2C_MSG_ERROR = 0xFF
};

// I2C receive buffer
static uint8_t i2cRxBuffer[32];
static volatile bool i2cDataReceived = false;
static volatile int i2cRxLength = 0;

// I2C transmit buffer for responses
static uint8_t i2cTxBuffer[32];
static volatile int i2cTxLength = 0;

// I2C receive callback
void onI2CReceive(int numBytes) {
  i2cRxLength = 0;
  while (Wire.available() && i2cRxLength < sizeof(i2cRxBuffer)) {
    i2cRxBuffer[i2cRxLength++] = Wire.read();
  }
  i2cDataReceived = true;
}

// I2C request callback
void onI2CRequest() {
  if (i2cTxLength > 0) {
    Wire.write(i2cTxBuffer, i2cTxLength);
    i2cTxLength = 0;
  } else {
    // No data to send, send ACK
    Wire.write(I2C_MSG_ACK);
  }
}

// Initialize I2C communication
void initI2CComm() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_CLOCK_SPEED);
  // Note: If acting as I2C slave, use Wire.begin(address) instead
  // Wire.onReceive(onI2CReceive);
  // Wire.onRequest(onI2CRequest);
  Serial.println("I2C initialized");
}

// Process I2C command from master
static void processI2CCommand() {
  if (i2cRxLength < 1) return;
  
  uint8_t msgType = i2cRxBuffer[0];
  
  switch (msgType) {
    case I2C_MSG_POSITION_CMD: {
      // Format: [msgType, targetID, x(4), y(4), angle(4), duration(4)]
      if (i2cRxLength >= 18) {
        GUICommand cmd;
        cmd.requestType = 0;  // Position command
        cmd.targetID = i2cRxBuffer[1];
        memcpy(&cmd.x, &i2cRxBuffer[2], 4);
        memcpy(&cmd.y, &i2cRxBuffer[6], 4);
        memcpy(&cmd.angle, &i2cRxBuffer[10], 4);
        memcpy(&cmd.duration, &i2cRxBuffer[14], 4);
        
        if (xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(100)) == pdPASS) {
          i2cTxBuffer[0] = I2C_MSG_ACK;
          i2cTxLength = 1;
        } else {
          i2cTxBuffer[0] = I2C_MSG_ERROR;
          i2cTxLength = 1;
        }
      }
      break;
    }
    
    case I2C_MSG_POSITION_REQ: {
      // Format: [msgType, targetID]
      if (i2cRxLength >= 2) {
        GUICommand cmd;
        cmd.requestType = 1;  // Position request
        cmd.targetID = i2cRxBuffer[1];
        cmd.x = 0;
        cmd.y = 0;
        cmd.angle = 0;
        cmd.duration = 0;
        
        if (xQueueSend(commandQueue, &cmd, pdMS_TO_TICKS(100)) == pdPASS) {
          i2cTxBuffer[0] = I2C_MSG_ACK;
          i2cTxLength = 1;
        } else {
          i2cTxBuffer[0] = I2C_MSG_ERROR;
          i2cTxLength = 1;
        }
      }
      break;
    }
    
    case I2C_MSG_EMAG_CTRL: {
      // Format: [msgType, enable]
      if (i2cRxLength >= 2) {
        bool enable = (i2cRxBuffer[1] != 0);
        setElectromagnetEnabled(enable);
        i2cTxBuffer[0] = I2C_MSG_ACK;
        i2cTxLength = 1;
      }
      break;
    }
    
    default:
      i2cTxBuffer[0] = I2C_MSG_ERROR;
      i2cTxLength = 1;
      break;
  }
}

// FreeRTOS I2C communication task
void i2cCommTask(void *parameter) {
  Serial.println("I2C Comm Task started");
  
  while (1) {
    // Process received I2C data
    if (i2cDataReceived) {
      i2cDataReceived = false;
      processI2CCommand();
    }
    
    // Check for status updates from communicator
    GUIStatus status;
    while (xQueueReceive(i2cStatusQueue, &status, 0) == pdPASS) {
      // Prepare status response for next I2C request
      if (status.ackReceived) {
        i2cTxBuffer[0] = I2C_MSG_STATUS_RESP;
        i2cTxBuffer[1] = status.targetID;
        memcpy(&i2cTxBuffer[2], &status.currentX, 4);
        memcpy(&i2cTxBuffer[6], &status.currentY, 4);
        memcpy(&i2cTxBuffer[10], &status.currentAngle, 4);
        memcpy(&i2cTxBuffer[14], &status.timestamp, 4);
        memcpy(&i2cTxBuffer[18], &status.batteryVoltage, 4);
        i2cTxLength = 22;
      } else {
        i2cTxBuffer[0] = I2C_MSG_NOACK_RESP;
        i2cTxBuffer[1] = status.targetID;
        i2cTxLength = 2;
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));  // 10ms cycle
  }
}
