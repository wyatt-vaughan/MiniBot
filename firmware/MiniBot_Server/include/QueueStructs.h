#ifndef QUEUE_STRUCTS_H
#define QUEUE_STRUCTS_H

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "ESPNowMessages.h"

// Internal queue message types
struct GUICommand {
  uint8_t targetID;
  bool isPositionRequest;  // true = position request, false = position command
  float x;
  float y;
  float angle;
  float duration;
};

struct GUIStatus {
  uint8_t targetID;
  bool ackReceived;
  float currentX;
  float currentY;
  float currentAngle;
  uint32_t timestamp;
  float batteryVoltage;
};

// Queue handles
extern QueueHandle_t commandQueue;  // GUI -> Communicator
extern QueueHandle_t statusQueue;   // Communicator -> GUI

// Initialize queues
void initQueues();

#endif // QUEUE_STRUCTS_H
