#include "QueueStructs.h"
#include <Arduino.h>

// Queue handle definitions
QueueHandle_t commandQueue = NULL;
QueueHandle_t statusQueue = NULL;

void initQueues() {
  commandQueue = xQueueCreate(10, sizeof(GUICommand));
  statusQueue = xQueueCreate(20, sizeof(GUIStatus));
  
  if (commandQueue == NULL || statusQueue == NULL) {
    Serial.println("Failed to create queues!");
  } else {
    Serial.println("Queues created successfully");
  }
}
