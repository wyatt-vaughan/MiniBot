#ifndef COMMUNICATOR_TASK_H
#define COMMUNICATOR_TASK_H

#include <Arduino.h>
#include <esp_now.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "ESPNowMessages.h"

// Task handle
extern TaskHandle_t commTaskHandle;

// Peer MAC address
extern uint8_t broadcastAddress[6];

// Initialize ESP-NOW
void initESPNow();

// FreeRTOS Communicator task
void communicatorTask(void *parameter);

// ESP-NOW callbacks
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len);

#endif // COMMUNICATOR_TASK_H
