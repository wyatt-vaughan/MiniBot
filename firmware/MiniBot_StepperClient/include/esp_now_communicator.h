#ifndef __ESP_NOW_COMMUNICATOR_H__
#define __ESP_NOW_COMMUNICATOR_H__

#include "messages_espnow.h"
#include "motion_queue.h"
#include "robot.h"
#include <WiFi.h>
#include <vector>
#include <esp_wifi.h>
#include <esp_mac.h>
#include <esp_now.h>
#include <functional>

// TODO this ID needs to be unique per device so set on compile or something
#define DEVICE_ID 0x22

#define WIFI_CHANNEL 6
#define WIFI_POWER WIFI_POWER_11dBm

// Callback type for received broadcast messages
typedef std::function<void(const uint8_t* mac_addr, const uint8_t* data, int len)> EspNowReceiveCallback;

/**
 * ESP-NOW Communicator Task
 * 
 * Priority: HIGH
 * Responsible for:
 * - Receiving commands via ESP-NOW protocol
 * - Parsing received motion commands
 * - Enqueuing commands to the motion queue
 * - Sending status updates back to the remote controller
 * 
 * @param pvParameters Pointer to initialization parameters (unused)
 */
void EspNowCommunicator_Task(void* pvParameters);

/**
 * Initialize the ESP-NOW communicator
 * Should be called before starting the task
 * 
 * @param motion_queue Pointer to MotionQueue
 * @return true on success, false on failure
 */
bool EspNowCommunicator_Init(MotionQueue* motion_queue);

/**
 * Register a callback function to be called when broadcast messages are received
 * 
 * @param callback Function to call with (mac_addr, data, length)
 * @return true on success, false on failure
 */
bool EspNowCommunicator_RegisterCallback(EspNowReceiveCallback callback);

#endif // __ESP_NOW_COMMUNICATOR_H__
