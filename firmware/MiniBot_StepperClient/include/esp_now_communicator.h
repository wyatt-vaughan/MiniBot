#ifndef __ESP_NOW_COMMUNICATOR_H__
#define __ESP_NOW_COMMUNICATOR_H__

#include "messages_espnow.h"
#include "motion_queue.h"
#include "robot.h"
#include <WiFi.h>
#include <vector>
#include <esp_wifi.h>
#include <esp_mac.h>
// #include "ESP32_NOW.h"

// TODO this ID needs to be unique per device so set on compile or something
#define DEVICE_ID 0x22
// uint8_t *last_mac;

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


// /* ESP-NOW Communication Class */
// class ESP_NOW_Peer_Class : public ESP_NOW_Peer {
// public:
//   ESP_NOW_Peer_Class(const uint8_t *mac_addr, uint8_t channel, wifi_interface_t iface, const uint8_t *lmk) : ESP_NOW_Peer(mac_addr, channel, iface, lmk) {}

//   // Destructor of the class
//   ~ESP_NOW_Peer_Class() {}

//   // Function to register the new broadcaster peer
//   bool add_peer() {
//     if (!add()) { // The add() method from the ESP_NOW_Peer class is used to register the peer in the ESP-NOW network.
//       log_e("Failed to register the broadcast peer");
//       return false;
//     }
//     return true;
//   }

//   void MotTestFunction(const uint8_t *cmdData) {
//   }

//   void MotorFunction(const uint8_t *cmdData) {
//     // memcpy(&incoming_MotCmd, incomingData, len);
//     // if (incoming.targetID == DEVICE_ID) {
//     //   Serial.println("YAY MESSAGE FOR ME :)");
//     //   memcpy(&targetMots, incomingData, len);
//     //   mot_update_reqd = true;
//     //   sendAck(last_mac);
//     // }
//   }

//   void PosCmdFunction(const uint8_t *cmdData) {
    
//   }

//   void PosReqFunction(const uint8_t *cmdData) {
    
//   }

//   void onReceive(const uint8_t *incomingData, size_t len, bool broadcast) {
//     // Handle incoming messages from broadcaster

//     // Check if message is for me
//     uint8_t targetid = 0;
//     memcpy(&targetid, incomingData, 1);
//     Serial.printf("Target ID: %d\n", targetid);
//     if (targetid != DEVICE_ID) return;
//     Serial.println("YAY FOR ME :)");

//     // Check message type and parse by associated function
//     uint8_t messageid = 0;
//     memcpy(&messageid, incomingData + 1, 1);
//     switch (messageid) {
//       case MSG_TYPE_MOT_TEST_COMMAND:
//         MotTestFunction(incomingData);
//         break;
//       case MSG_TYPE_POSITION_COMMAND:
//         PosCmdFunction(incomingData);
//         break;
//       case MSG_TYPE_POSITION_REQUEST:
//         PosReqFunction(incomingData);
//         break;
//       default:
//         Serial.println("Unknown message type");
//         return;
//     }
//   }

//   void sendAck(const uint8_t *mac) {
//     AckMessage ack;
//     ack.responderID = DEVICE_ID;
//     ack.timestamp = millis();
//     ack.x = 12.34;  // Spoofed coordinates
//     ack.y = 56.78;

//     esp_now_send(mac, (uint8_t *)&ack, sizeof(ack));
//   }
// private:
// };


#endif // __ESP_NOW_COMMUNICATOR_H__
