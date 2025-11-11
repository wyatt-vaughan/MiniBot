#include <WiFi.h>
#include <vector>
#include <esp_wifi.h>
#include <esp_mac.h>
#include "ESP32_NOW.h"

#include "messages.h"
#include "minibot.h"

#define DEVICE_ID 0x22

#define AMP_PIN 0
#define BATTERY_PIN 1
#define MOTOR_SLEEP_PIN 7
#define MOTOR_L_CTRL_A 6
#define MOTOR_L_CTRL_B 5
#define MOTOR_R_CTRL_A 4
#define MOTOR_R_CTRL_B 10

#define WIFI_CHANNEL 6

uint8_t *last_mac;
size_t dataLen = 0;
const uint8_t *dataPtr;

class ESP_NOW_Peer_Class : public ESP_NOW_Peer {
public:
  ESP_NOW_Peer_Class(const uint8_t *mac_addr, uint8_t channel, wifi_interface_t iface, const uint8_t *lmk) : ESP_NOW_Peer(mac_addr, channel, iface, lmk) {}

  // Destructor of the class
  ~ESP_NOW_Peer_Class() {}

  // Function to register the new broadcaster peer
  bool add_peer() {
    if (!add()) { // The add() method from the ESP_NOW_Peer class is used to register the peer in the ESP-NOW network.
      log_e("Failed to register the broadcast peer");
      return false;
    }
    return true;
  }

  void MotTestFunction() {
  }

  void MotorFunction() {
    // memcpy(&incoming_MotCmd, incomingData, len);
    // if (incoming.targetID == DEVICE_ID) {
    //   Serial.println("YAY MESSAGE FOR ME :)");
    //   memcpy(&targetMots, incomingData, len);
    //   mot_update_reqd = true;
    //   sendAck(last_mac);
    // }
  }

  void PosCmdFunction() {
    
  }

  void PosReqFunction() {
    
  }

  void onReceive(const uint8_t *incomingData, size_t len, bool broadcast) {
    // Handle incoming messages from broadcaster

    // Check if message is for me
    uint8_t targetid = 0;
    memcpy(&targetid, incomingData, 1);
    Serial.printf("Target ID: %d\n", targetid);
    if (targetid == DEVICE_ID) {
      Serial.println("YAY FOR ME :)");
      dataPtr = incomingData;
      dataLen = len;
    }
  }

  void sendAck(const uint8_t *mac) {
    AckMessage ack;
    ack.responderID = DEVICE_ID;
    ack.timestamp = millis();
    ack.x = 12.34;  // Spoofed coordinates
    ack.y = 56.78;

    esp_now_send(mac, (uint8_t *)&ack, sizeof(ack));
  }

  float getBatteryVoltage(void)
  {
    float voltage = analogRead(BATTERY_PIN);
    voltage *= (3.23 * 1.85 / 4095.0);  // a little bit fudged heh
    return voltage;
  }
private:
};

std::vector<ESP_NOW_Peer_Class> broadcasters;
MiniBot_DC bot = MiniBot_DC(MOTOR_SLEEP_PIN, MOTOR_L_CTRL_A, MOTOR_L_CTRL_B, MOTOR_R_CTRL_B, MOTOR_R_CTRL_A);

void register_new_broadcaster(const esp_now_recv_info_t *info, const uint8_t *data, int len, void *arg) {
  if (memcmp(info->des_addr, ESP_NOW.BROADCAST_ADDR, 6) == 0) { // Check if the message was sent to the broadcast address
    Serial.printf("Unknown peer " MACSTR " sent a broadcast message\n", MAC2STR(info->src_addr));
    Serial.println("Registering the peer as a broadcaster");
    // memcpy(last_mac, info->src_addr, 6);

    // Create a new broadcaster object
    ESP_NOW_Peer_Class new_broadcaster(info->src_addr, WIFI_CHANNEL, WIFI_IF_STA, NULL);

    // Add the new broadcaster to the list of broadcasters
    broadcasters.push_back(new_broadcaster);

    // Register the new broadcaster in the ESP-NOW network
    if (!broadcasters.back().add_peer()) {
      Serial.println("Failed to register the new broadcaster");
      return;
    }
  } else {
    // The receiver will only receive broadcast messages
    log_v("Received a unicast message from " MACSTR, MAC2STR(info->src_addr));
    log_v("Igorning the message");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Client starting...");

  WiFi.mode(WIFI_STA);
  WiFi.setChannel(WIFI_CHANNEL); // must match your server AP channel
  Serial.printf("WiFi started on channel %d\n", WiFi.channel());

  // esp_wifi_set_max_tx_power(40);

  if (!ESP_NOW.begin()) {
    Serial.println("Failed to initialize ESP-NOW");
  }

  // Register the new peer callback that we created before
  // The NULL parameter is an optional argument that can be passed to the callback.
  ESP_NOW.onNewPeer(register_new_broadcaster, NULL);

  Serial.println("ESP-NOW initialized and listening...");

  bot.initialize();
}

void loop() {
  if (dataLen > 0) {
    // Parse message contents based on size. MAKE SURE NO IDENTICAL SIZE PACKETS :)
    Serial.printf("MSG RECEIVED SIZE %d\n", dataLen);
    switch (dataLen) {
      case sizeof(MotTestCommand): {
        MotTestCommand newcmd;
        memcpy(&newcmd, dataPtr, dataLen);
        Serial.printf("EN: %d\tM0vvel: %d\tM1vel: %d\n", newcmd.enabled, newcmd.m0_vel, newcmd.m1_vel);
        bot.MotorDebug(newcmd.enabled, newcmd.m0_vel, newcmd.m1_vel);
        break;
      }
      case sizeof(PositionCommand): {
        PositionCommand newcmd;
        memcpy(&newcmd, dataPtr, dataLen);
        // PosCmdFunction();
        break;
      }
      case sizeof(PositionRequest): {
        PositionRequest newcmd;
        memcpy(&newcmd, dataPtr, dataLen);
        // PosReqFunction();
        break;
      }
      case sizeof(AckMessage): {
        Serial.printf("ERROR: AckMessage NOT VALID CMD");
        break;
      }
    }

    dataLen = 0;
  }
}
