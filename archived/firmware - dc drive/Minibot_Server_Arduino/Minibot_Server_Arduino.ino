#include <WiFi.h>
#include <WebServer.h>
#include <vector>
#include <esp_mac.h>
#include "ESP32_NOW.h"

#include "messages.h"


#define WIFI_SSID "ESP32-MOTOR"
#define WIFI_PASS ""
#define WIFI_CHANNEL 6

class ESP_NOW_Broadcast_Peer : public ESP_NOW_Peer {
public:
  // Constructor of the class using the broadcast address
  ESP_NOW_Broadcast_Peer(uint8_t channel, wifi_interface_t iface, const uint8_t *lmk) : ESP_NOW_Peer(ESP_NOW.BROADCAST_ADDR, channel, iface, lmk) {}

  // Destructor of the class
  ~ESP_NOW_Broadcast_Peer() {
    remove(); // Remove the peer from the ESP-NOW network (list of peers).
  }

  // Function to properly initialize the ESP-NOW and register the broadcast peer
  bool begin() {
    // The ESP-NOW communication must be started before adding the peer by calling ESP_NOW.begin().
    // The add() method from the ESP_NOW_Peer class is used to register the peer in the ESP-NOW network.
    if (!ESP_NOW.begin() || !add()) {
      log_e("Failed to initialize ESP-NOW or register the broadcast peer");
      return false;
    }
    return true;
  }

  // Function to send a message to all devices within the network
  bool send_message(const uint8_t *data, size_t len) {
    if (!send(data, len)) { // The send(const uint8_t *data, size_t len) method from the ESP_NOW_Peer class is used to send data to the peer.
      log_e("Failed to broadcast message");
      return false;
    }
    return true;
  }
};

ESP_NOW_Broadcast_Peer broadcast_peer(WIFI_CHANNEL, WIFI_IF_STA, NULL);
MotTestCommand testcmd;
WebServer server(80);
IPAddress local_IP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

void handleRoot() {
  server.send(200, "text/html", R"rawliteral(
    <!DOCTYPE html>
    <html>
    <head>
      <title>Motor Control</title>
      <style>
        body { font-family: Arial; text-align: center; padding: 2em; }
        .switch { margin: 1em; }
      </style>
    </head>
    <body>
      <h2>Motor Control Panel</h2>
      <div class="switch">
        <label>Enable Motor: <input type="checkbox" id="enableSwitch"></label>
      </div>
      <div class="switch">
        <label>M0 Speed: <input type="range" id="speedSlider0" min="-127" max="127" value="0"></label>
        <span id="speedVal0">0</span>
      </div>
      <div class="switch">
        <label>M1 Speed: <input type="range" id="speedSlider1" min="-127" max="127" value="0"></label>
        <span id="speedVal1">0</span>
      </div>
      <div class="switch">
        <label>Target ID: <input type="text" id="targetID" placeholder="0x01" size="4"></label>
      </div>
      <button onclick="send()">Send Command</button>
      <script>
        const enable = document.getElementById('enableSwitch');
        const speed0 = document.getElementById('speedSlider0');
        const speedVal0 = document.getElementById('speedVal0');
        const speed1 = document.getElementById('speedSlider1');
        const speedVal1 = document.getElementById('speedVal1');
        const target = document.getElementById('targetID').value || 0;

        speed0.oninput = () => speedVal0.textContent = speed0.value;
        speed1.oninput = () => speedVal1.textContent = speed1.value;

        function send() {
          const target = document.getElementById('targetID').value || 0;
          fetch(`/set?en=${enable.checked ? 1 : 0}&spd0=${speed0.value}&spd1=${speed1.value}&id=${target}`);
        }
      </script>
    </body>
    </html>
  )rawliteral");
}

// void sendCommand() {
//   Serial.print((String)"Sending cmd to 0x");
//   Serial.println(cmd.targetID, HEX);
//   Serial.println((String)"EN: " + cmd.enabled + "\tFW: " + cmd.forward + "\tSPD: " + cmd.speed);
//   // esp_now_send(broadcastAddress, (uint8_t *)&cmd, sizeof(cmd));
//   if (!broadcast_peer.send_message((uint8_t *)&cmd, sizeof(cmd))) {
//     Serial.println("Failed to broadcast message");
//   }
// }

void sendTestCommand() {
  Serial.print((String)"Sending testcmd to 0x");
  Serial.println(testcmd.targetID, HEX);
  Serial.println((String)"EN: " + testcmd.enabled + "\tm0_speed: " + testcmd.m0_vel + "\tm1_speed: " + testcmd.m1_vel);
  if (!broadcast_peer.send_message((uint8_t *)&testcmd, sizeof(testcmd))) {
    Serial.println("Failed to broadcast message");
  }
}

// void handleSet() {
//   if (server.hasArg("en")) cmd.enabled = server.arg("en").toInt();
//   if (server.hasArg("dir")) cmd.forward = server.arg("dir").toInt();
//   if (server.hasArg("spd")) cmd.speed = server.arg("spd").toInt();
//   if (server.hasArg("id")) {
//     String idStr = server.arg("id");
//     idStr.trim();
//     if (idStr.startsWith("0x") || idStr.startsWith("0X")) {
//       cmd.targetID = strtol(idStr.c_str(), nullptr, 16);  // parse as hex
//     } else {
//       cmd.targetID = idStr.toInt();  // parse as decimal
//     }
//   }

//   sendCommand();
//   server.send(200, "text/plain", "OK");
// }

void handleSet() {
  if (server.hasArg("en")) testcmd.enabled = server.arg("en").toInt();
  if (server.hasArg("spd0")) testcmd.m0_vel = server.arg("spd0").toInt();
  if (server.hasArg("spd1")) testcmd.m1_vel = server.arg("spd1").toInt();
  if (server.hasArg("id")) {
    String idStr = server.arg("id");
    idStr.trim();
    if (idStr.startsWith("0x") || idStr.startsWith("0X")) {
      testcmd.targetID = strtol(idStr.c_str(), nullptr, 16);  // parse as hex
    } else {
      testcmd.targetID = idStr.toInt();  // parse as decimal
    }
  }

  sendTestCommand();
  server.send(200, "text/plain", "OK");
}

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  if (len == sizeof(AckMessage)) {
    AckMessage ack;
    memcpy(&ack, incomingData, sizeof(ack));
    
    const uint8_t *mac = recv_info->src_addr;

    Serial.printf("ACK from 0x%02X | Time: %lu | Position: (%.2f, %.2f)\n",
                  ack.responderID, ack.timestamp, ack.x, ack.y);
  }
}

void register_new_broadcaster(const esp_now_recv_info_t *info, const uint8_t *data, int len, void *arg) {
  Serial.printf("Unknown peer " MACSTR " sent a broadcast message\n", MAC2STR(info->src_addr));
  Serial.println("Registering the peer as a broadcaster");
}


void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_AP_STA);
  WiFi.setChannel(WIFI_CHANNEL);
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  if (!broadcast_peer.begin()) {
    Serial.println("Failed to initialize broadcast peer");
  }

  esp_now_register_recv_cb(OnDataRecv);

  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.begin();
}

void loop() {
  server.handleClient();
}
