#include <WiFi.h>
#include <esp_now.h>
#include <WebServer.h>

#define WIFI_SSID "ESP32-MOTOR"
#define WIFI_PASS ""

typedef struct {
  uint8_t targetID;
  bool enabled;
  bool forward;
  uint8_t speed;
} MotorCommand;

typedef struct {
  uint8_t responderID;
  uint32_t timestamp;
  float x;
  float y;
} AckMessage;

MotorCommand cmd;
WebServer server(80);
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void sendCommand() {
  Serial.print("Sending cmd to");
  esp_now_send(broadcastAddress, (uint8_t *)&cmd, sizeof(cmd));
}

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
        <label>Direction: <input type="checkbox" id="dirSwitch"></label>
      </div>
      <div class="switch">
        <label>Speed: <input type="range" id="speedSlider" min="0" max="255" value="180"></label>
        <span id="speedVal">180</span>
      </div>
      <button onclick="send()">Send Command</button>

      <script>
        const enable = document.getElementById('enableSwitch');
        const dir = document.getElementById('dirSwitch');
        const speed = document.getElementById('speedSlider');
        const speedVal = document.getElementById('speedVal');

        speed.oninput = () => speedVal.textContent = speed.value;

        function send() {
          fetch(`/set?en=${enable.checked ? 1 : 0}&dir=${dir.checked ? 1 : 0}&spd=${speed.value}`);
        }
      </script>
    </body>
    </html>
  )rawliteral");
}

void handleSet() {
  if (server.hasArg("en")) cmd.enabled = server.arg("en").toInt();
  if (server.hasArg("dir")) cmd.forward = server.arg("dir").toInt();
  if (server.hasArg("spd")) cmd.speed = server.arg("spd").toInt();

  cmd.targetID = 0x01;
  sendCommand();
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

void setup() {
  Serial.begin(115200);

  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  Serial.println(WiFi.softAPIP());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (!esp_now_is_peer_exist(broadcastAddress)) {
    esp_now_add_peer(&peerInfo);
  }

  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.begin();
}

void loop() {
  server.handleClient();
}
