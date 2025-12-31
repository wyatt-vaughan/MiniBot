#include "GUITask.h"
#include "QueueStructs.h"

// Web Server
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Status storage for 36 robots (12x3 grid)
GUIStatus robotStatus[36];

// Task handle
TaskHandle_t guiTaskHandle = NULL;

// HTML Webpage
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>MiniBot Controller</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial; margin: 10px; background-color: #1a1a1a; color: #fff; }
    h1 { text-align: center; color: #4CAF50; }
    .grid { display: grid; grid-template-columns: repeat(12, 1fr); gap: 10px; margin: 20px auto; max-width: 2400px; }
    .cell { background: #2a2a2a; border: 2px solid #444; border-radius: 8px; padding: 10px; }
    .cell-header { background: #3a3a3a; padding: 5px; margin: -10px -10px 10px -10px; border-radius: 6px 6px 0 0; text-align: center; font-weight: bold; }
    .input-group { margin: 5px 0; }
    label { display: block; font-size: 11px; color: #aaa; margin-bottom: 2px; }
    input[type="text"], input[type="number"] { width: 100%; padding: 4px; box-sizing: border-box; background: #1a1a1a; color: #fff; border: 1px solid #555; border-radius: 4px; font-size: 12px; }
    button { width: 100%; padding: 8px; margin-top: 5px; background: #4CAF50; color: white; border: none; border-radius: 4px; cursor: pointer; font-weight: bold; }
    button:hover { background: #45a049; }
    button:active { background: #3d8b40; }
    .status { margin-top: 10px; padding-top: 10px; border-top: 1px solid #444; font-size: 11px; }
    .status div { margin: 3px 0; }
    .status-label { color: #888; }
    .status-value { color: #4CAF50; font-weight: bold; }
    .status-error { color: #ff5555; font-weight: bold; }
    .btn-request { background: #2196F3; margin-top: 5px; }
    .btn-request:hover { background: #1976D2; }
    .btn-request:active { background: #1565C0; }
    input[type="text"]:focus, input[type="number"]:focus { outline: none; border-color: #4CAF50; }
  </style>
</head>
<body>
  <h1>MiniBot Server Controller</h1>
  <div class="grid" id="robotGrid"></div>

  <script>
    var gateway = `ws://${window.location.hostname}/ws`;
    var websocket;
    var robotData = {};

    function initWebSocket() {
      websocket = new WebSocket(gateway);
      websocket.onopen = onOpen;
      websocket.onclose = onClose;
      websocket.onmessage = onMessage;
    }

    function onOpen(event) {
      console.log('WebSocket Connected');
    }

    function onClose(event) {
      console.log('WebSocket Disconnected');
      setTimeout(initWebSocket, 2000);
    }

    function onMessage(event) {
      var data = JSON.parse(event.data);
      robotData[data.id] = data;
      updateStatus(data.id);
    }

    function sendCommand(id) {
      var targetId = document.getElementById('tid_' + id).value;
      var x = document.getElementById('x_' + id).value;
      var y = document.getElementById('y_' + id).value;
      var angle = document.getElementById('a_' + id).value;
      var duration = document.getElementById('d_' + id).value;
      
      var message = 'cmd,' + targetId + ',' + x + ',' + y + ',' + angle + ',' + duration;
      websocket.send(message);
      console.log('Sent: ' + message);
    }

    function requestPosition(id) {
      var targetId = document.getElementById('tid_' + id).value;
      var message = 'req,' + targetId;
      websocket.send(message);
      console.log('Position request: ' + message);
    }

    function updateStatus(id) {
      var data = robotData[id];
      if (data) {
        if (data.ack === false) {
          document.getElementById('pos_' + id).innerHTML = 
            'Position: <span class="status-error">NO ACK</span>';
          document.getElementById('ang_' + id).innerHTML = 
            'Angle: <span class="status-error">TIMEOUT</span>';
          document.getElementById('ts_' + id).innerHTML = 
            'Last Update: <span class="status-error">--</span>';
          document.getElementById('batt_' + id).innerHTML = 
            'Battery: <span class="status-error">--</span>';
        } else {
          document.getElementById('pos_' + id).innerHTML = 
            'Position: <span class="status-value">(' + data.x.toFixed(2) + ', ' + data.y.toFixed(2) + ')</span>';
          document.getElementById('ang_' + id).innerHTML = 
            'Angle: <span class="status-value">' + data.angle.toFixed(3) + ' rad</span>';
          document.getElementById('ts_' + id).innerHTML = 
            'Last Update: <span class="status-value">' + data.ts + ' ms</span>';
          document.getElementById('batt_' + id).innerHTML = 
            'Battery: <span class="status-value">' + data.batt.toFixed(2) + 'V</span>';
        }
      }
    }

    function createGrid() {
      var grid = document.getElementById('robotGrid');
      for (var i = 0; i < 36; i++) {
        var cell = document.createElement('div');
        cell.className = 'cell';
        
        var hexId = '0x' + (i < 16 ? '0' : '') + i.toString(16).toUpperCase();
        
        cell.innerHTML = `
          <div class="cell-header">Robot ${i}</div>
          <div class="input-group">
            <label>Target ID:</label>
            <input type="text" id="tid_${i}" value="${hexId}">
          </div>
          <div class="input-group">
            <label>X Target (m):</label>
            <input type="number" id="x_${i}" value="0.0" step="1">
          </div>
          <div class="input-group">
            <label>Y Target (m):</label>
            <input type="number" id="y_${i}" value="0.0" step="1">
          </div>
          <div class="input-group">
            <label>Angle (rad):</label>
            <input type="number" id="a_${i}" value="0.0" step="0.1">
          </div>
          <div class="input-group">
            <label>Duration (s):</label>
            <input type="number" id="d_${i}" value="1.0" step="0.1">
          </div>
          <button onclick="sendCommand(${i})">SEND</button>
          <button class="btn-request" onclick="requestPosition(${i})">REQUEST POSE</button>
          <div class="status">
            <div id="pos_${i}" class="status-label">Position: --</div>
            <div id="ang_${i}" class="status-label">Angle: --</div>
            <div id="ts_${i}" class="status-label">Last Update: --</div>
            <div id="batt_${i}" class="status-label">Battery: --</div>
          </div>
        `;
        
        grid.appendChild(cell);
        robotData[i] = {id: i, x: 0, y: 0, angle: 0, ts: 0, batt: 0, ack: true};
      }
    }

    window.addEventListener('load', function() {
      createGrid();
      initWebSocket();
    });
  </script>
</body>
</html>
)rawliteral";

// WebSocket event handler
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, 
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("WebSocket client #%u connected\n", client->id());
    // Don't send initial status - client will get updates as they occur
    // This prevents overwhelming the WebSocket queue
  } 
  else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
  }
  else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      data[len] = 0;
      String message = (char*)data;
      
      // Check message type: "cmd,..." or "req,..."
      if (message.startsWith("cmd,")) {
        // Parse command: format "cmd,id,x,y,angle,duration"
        GUICommand cmd;
        cmd.isPositionRequest = false;
        
        int idx1 = message.indexOf(',', 4);  // After "cmd,"
        int idx2 = message.indexOf(',', idx1 + 1);
        int idx3 = message.indexOf(',', idx2 + 1);
        int idx4 = message.indexOf(',', idx3 + 1);
        
        if (idx1 > 0 && idx2 > 0 && idx3 > 0 && idx4 > 0) {
          String targetIdStr = message.substring(4, idx1);
          // Parse hex string (e.g., "0x05" or "5")
          if (targetIdStr.startsWith("0x") || targetIdStr.startsWith("0X")) {
            cmd.targetID = (uint8_t)strtol(targetIdStr.c_str(), NULL, 16);
          } else {
            cmd.targetID = targetIdStr.toInt();
          }
          
          cmd.x = message.substring(idx1 + 1, idx2).toFloat();
          cmd.y = message.substring(idx2 + 1, idx3).toFloat();
          cmd.angle = message.substring(idx3 + 1, idx4).toFloat();
          cmd.duration = message.substring(idx4 + 1).toFloat();
          
          // Send to command queue
          if (xQueueSend(commandQueue, &cmd, 0) == pdPASS) {
            Serial.printf("Command queued for robot 0x%02X: x=%.2f y=%.2f a=%.2f d=%.2f\n", 
                         cmd.targetID, cmd.x, cmd.y, cmd.angle, cmd.duration);
          } else {
            Serial.println("Command queue full!");
          }
        } else {
          Serial.println("Failed to parse command message");
        }
      } else if (message.startsWith("req,")) {
        // Parse request: format "req,id"
        GUICommand cmd;
        cmd.isPositionRequest = true;
        
        String targetIdStr = message.substring(4);
        // Parse hex string (e.g., "0x05" or "5")
        if (targetIdStr.startsWith("0x") || targetIdStr.startsWith("0X")) {
          cmd.targetID = (uint8_t)strtol(targetIdStr.c_str(), NULL, 16);
        } else {
          cmd.targetID = targetIdStr.toInt();
        }
        
        cmd.x = 0;
        cmd.y = 0;
        cmd.angle = 0;
        cmd.duration = 0;
        
        // Send to command queue
        if (xQueueSend(commandQueue, &cmd, 0) == pdPASS) {
          Serial.printf("Position request queued for robot 0x%02X\n", cmd.targetID);
        } else {
          Serial.println("Command queue full!");
        }
      }
    }
  }
}

// Initialize GUI components
void initGUI() {
  // Initialize robot status array
  for (int i = 0; i < 36; i++) {
    robotStatus[i].targetID = i;
    robotStatus[i].ackReceived = true;
    robotStatus[i].currentX = 0.0;
    robotStatus[i].currentY = 0.0;
    robotStatus[i].currentAngle = 0.0;
    robotStatus[i].timestamp = 0;
    robotStatus[i].batteryVoltage = 0.0;
  }
  
  // Setup WebSocket
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  
  // Setup web server routes
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });
  
  server.begin();
  Serial.println("Web server started");
}

// FreeRTOS GUI Task
void guiTask(void *parameter) {
  Serial.println("GUI Task started");
  
  GUIStatus status;
  
  while (1) {
    // Check for status updates from communicator
    if (xQueueReceive(statusQueue, &status, pdMS_TO_TICKS(10)) == pdPASS) {
      // Update robot status array
      if (status.targetID < 36) {
        robotStatus[status.targetID] = status;
        
        // Only send if we have connected clients and queue isn't full
        if (ws.count() > 0) {
          // Broadcast to all WebSocket clients
          String json = "{\"id\":" + String(status.targetID) + 
                       ",\"x\":" + String(status.currentX, 2) +
                       ",\"y\":" + String(status.currentY, 2) +
                       ",\"angle\":" + String(status.currentAngle, 3) +
                       ",\"ts\":" + String(status.timestamp) +
                       ",\"batt\":" + String(status.batteryVoltage, 2) +
                       ",\"ack\":" + String(status.ackReceived ? "true" : "false") + "}";
          
          // Try to send, but don't block if queue is full
          ws.textAll(json);
          
          // Small delay to prevent overwhelming the WebSocket
          vTaskDelay(pdMS_TO_TICKS(5));
        }
        
        if (status.ackReceived) {
          Serial.printf("Status update for robot 0x%02X: (%.2f, %.2f) %.3frad %.2fV\n",
                       status.targetID, status.currentX, status.currentY, 
                       status.currentAngle, status.batteryVoltage);
        } else {
          Serial.printf("ACK timeout for robot 0x%02X\n", status.targetID);
        }
      }
    }
    
    // Clean up WebSocket clients
    ws.cleanupClients();
    
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
