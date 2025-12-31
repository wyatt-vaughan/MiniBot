#include "esp_now_communicator.h"

static MotionQueue* comm_queue = NULL;
static EspNowReceiveCallback receive_callback = nullptr;
static Robot* g_robot_ptr = NULL;

static void esp_now_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len);
static void esp_now_message_handler(const uint8_t *mac_addr, const uint8_t *data, int len);

bool EspNowCommunicator_Init(MotionQueue* motion_queue) {
    if (motion_queue == NULL) {
        return false;
    }
    
    comm_queue = motion_queue;
    
    Serial.println("Setting up WiFi...");
    WiFi.mode(WIFI_STA);
    delay(100);
    Serial.printf("WiFi MAC: %s\n", WiFi.macAddress().c_str());
    
    Serial.println("Initializing ESP-NOW...");
    if (esp_now_init() != ESP_OK) {
        Serial.println("ERROR: ESP-NOW initialization failed");
        return false;
    }
    Serial.println("ESP-NOW initialized successfully");
    
    // Set channel after ESP-NOW init
    esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    Serial.printf("WiFi channel set to %d\n", WiFi.channel());
    
    Serial.println("Registering ESP-NOW callback...");
    esp_now_register_recv_cb(esp_now_recv_cb);
    
    Serial.println("Adding broadcast peer...");
    esp_now_peer_info_t peer = {};
    memset(&peer, 0, sizeof(esp_now_peer_info_t));
    memset(peer.peer_addr, 0xFF, ESP_NOW_ETH_ALEN);
    peer.channel = WIFI_CHANNEL;
    
    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("WARNING: Failed to add broadcast peer (may already exist)");
    }
    
    Serial.println("Registering message handler...");
    EspNowCommunicator_RegisterCallback(esp_now_message_handler);
    
    Serial.println("ESP-NOW setup complete");
    return true;
}

static void esp_now_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len) {
    if (mac_addr == NULL || data == NULL) {
        return;
    }
    
    if (receive_callback) {
        receive_callback(mac_addr, data, len);
    }
}

static void esp_now_message_handler(const uint8_t *mac_addr, const uint8_t *data, int len) {
    if (len < 2 || comm_queue == NULL) {
        return;
    }
    
    uint8_t target_id = data[0];
    uint8_t msg_type = data[1];
    
    if (target_id != DEVICE_ID) {
        Serial.printf("Message not for us (target: 0x%02X, our ID: 0x%02X), ignoring\n", target_id, DEVICE_ID);
        return;
    }
    
    Serial.printf("Received message for us (type: %d, len: %d)\n", msg_type, len);
    
    switch (msg_type) {
        case MSG_TYPE_POSITION_COMMAND: {
            if (len < sizeof(PositionCommand)) {
                Serial.printf("ERROR: PositionCommand message too short (got %d, need %zu)\n", len, sizeof(PositionCommand));
                return;
            }
            
            PositionCommand* cmd = (PositionCommand*)data;
            
            Serial.printf("PositionCommand: target=(%f, %f), angle=%f rad, duration=%f ms\n",
                         cmd->target_x_mm, cmd->target_y_mm, cmd->target_a_rad, cmd->move_duration_ms);
            
            MotionCommand motion_cmd = MotionCommand{
                cmd->target_x_mm,
                cmd->target_y_mm,
                cmd->target_a_rad,
                cmd->move_duration_ms,
                NULL
            };
            
            if (MotionQueue_Enqueue(comm_queue, &motion_cmd)) {
                Serial.println("  Successfully enqueued PositionCommand");
            } else {
                Serial.println("  ERROR: Failed to enqueue PositionCommand (queue full?)");
            }
            break;
        }
        
        case MSG_TYPE_MOT_TEST_COMMAND: {
            if (len < sizeof(MotTestCommand)) {
                Serial.printf("ERROR: MotTestCommand message too short (got %d, need %zu)\n", len, sizeof(MotTestCommand));
                return;
            }
            Serial.println("Received MotTestCommand (not yet implemented)");
            break;
        }
        
        case MSG_TYPE_POSITION_REQUEST: {
            if (len < sizeof(PositionRequest)) {
                Serial.printf("ERROR: PositionRequest message too short (got %d, need %zu)\n", len, sizeof(PositionRequest));
                return;
            }
            
            PositionRequest* req = (PositionRequest*)data;
            Serial.printf("Received PositionRequest (timestamp: %u)\n", req->timestamp);
            
            if (g_robot_ptr == NULL) {
                Serial.println("ERROR: Robot pointer not available");
                return;
            }
            
            float x, y, theta;
            g_robot_ptr->getPosition(&x, &y, &theta);
            
            AckMessage ack = {};
            ack.responderID = DEVICE_ID;
            ack.msg_type = MSG_TYPE_ACK_MESSAGE;
            ack.timestamp = millis();
            ack.x = x;
            ack.y = y;
            ack.orientation_rad = theta;
            ack.battery_voltage = g_robot_ptr->getBatteryVoltage();
            
            esp_err_t result = esp_now_send(mac_addr, (uint8_t*)&ack, sizeof(AckMessage));
            if (result == ESP_OK) {
                Serial.printf("Sent AckMessage to sender: pos=(%.2f, %.2f), angle=%.2f rad, battery=%.2f V\n",
                             ack.x, ack.y, ack.orientation_rad, ack.battery_voltage);
            } else {
                Serial.printf("ERROR: Failed to send AckMessage (error: %d)\n", result);
            }
            break;
        }
        
        case MSG_TYPE_ACK_MESSAGE: {
            if (len < sizeof(AckMessage)) {
                Serial.printf("ERROR: AckMessage message too short (got %d, need %zu)\n", len, sizeof(AckMessage));
                return;
            }
            Serial.println("Received AckMessage (not yet implemented)");
            break;
        }
        
        default:
            Serial.printf("Unknown message type: %d\n", msg_type);
            break;
    }
}

bool EspNowCommunicator_RegisterCallback(EspNowReceiveCallback callback) {
    receive_callback = callback;
    return true;
}

void EspNowCommunicator_Task(void* pvParameters) {
    Robot* robot = (Robot*)pvParameters;
    
    if (comm_queue == NULL || robot == NULL) {
        vTaskDelete(NULL);
        return;
    }
    
    g_robot_ptr = robot;
    
    // TEST LOOP
    MotionCommand testmc0 = MotionCommand{10.0f, 0.0f, 0.0f, 2000, NULL};
    MotionCommand testmc1 = MotionCommand{0.0f, 0.0f, 0.0f, 1000, NULL};
    MotionCommand testmc2 = MotionCommand{0.0f, 0.0f, 1.0f, 1000, NULL};
    MotionCommand testmc3 = MotionCommand{0.0f, 0.0f, 0.0f, 2000, NULL};
    randomSeed(esp_random());
    vTaskDelay(pdMS_TO_TICKS(2000));
    while (1) {
        Serial.println("ESP-NOW Communicator Task loop started");
        
        // FOR TEST ONLY, SIMULATE RECEIVED COMMANDS
        if (MotionQueue_Enqueue(comm_queue, &testmc0))
            Serial.println("Enqueued testmc0");
        else
            Serial.println("Failed to enqueue testmc0");
        Serial.println(MotionQueue_GetSize(comm_queue));
        vTaskDelay(pdMS_TO_TICKS(1));
        
        if (MotionQueue_Enqueue(comm_queue, &testmc1))
            Serial.println("Enqueued testmc1");
        else
            Serial.println("Failed to enqueue testmc1");
        Serial.println(MotionQueue_GetSize(comm_queue));
        vTaskDelay(pdMS_TO_TICKS(1));

        if (MotionQueue_Enqueue(comm_queue, &testmc2))
            Serial.println("Enqueued testmc2");
        else
            Serial.println("Failed to enqueue testmc2");
        Serial.println(MotionQueue_GetSize(comm_queue));
        vTaskDelay(pdMS_TO_TICKS(1));

        if (MotionQueue_Enqueue(comm_queue, &testmc1))
            Serial.println("Enqueued testmc1");
        else
            Serial.println("Failed to enqueue testmc1");
        Serial.println(MotionQueue_GetSize(comm_queue));
        vTaskDelay(pdMS_TO_TICKS(8000));

        // if (MotionQueue_Enqueue(comm_queue, &testmc2))
        //     Serial.println("Enqueued testmc2");
        // else
        //     Serial.println("Failed to enqueue testmc2");
        // Serial.println(MotionQueue_GetSize(comm_queue));
        // vTaskDelay(pdMS_TO_TICKS(10));

        // MotionCommand testmcRAND = MotionCommand{(float)random(0, 101), (float)random(0, 101), (float)random(0, 100) / 20, 5000, NULL};
        // if (MotionQueue_Enqueue(comm_queue, &testmcRAND))
        //     Serial.println("Enqueued testmcRAND");
        // else
        //     Serial.println("Failed to enqueue testmcRAND");
        // Serial.println(MotionQueue_GetSize(comm_queue));
        // vTaskDelay(pdMS_TO_TICKS(10000));

        // if (MotionQueue_Enqueue(comm_queue, &testmc3))
        //     Serial.println("Enqueued testmc3");
        // else
        //     Serial.println("Failed to enqueue testmc3");
        // Serial.println(MotionQueue_GetSize(comm_queue));
        // vTaskDelay(pdMS_TO_TICKS(5000));
        
    }
}
