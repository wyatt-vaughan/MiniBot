#include "esp_now_communicator.h"

// Static variables for task state
static MotionQueue* comm_queue = NULL;
static EspNowReceiveCallback receive_callback = nullptr;

// Forward declaration of the ESP-NOW receive callback
static void esp_now_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len);

// Message parsing callback - processes received messages and enqueues commands
static void esp_now_message_handler(const uint8_t *mac_addr, const uint8_t *data, int len);

bool EspNowCommunicator_Init(MotionQueue* motion_queue) {
    if (motion_queue == NULL) {
        return false;
    }
    
    comm_queue = motion_queue;
    
    // Set up WiFi radio, register callbacks for received messages
    WiFi.mode(WIFI_STA);
    WiFi.setTxPower(WIFI_POWER);
    esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    Serial.printf("WiFi started on channel %d\n", WiFi.channel());

    // ESP-NOW Setup
    if (esp_now_init() != ESP_OK) {
        Serial.println("ERROR: ESP-NOW initialization failed");
        return false;
    }
    
    // Register the receive callback
    esp_now_register_recv_cb(esp_now_recv_cb);
    
    // Add a broadcast peer (MAC address FF:FF:FF:FF:FF:FF)
    esp_now_peer_info_t peer = {};
    memset(&peer, 0, sizeof(esp_now_peer_info_t));
    memset(peer.peer_addr, 0xFF, ESP_NOW_ETH_ALEN);  // Broadcast address
    peer.channel = WIFI_CHANNEL;
    
    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("WARNING: Failed to add broadcast peer (may already exist)");
    }
    
    // Register the message parsing handler
    EspNowCommunicator_RegisterCallback(esp_now_message_handler);
    
    return true;
}

/**
 * Internal ESP-NOW receive callback - this will be called by the ESP-NOW stack
 */
static void esp_now_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len) {
    if (mac_addr == NULL || data == NULL) {
        return;
    }
    
    // Call the registered user callback if one exists
    if (receive_callback) {
        receive_callback(mac_addr, data, len);
    }
}

/**
 * Message parsing callback - checks destination address and message type
 * Enqueues appropriate commands to the motion queue
 */
static void esp_now_message_handler(const uint8_t *mac_addr, const uint8_t *data, int len) {
    if (len < 2 || comm_queue == NULL) {
        return;
    }
    
    // Extract destination address and message type from first 2 bytes
    uint8_t target_id = data[0];
    uint8_t msg_type = data[1];
    
    // Check if this message is for us
    if (target_id != DEVICE_ID) {
        Serial.printf("Message not for us (target: 0x%02X, our ID: 0x%02X), ignoring\n", target_id, DEVICE_ID);
        return;
    }
    
    Serial.printf("Received message for us (type: %d, len: %d)\n", msg_type, len);
    
    switch (msg_type) {
        case MSG_TYPE_POSITION_COMMAND: {
            // Check message length
            if (len < sizeof(PositionCommand)) {
                Serial.printf("ERROR: PositionCommand message too short (got %d, need %zu)\n", len, sizeof(PositionCommand));
                return;
            }
            
            // Cast and parse the message
            PositionCommand* cmd = (PositionCommand*)data;
            
            Serial.printf("PositionCommand: target=(%f, %f), angle=%f rad, duration=%f ms\n",
                         cmd->target_x_mm, cmd->target_y_mm, cmd->target_a_rad, cmd->move_duration_ms);
            
            // Create a MotionCommand from the PositionCommand
            MotionCommand motion_cmd = MotionCommand{
                cmd->target_x_mm,
                cmd->target_y_mm,
                cmd->target_a_rad,
                cmd->move_duration_ms,
                NULL
            };
            
            // Enqueue the command
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
            Serial.println("Received PositionRequest (not yet implemented)");
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
    // Extract robot pointer from task parameters
    Robot* robot = (Robot*)pvParameters;
    
    // Task initialization
    if (comm_queue == NULL || robot == NULL) {
        vTaskDelete(NULL);
        return;
    }
    
    // TEST LOOP
    MotionCommand testmc0 = MotionCommand{100.0f, 0.0f, 0.0f, 2000, NULL};
    MotionCommand testmc1 = MotionCommand{20.0f, 0.0f, 2.0f, 1000, NULL};
    MotionCommand testmc2 = MotionCommand{30.0f, 30.0f, 0.0f, 1000, NULL};
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
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        if (MotionQueue_Enqueue(comm_queue, &testmc3))
            Serial.println("Enqueued testmc3");
        else
            Serial.println("Failed to enqueue testmc3");
        Serial.println(MotionQueue_GetSize(comm_queue));
        vTaskDelay(pdMS_TO_TICKS(3000));

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
