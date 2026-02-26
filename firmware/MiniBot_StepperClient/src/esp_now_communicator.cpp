#include "esp_now_communicator.h"
#include "device_id.h"
#include "position_estimator.h"

static MotionQueue comm_queue = NULL;
static EspNowReceiveCallback receive_callback = nullptr;
static Robot* g_robot_ptr = NULL;
static uint8_t last_sender_mac[6] = {0};
static bool has_pending_completion_ack = false;

static void esp_now_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len);
static void esp_now_message_handler(const uint8_t *mac_addr, const uint8_t *data, int len);
static bool ensure_peer_exists(const uint8_t *mac_addr);
static bool send_ack_message(const uint8_t *mac_addr);
static void send_completion_ack_if_pending();

bool EspNowCommunicator_Init(MotionQueue motion_queue) {
    if (motion_queue == NULL) {
        return false;
    }
    
    comm_queue = motion_queue;
    
    WiFi.mode(WIFI_STA);
    delay(100);
    Serial.printf("WiFi MAC: %s\n", WiFi.macAddress().c_str());
    
    if (esp_now_init() != ESP_OK) {
        Serial.println("ERROR: ESP-NOW initialization failed");
        return false;
    }
    
    esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_now_register_recv_cb(esp_now_recv_cb);
    
    esp_now_peer_info_t peer = {};
    memset(&peer, 0, sizeof(esp_now_peer_info_t));
    memset(peer.peer_addr, 0xFF, ESP_NOW_ETH_ALEN);
    peer.channel = WIFI_CHANNEL;
    
    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("WARNING: Failed to add broadcast peer");
    }
    
    EspNowCommunicator_RegisterCallback(esp_now_message_handler);
    Serial.println("ESP-NOW ready");
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

static bool ensure_peer_exists(const uint8_t *mac_addr) {
    if (mac_addr == NULL) {
        return false;
    }
    
    if (!esp_now_is_peer_exist(mac_addr)) {
        esp_now_peer_info_t peer_info = {};
        memcpy(peer_info.peer_addr, mac_addr, ESP_NOW_ETH_ALEN);
        peer_info.channel = WIFI_CHANNEL;
        peer_info.encrypt = false;
        
        esp_err_t add_result = esp_now_add_peer(&peer_info);
        if (add_result != ESP_OK) {
            Serial.printf("ERROR: Failed to add peer (error: %d)\n", add_result);
            return false;
        }
    }
    return true;
}

static bool send_ack_message(const uint8_t *mac_addr) {
    if (mac_addr == NULL || g_robot_ptr == NULL) {
        return false;
    }
    
    float x, y, theta;
    g_robot_ptr->getPosition(&x, &y, &theta);
    
    AckMessage ack = {};
    ack.responderID = getDeviceID();
    ack.msg_type = MSG_TYPE_ACK_MESSAGE;
    ack.timestamp = millis();
    ack.x = x;
    ack.y = y;
    ack.orientation_rad = theta;
    ack.battery_voltage = g_robot_ptr->getBatteryVoltage();
    
    if (!ensure_peer_exists(mac_addr)) {
        return false;
    }
    
    esp_err_t result = esp_now_send(mac_addr, (uint8_t*)&ack, sizeof(AckMessage));
    if (result != ESP_OK) {
        Serial.printf("ERROR: Failed to send ack (err: %d)\n", result);
        return false;
    }
    return true;
}

static bool send_nack_message(const uint8_t *mac_addr, const uint8_t error_type) {
    if (mac_addr == NULL || g_robot_ptr == NULL) {
        return false;
    }
    
    float x, y, theta;
    g_robot_ptr->getPosition(&x, &y, &theta);
    
    NackMessage nack = {};
    nack.responderID = getDeviceID();
    nack.msg_type = MSG_TYPE_NACK_MESSAGE;
    nack.timestamp = millis();
    nack.err_type = error_type;
    
    if (!ensure_peer_exists(mac_addr)) {
        return false;
    }
    
    esp_err_t result = esp_now_send(mac_addr, (uint8_t*)&nack, sizeof(NackMessage));
    if (result != ESP_OK) {
        Serial.printf("ERROR: Failed to send nack (err: %d)\n", result);
        return false;
    }
    return true;
}

static bool send_mag_field_response(const uint8_t *mac_addr) {
    if (mac_addr == NULL) {
        return false;
    }
    
    float x, y, z;
    if (!PositionEstimator_GetLatestMagneticField(&x, &y, &z)) {
        Serial.println("ERROR: Could not retrieve magnetometer readings");
        return send_nack_message(mac_addr, ERR_ROBOT_UNAVAILABLE);
    }
    
    MagneticFieldResponse response = {};
    response.responderID = getDeviceID();
    response.msg_type = MSG_TYPE_MAG_REQUEST;
    response.timestamp = millis();
    response.field_x_gauss = x;
    response.field_y_gauss = y;
    response.field_z_gauss = z;
    
    if (!ensure_peer_exists(mac_addr)) {
        return false;
    }
    
    esp_err_t result = esp_now_send(mac_addr, (uint8_t*)&response, sizeof(MagneticFieldResponse));
    if (result != ESP_OK) {
        Serial.printf("ERROR: Failed to send mag field response (err: %d)\n", result);
        return false;
    }
    return true;
}

static void send_completion_ack_if_pending() {
    if (has_pending_completion_ack) {
        Serial.println("Sending motion completion acknowledgment");
        send_ack_message(last_sender_mac);
        has_pending_completion_ack = false;
    }
}

static void esp_now_message_handler(const uint8_t *mac_addr, const uint8_t *data, int len) {
    if (len < 2 || comm_queue == NULL) {
        return;
    }
    
    uint8_t target_id = data[0];
    uint8_t msg_type = data[1];
    
    if (target_id != getDeviceID()) {
        return;
    }
    
    switch (msg_type) {
        case MSG_TYPE_POSITION_COMMAND: {
            if (len < sizeof(PositionCommand)) {
                Serial.printf("ERROR: PositionCommand too short\n");
                send_nack_message(mac_addr, ERR_INVALID_MSG_SIZE);
                return;
            }
            
            PositionCommand* cmd = (PositionCommand*)data;
            
            MotionCommand motion_cmd = {
                cmd->target_x_mm,
                cmd->target_y_mm,
                cmd->target_a_rad,
                cmd->move_duration_ms
            };
            
            if (MotionQueue_Enqueue(comm_queue, &motion_cmd)) {
                // Store sender MAC for completion ack
                memcpy(last_sender_mac, mac_addr, 6);
                has_pending_completion_ack = true;
                send_ack_message(mac_addr);
            } else {
                Serial.println("ERROR: Motion queue full");
                send_nack_message(mac_addr, ERR_QUEUE_FULL);
            }
            break;
        }
        
        case MSG_TYPE_MOT_TEST_COMMAND: {
            if (len < sizeof(MotTestCommand)) {
                Serial.println("ERROR: MotTestCommand too short");
                return;
            }
            Serial.println("Received motor test command");
            send_nack_message(mac_addr, ERR_NOT_IMPLEMENTED);
            break;
        }
        
        case MSG_TYPE_POSITION_REQUEST: {
            if (len < sizeof(PositionRequest)) {
                Serial.println("ERROR: PositionRequest too short");
                send_nack_message(mac_addr, ERR_INVALID_MSG_SIZE);
                return;
            }
            
            if (g_robot_ptr == NULL) {
                Serial.println("ERROR: Robot pointer unavailable");
                send_nack_message(mac_addr, ERR_ROBOT_UNAVAILABLE);
                return;
            }
            
            send_ack_message(mac_addr);
            break;
        }
        
        case MSG_TYPE_ACK_MESSAGE: {
            if (len < sizeof(AckMessage)) {
                Serial.println("ERROR: AckMessage too short");
                send_nack_message(mac_addr, ERR_INVALID_MSG_SIZE);
                return;
            }
            Serial.println("Received acknowledgment message");
            send_nack_message(mac_addr, ERR_NOT_IMPLEMENTED);
            break;
        }
        
        case MSG_TYPE_MAG_REQUEST: {
            if (len < sizeof(MagneticFieldRequest)) {
                Serial.println("ERROR: MagneticFieldRequest too short");
                send_nack_message(mac_addr, ERR_INVALID_MSG_SIZE);
                return;
            }
            Serial.println("Received magnetometer field request");
            send_mag_field_response(mac_addr);
            break;
        }
        
        default:
            send_nack_message(mac_addr, ERR_UNKNOWN_MSG);
            break;
    }
}

bool EspNowCommunicator_RegisterCallback(EspNowReceiveCallback callback) {
    receive_callback = callback;
    return true;
}

bool EspNowCommunicator_SendAlert(uint8_t error_type) {
    // Only send alert if we have a valid last sender
    // Check if the MAC address is not all zeros
    bool has_valid_sender = false;
    for (int i = 0; i < 6; i++) {
        if (last_sender_mac[i] != 0) {
            has_valid_sender = true;
            break;
        }
    }
    
    if (!has_valid_sender) {
        Serial.println("WARNING: No valid sender MAC for alert");
        return false;
    }
    
    return send_nack_message(last_sender_mac, error_type);
}

void EspNowCommunicator_Task(void* pvParameters) {
    Robot* robot = (Robot*)pvParameters;
    
    if (comm_queue == NULL || robot == NULL) {
        vTaskDelete(NULL);
        return;
    }
    
    g_robot_ptr = robot;

    while (1) {
        if (has_pending_completion_ack && !g_robot_ptr->isMoving()) {
            send_completion_ack_if_pending();
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
