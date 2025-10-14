#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

#define DEVICE_ID 0x01
#define MOTOR_SLEEP_PIN 4
#define MOTOR_CTRL_A 6
#define MOTOR_CTRL_B 5
#define MOTOR_PWM_CH 0
#define MOTOR_PWM_FREQ 5000
#define MOTOR_PWM_RES 8
#define MOTOR_PWM_PIN MOTOR_CTRL_A

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

MotorCommand incoming;

void setupMotor() {
  pinMode(MOTOR_SLEEP_PIN, OUTPUT);
  pinMode(MOTOR_CTRL_A, OUTPUT);
  pinMode(MOTOR_CTRL_B, OUTPUT);

  ledcAttachChannel(MOTOR_CTRL_A, MOTOR_PWM_FREQ, MOTOR_PWM_RES, MOTOR_PWM_CH);
}

void updateMotorState() {
  if (!incoming.enabled) {
    digitalWrite(MOTOR_SLEEP_PIN, LOW);
    digitalWrite(MOTOR_CTRL_A, LOW);
    digitalWrite(MOTOR_CTRL_B, LOW);
    return;
  }

  digitalWrite(MOTOR_SLEEP_PIN, HIGH);

  if (incoming.forward) {
    digitalWrite(MOTOR_CTRL_B, LOW);
    ledcWrite(MOTOR_PWM_CH, incoming.speed);
  } else {
    digitalWrite(MOTOR_CTRL_B, HIGH);
    ledcWrite(MOTOR_PWM_CH, 0); // Not used in reverse
    digitalWrite(MOTOR_CTRL_A, LOW);
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

void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  if (len == sizeof(MotorCommand)) {
    memcpy(&incoming, incomingData, len);
    if (incoming.targetID == DEVICE_ID) {
      updateMotorState();
      sendAck(recv_info->src_addr);
    }
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  esp_wifi_set_max_tx_power(40);

  setupMotor();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(onDataRecv);
}

void loop() {
  // Nothing here — all handled via callbacks
  // delay(50);
  // esp_sleep_enable_timer_wakeup(1000000);  // Sleep for 1 second (1,000,000 us)
  // esp_light_sleep_start();
}
