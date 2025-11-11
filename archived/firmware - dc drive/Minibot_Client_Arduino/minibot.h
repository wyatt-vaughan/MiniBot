class MiniBot_DC {
public:
  MiniBot_DC(uint8_t en, uint8_t m0_A, uint8_t m0_B, uint8_t m1_A, uint8_t m1_B) {
    en_pin = en;
    m0_A_pin = m0_A;
    m0_B_pin = m0_B;
    m1_A_pin = m1_A;
    m1_B_pin = m1_B;
  }

  void initialize() {
    pinMode(en_pin, OUTPUT);
    pinMode(m0_A_pin, OUTPUT);
    pinMode(m0_B_pin, OUTPUT);
    pinMode(m1_A_pin, OUTPUT);
    pinMode(m1_B_pin, OUTPUT);

    ledcAttachChannel(m0_A_pin, MOTOR_PWM_FREQ, MOTOR_PWM_RES, 1);
    ledcAttachChannel(m0_B_pin, MOTOR_PWM_FREQ, MOTOR_PWM_RES, 2);
    ledcAttachChannel(m1_A_pin, MOTOR_PWM_FREQ, MOTOR_PWM_RES, 3);
    ledcAttachChannel(m1_B_pin, MOTOR_PWM_FREQ, MOTOR_PWM_RES, 4);

    disableMotors();
  }

  void disableMotors() {
    digitalWrite(en_pin, LOW);
    ledcWrite(m0_A_pin, 0);
    ledcWrite(m0_B_pin, 0);
    ledcWrite(m1_A_pin, 0);
    ledcWrite(m1_B_pin, 0);
  }

  void MotorDebug(bool en, int8_t m0_vel, int8_t m1_vel) {
    if (!en) {
      Serial.println("DISABLING MOTORS");
      disableMotors();
      return;
    }

    if (m0_vel >= 0) {
      Serial.println("M0 FORWARDS");
      ledcWrite(m0_A_pin, 2 * m0_vel);
      ledcWrite(m0_B_pin, 0);
    } else {
      Serial.println("M0 BACKWORDS");
      ledcWrite(m0_A_pin, 0);
      ledcWrite(m0_B_pin, -2 * m0_vel);
    }

    if (m1_vel >= 0) {
      Serial.println("M1 FORWARDS");
      ledcWrite(m1_A_pin, 2 * m1_vel);
      ledcWrite(m1_B_pin, 0);
    } else {
      Serial.println("M1 BACKWORDS");
      ledcWrite(m1_A_pin, 0);
      ledcWrite(m1_B_pin, -2 * m1_vel);
    }

    digitalWrite(en_pin, HIGH);
  }

  void updateTarget(float xtar, float ytar) {
    tar_x = xtar;
    tar_y = ytar;
  }

  void getPosition(float& xpos, float& ypos) {
    xpos = pos_x;
    ypos = pos_y;
  }

  
private:
  uint8_t en_pin;
  uint8_t m0_A_pin;
  uint8_t m0_B_pin;
  uint8_t m1_A_pin;
  uint8_t m1_B_pin;

  uint16_t MOTOR_PWM_FREQ = 500;
  uint8_t MOTOR_PWM_RES = 8;
  
  float pos_x;
  float pos_y;
  float tar_x;
  float tar_y;

};

class MiniBot_STEP {
public:
private:
};