
#define M0_STEP 5
#define M0_DIR 4
#define M0_EN 6
#define M0_RST 7
#define M1_STEP 1
#define M1_DIR 10
#define M1_EN 0
#define M1_RST 3


void setup() {
  Serial.begin(115200);

  pinMode(M0_STEP, OUTPUT);
  pinMode(M0_DIR, OUTPUT);
  pinMode(M0_EN, OUTPUT);
  pinMode(M0_RST, OUTPUT);
  pinMode(M1_STEP, OUTPUT);
  pinMode(M1_DIR, OUTPUT);
  pinMode(M1_EN, OUTPUT);
  pinMode(M1_RST, OUTPUT);

  digitalWrite(M0_STEP, LOW);
  digitalWrite(M0_DIR, LOW);
  digitalWrite(M0_EN, LOW);
  digitalWrite(M0_RST, LOW);
  digitalWrite(M1_STEP, LOW);
  digitalWrite(M1_DIR, LOW);
  digitalWrite(M1_EN, LOW);
  digitalWrite(M1_RST, LOW);

  // Set microstepping to 1/256
  // digitalWrite(M0_STEP, LOW);
  // digitalWrite(M0_DIR, LOW);
  // digitalWrite(M1_STEP, LOW);
  // digitalWrite(M1_DIR, LOW);

  // Set microstepping to 1/8
  digitalWrite(M0_STEP, HIGH);
  digitalWrite(M0_DIR, LOW);
  digitalWrite(M1_STEP, HIGH);
  digitalWrite(M1_DIR, LOW);

  // Startup
  delay(20);
  digitalWrite(M0_RST, HIGH);
  digitalWrite(M1_RST, HIGH);

  delay(100);
  digitalWrite(M0_EN, HIGH);
  digitalWrite(M1_EN, HIGH);
}

void loop() {
  uint16_t stepdelay = 1200;
  digitalWrite(M0_DIR, LOW);
  digitalWrite(M1_DIR, LOW);
  for (uint16_t i = 0; i < 500; i++)
  {
    digitalWrite(M0_STEP, HIGH);
    digitalWrite(M1_STEP, HIGH);
    delayMicroseconds(stepdelay);
    digitalWrite(M0_STEP, LOW);
    digitalWrite(M1_STEP, LOW);
    delayMicroseconds(stepdelay);
  }

  digitalWrite(M0_EN, LOW);
  digitalWrite(M1_EN, LOW);
  delay(2000);
  digitalWrite(M0_EN, HIGH);
  digitalWrite(M1_EN, HIGH);

  digitalWrite(M0_DIR, HIGH);
  digitalWrite(M1_DIR, HIGH);
  for (uint16_t i = 0; i < 500; i++)
  {
    digitalWrite(M0_STEP, HIGH);
    digitalWrite(M1_STEP, HIGH);
    delayMicroseconds(stepdelay);
    digitalWrite(M0_STEP, LOW);
    digitalWrite(M1_STEP, LOW);
    delayMicroseconds(stepdelay);
  }

  digitalWrite(M0_EN, LOW);
  digitalWrite(M1_EN, LOW);
  delay(2000);
  digitalWrite(M0_EN, HIGH);
  digitalWrite(M1_EN, HIGH);

  digitalWrite(M0_DIR, HIGH);
  digitalWrite(M1_DIR, LOW);
  for (uint16_t i = 0; i < 500; i++)
  {
    digitalWrite(M0_STEP, HIGH);
    digitalWrite(M1_STEP, HIGH);
    delayMicroseconds(stepdelay);
    digitalWrite(M0_STEP, LOW);
    digitalWrite(M1_STEP, LOW);
    delayMicroseconds(stepdelay);
  }

  digitalWrite(M0_EN, LOW);
  digitalWrite(M1_EN, LOW);
  delay(2000);
  digitalWrite(M0_EN, HIGH);
  digitalWrite(M1_EN, HIGH);

  // GPIO.out_w1ts = ((uint32_t)1 << 22);
}
