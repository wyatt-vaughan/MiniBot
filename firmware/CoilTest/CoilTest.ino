#define IN1 4
#define IN2 5

const int delay_ms=200;

void setup() {
  pinMode(2, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
}

void loop() {
  digitalWrite(2, HIGH);
  // digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  delay(delay_ms);
  digitalWrite(2, LOW);
  // digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  delay(delay_ms);
}
