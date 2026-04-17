const uint8_t npins = 5;
const uint8_t ledpins[npins] = {27, 33, 25, 26, 14};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(250000);
  for (uint8_t i = 0; i <= npins; i++)
  {
    pinMode(ledpins[i], OUTPUT);
    digitalWrite(ledpins[i], LOW);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(3000);
  for (uint8_t i = 0; i <= npins; i++)
  {
    Serial.println(i);
    digitalWrite(ledpins[i], HIGH);
    delay(1000);
  }
  delay(500);
  for (uint8_t i = 0; i <= npins; i++)
  {
    digitalWrite(ledpins[i], LOW);
  }
  delay(10000);
}
