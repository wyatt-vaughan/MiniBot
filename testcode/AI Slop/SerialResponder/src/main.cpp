#include <Arduino.h>

void setup() {
  Serial.begin(500000);
  Serial.println("READY");
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'T' || c == 't') {
      char buf[22];
      snprintf(buf, sizeof(buf), "T:%lu", micros());
      Serial.println(buf);
    }
    // all other bytes (newlines, carriage returns, etc.) are ignored
  }
}