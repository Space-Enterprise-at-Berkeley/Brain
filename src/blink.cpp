#include <Arduino.h>

void setup() {
  Serial.begin(9600);
  pinMode(17, OUTPUT);
}

void loop() {
  digitalWrite(17, HIGH);
  delay(1000);
  digitalWrite(17, LOW);
  delay(1000);
  Serial.println("test");
}
