
#include <Arduino.h>
#include <SPI.h>
#include <LTC4151.h>
#include <Wire.h>

LTC4151 sensor;


void setup() {
  Serial.begin(9600);
  delay(1000);

  Wire1.begin();

  sensor.init(LTC4151::F, LTC4151::F, &Wire1);
}

void loop() {
  Serial.print("O");
  Serial.print(": ");
  Serial.print(sensor.getLoadCurrent(0.02));
  Serial.print("A ");
  // Serial.print(sensor.getSnapshotLoadCurrent(0.02));
  // Serial.print("A ");
  Serial.print(sensor.getInputVoltage());
  // Serial.print("V ");
  // Serial.print(sensor.getSnapshotInputVoltage());
  Serial.print("V ");
  Serial.print(sensor.getADCInVoltage());
  Serial.print("V ");
  // Serial.print(sensor.getSnapshotADCInVoltage());
  // Serial.print("V ");
  Serial.println();
  delay(10);
}
