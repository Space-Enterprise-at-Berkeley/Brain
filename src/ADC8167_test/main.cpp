
#include <Arduino.h>
#include <SPI.h>
#include <ADS8167.h>
#include <Wire.h>

//ADC 1
// uint8_t cs_pin = 37;
// uint8_t rdy_pin = 26;
// uint8_t alrt_pin = 9;

//ADC 2
uint8_t cs_pin = 36;
uint8_t rdy_pin = 27;
uint8_t alrt_pin = 10;

ADS8167 adc(&SPI, cs_pin, rdy_pin, alrt_pin);

void setup() {
  Serial.begin(9600);
  delay(1000);

  adc.init();
  adc.setManualMode();
  adc.setAllInputsSeparate();
  pinMode(rdy_pin, INPUT_PULLUP);
}

void loop() {
  for (int i = 0; i < 8; i ++){
    Serial.print("A");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(adc.readData(i));
  }
  delay(1000);
}