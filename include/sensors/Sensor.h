#pragma once
#include <Arduino.h>

class Sensor {
  public:
    Sensor() {};
    virtual float read(uint8_t channel);
};
