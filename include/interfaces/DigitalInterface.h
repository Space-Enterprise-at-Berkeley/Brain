#pragma once
#include <Arduino.h>

class DigitalInterface {
  public:
    DigitalInterface(){};
    virtual bool set_output(uint8_t channel, uint8_t state) = 0;
    virtual uint8_t get_input(uint8_t channel) = 0;
};
