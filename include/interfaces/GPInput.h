#pragma once
#include "DigitalInterface.h"

#include <Arduino.h>

class GPInput : public DigitalInterface {
  private:
    uint8_t pin;
    bool set_output(uint8_t channel, uint8_t state) override { return 0; };
  public:
    /**
     * Construct a GPIO input object.
     * 
     * @param pin The physical pin to connect to
     * @param pullup Whether this input pin should have a pullup to 3.3 volts
     */
    GPInput(uint8_t pin, bool pullup = true);
    uint8_t get_input(uint8_t channel) override;
};
