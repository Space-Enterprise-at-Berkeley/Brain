#pragma once
#include "DigitalInterface.h"

#include <Arduino.h>

class GPIOutput : public DigitalInterface {
  private:
    uint8_t pin;
    uint8_t get_input(uint8_t channel) override { return 0; };
  public:
    /**
     * Construct a GPIO output object.
     * 
     * @param pin The physical pin to connect to
     */
    GPIOutput(uint8_t pin);
    bool set_output(uint8_t channel, uint8_t state) override;
};
