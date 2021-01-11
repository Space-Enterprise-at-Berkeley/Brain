#include "interfaces/GPIOutput.h"

GPIOutput::GPIOutput(uint8_t pin) : pin(pin) {
  pinMode(pin, OUTPUT);
}

bool GPIOutput::set_output(uint8_t channel, uint8_t state) {
  if(channel != 0 || state > 1) return false;
  digitalWriteFast(pin, state);
  return true;
}
