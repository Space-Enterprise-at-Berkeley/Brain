#include "interfaces/GPInput.h"

GPInput::GPInput(uint8_t pin, bool pullup) : pin(pin) {
  if(pullup) {
    pinMode(pin, INPUT_PULLUP);
  } else {
    pinMode(pin, INPUT);
  }
}
uint8_t GPInput::get_input(uint8_t channel) {
  return digitalReadFast(pin);
}
