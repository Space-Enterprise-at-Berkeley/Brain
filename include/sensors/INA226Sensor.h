#pragma once
#include "Sensor.h"
#include "interfaces/DigitalInterface.h"

#include <i2c_device.h>

class INA226Sensor : public Sensor {
  public:
    enum Averages {
      AVERAGE_1             = 0b000,
      AVERAGE_4             = 0b001,
      AVERAGE_16            = 0b010,
      AVERAGE_64            = 0b011,
      AVERAGE_128           = 0b100,
      AVERAGE_256           = 0b101,
      AVERAGE_512           = 0b110,
      AVERAGE_1024          = 0b111,
    };
    enum Time {
      TIME_140US    = 0b000,
      TIME_204US    = 0b001,
      TIME_332US    = 0b010,
      TIME_588US    = 0b011,
      TIME_1100US   = 0b100,
      TIME_2116US   = 0b101,
      TIME_4156US   = 0b110,
      TIME_8244US   = 0b111,
    };
    enum Mode {
      POWER_DOWN      = 0b000,
      SHUNT_TRIG      = 0b001,
      BUS_TRIG        = 0b010,
      SHUNT_BUS_TRIG  = 0b011,
      ADC_OFF         = 0b100,
      SHUNT_CONT      = 0b101,
      BUS_CONT        = 0b110,
      SHUNT_BUS_CONT  = 0b111,
    };
  private:
    I2CMaster *bus;
    I2CDevice device;
    DigitalInterface *alert;
    bool inverted;
    float current_lsb;
    float power_lsb;
  public:
    INA226Sensor(I2CMaster *bus, uint8_t address, DigitalInterface *alert, float shunt, float imax, bool inverted = false);
    float read(uint8_t channel) override;
};
