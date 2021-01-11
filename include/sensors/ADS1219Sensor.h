#pragma once
#include "Sensor.h"
#include "interfaces/DigitalInterface.h"

#include <i2c_device.h>

class ADS1219Sensor : public Sensor {
  private:
    I2CMaster *bus;
    I2CDevice device;
    DigitalInterface *drdy;
  public:
    static const uint8_t GAIN_MASK = 0xEF;
    enum Gain {
      GAIN_1X = 0x00,
      GAIN_4X = 0x10,
    };
    static const uint8_t RATE_MASK = 0xF3;
    enum Rate {
      RATE_20 = 0x00,
      RATE_90 = 0x04,
      RATE_330 = 0x08,
      RATE_1000 = 0x0C,
    };
    static const uint8_t MODE_MASK = 0xFD;
    enum Mode {
      SINGLE = 0x00,
      CONTINUOUS = 0x02,
    };
    static const uint8_t REF_MASK = 0xFE;
    enum Ref {
      INTERNAL = 0x00,
      EXTERNAL = 0x01,
    };
    static const uint8_t MUX_MASK = 0x1F;
    enum Mux {
      DIFF_0_1 = 0x00,
      DIFF_2_3 = 0x20,
      DIFF_1_2 = 0x40,
      SINGLE_0 = 0x60,
      SINGLE_1 = 0x80,
      SINGLE_2 = 0xA0,
      SINGLE_3 = 0xC0,
      SHORTED = 0xE0,
    };
    ADS1219Sensor(I2CMaster *bus, uint8_t address, DigitalInterface *drdy);
    float read(uint8_t channel) override;
};
