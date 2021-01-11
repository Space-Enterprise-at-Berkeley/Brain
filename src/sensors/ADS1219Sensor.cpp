#include "sensors/ADS1219Sensor.h"

ADS1219Sensor::ADS1219Sensor(I2CMaster *bus, uint8_t address, DigitalInterface *drdy)
  : bus(bus), device(*bus, address, _BIG_ENDIAN), drdy(drdy)
{
}

float ADS1219Sensor::read(uint8_t channel) {
  uint8_t config = 0;
  config |= Gain::GAIN_1X;
  config |= Rate::RATE_330;
  config |= Mode::SINGLE;
  config |= Ref::EXTERNAL;
  switch(channel) {
    case 0:
      config |= Mux::SINGLE_0;
      break;
    case 1:
      config |= Mux::SINGLE_1;
      break;
    case 2:
      config |= Mux::SINGLE_2;
      break;
    case 3:
      config |= Mux::SINGLE_3;
      break;
    default:
      return 0.0;
  }
  device.write(0x40, &config, 1, false);
  device.write(0x08, nullptr, 0, true);
  while(drdy->get_input(0) == 1); // wait for response
  uint8_t *buffer;
  device.read(0x10, buffer, 3, true);
  long data32 = buffer[0];
  data32 <<= 8;
  data32 |= buffer[1];
  data32 <<= 8;
  data32 |= buffer[2];
  return (data32 << 8) >> 8;
}
