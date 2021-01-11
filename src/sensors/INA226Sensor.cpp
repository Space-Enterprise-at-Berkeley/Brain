#include "sensors/INA226Sensor.h"

INA226Sensor::INA226Sensor(I2CMaster *bus, uint8_t address, DigitalInterface *alert, float shunt, float imax, bool inverted)
  : bus(bus), device(*bus, address, _BIG_ENDIAN), alert(alert), inverted(inverted)
{
  uint16_t config = 0;
  config |= Averages::AVERAGE_1 << 9;
  config |= Time::TIME_1100US << 6; // bus conv time
  config |= Time::TIME_1100US << 3; // shunt conv time
  config |= Mode::SHUNT_BUS_CONT;
  device.write(0x00, config, false);

  float i_max_possible = 0.08192 / shunt;
  float minimum_lsb = imax / 32767.0;
  current_lsb = (uint16_t)(minimum_lsb * 100000000);
  current_lsb /= 100000000;
  current_lsb /= 0.0001;
  current_lsb = ceil(current_lsb);
  current_lsb *= 0.0001;
  power_lsb = current_lsb * 25.0;
  uint16_t calibration = (uint16_t)((0.00512) / (current_lsb * shunt));
  device.write(0x05, calibration, true);
}

float INA226Sensor::read(uint8_t channel) {
  uint16_t raw = 0;
  switch(channel) {
    case 0: // bus voltage
      device.read(0x02, &raw, true);
      return raw * 0.00125;
    case 1: // shunt current
      device.read(0x04, &raw, true);
      return (raw * current_lsb);
    case 2: // bus power
      device.read(0x03, &raw, true);
      return raw * power_lsb;
    default:
      return 0.0;
  }
}
