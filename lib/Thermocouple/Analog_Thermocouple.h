/*
  Analog_Thermocouple.h - A c++ library to interface with analog thermocouples
  via our ADCs: ADS1219.
  Created by Vamshi Balanaga, Jan 20, 2021.
*/

#ifndef __ANALOG_THERMS__
#define __ANALOG_THERMS__

#include <ADS1219.h>
#include <ADS8167.h>
#include <Wire.h>

namespace Thermocouple {
  namespace Analog {

    ADC ** _adcs;

    uint8_t * _adcIndices; // array of size _numSensors
    uint8_t * _adcChannels;

    uint8_t _numSensors; // number of analog thermocouples, not number of adcs

    float tempOverVoltageScale = 1 / 0.01;
    float voltageOffset = 0.75, tempOffset = 25; // 25 C = 0.450 V

    long rawRead;
    float voltageRead;

    void init (uint8_t numSensors, uint8_t * adcIndices, uint8_t * adcChannels, ADC ** adcs) {
      _numSensors = numSensors;
      _adcIndices = adcIndices;
      _adcChannels = adcChannels;
      _adcs = adcs;
    }

    // void readTemperatureData(float *data) {
    //   int i = 0;
    //   while (i < _numSensors) {
    //     rawRead = _adcs[_adcIndices[i]]->readData(_adcChannels[i]);
    //     voltageRead = (float) rawRead * (5.0 / pow(2,15));
    //     data[i] = ((voltageRead - voltageOffset) * tempOverVoltageScale) + tempOffset;
    //     i++;
    //   }
    //   data[i] = -1;
    // }


    float readSpecificTemperatureData(int idx) {
      long rawValue = _adcs[_adcIndices[idx]]->readData(_adcChannels[idx]);
      // voltageRead = (float) rawRead * (5.0 / pow(2,15));
      // data[0] = ((voltageRead - voltageOffset) * tempOverVoltageScale) + tempOffset;
      // data[1] = -1;

      double values[2][2] = { // [x, y] pairs
        {1310,  -40},
        {22937, 125}
      };
      // return std::lerp(-123.89876445934394, 1131.40825, (double) rawValue / 64901);
      float upperBound = values[1][0];
      float lowerBound = values[0][0];
      float upperBoundPressure = values[1][1];
      float lowerBoundPressure = values[0][1];
      float proportion = (rawValue - lowerBound)/(upperBound - lowerBound);
      float convertedValue = proportion * (upperBoundPressure - lowerBoundPressure) + lowerBoundPressure;
      return convertedValue;
    }

    // float *data is only of size 7 rn, ensure that we only expect <= 7 readings.
    void readTemperatureData(float *data) {
      int i = 0;
      while (i < _numSensors) {
        data[i] = readSpecificTemperatureData(i);
        i++;
      }
      data[i] = -1;
    }





    void readSpecificTemperatureData(int idx, float *data) {
      long rawValue = _adcs[_adcIndices[idx]]->readData(_adcChannels[idx]);
      // voltageRead = (float) rawRead * (5.0 / pow(2,15));
      // data[0] = ((voltageRead - voltageOffset) * tempOverVoltageScale) + tempOffset;
      // data[1] = -1;

      double values[2][2] = { // [x, y] pairs
        {1310,  -40},
        {22937, 125}
      };
      // return std::lerp(-123.89876445934394, 1131.40825, (double) rawValue / 64901);
      float upperBound = values[1][0];
      float lowerBound = values[0][0];
      float upperBoundPressure = values[1][1];
      float lowerBoundPressure = values[0][1];
      float proportion = (rawValue - lowerBound)/(upperBound - lowerBound);
      float convertedValue = proportion * (upperBoundPressure - lowerBoundPressure) + lowerBoundPressure;
      data[0] = convertedValue;
      data[1] = -1;
    }

  }
}

#endif
