/*
  ducer.h - A c++ library to interface with the analog pressure transducers via the
  Created by Vamshi Balanaga, Sept 18, 2020.
*/

#ifndef __DUCERS__
#define __DUCERS__

#include <Arduino.h>
#include <ADS1219.h>
#include <ADS8167.h>
#include <cmath>


using namespace std;

namespace Ducers {

  ADC ** _adcs;

  uint8_t * _adcIndices; // array of size _numSensors
  uint8_t * _adcChannels;
  uint8_t * _ptTypes;

  float * _latestReads;

  const uint8_t strideLength = sizeof(ADS8167);

  uint8_t _numSensors; // number of analog thermocouples, not number of adcs

  uint8_t buffer[4];

  void init (uint8_t numSensors, uint8_t * adcIndices, uint8_t * adcChannels, uint8_t * ptTypes, ADC ** adcs) {
    _numSensors = numSensors;
    _adcIndices = adcIndices;
    _adcChannels = adcChannels;
    _ptTypes = ptTypes;
    _adcs = adcs;
    _latestReads = (float *)malloc(numSensors);
  }

  /*
    lox    : Static Pressure=-1.158+1.029*Dome Pressure-0.02228*High Pressure
    propane: Static Pressure=-20.08+1.413*Dome Pressure+0.002343*High Pressure
  */

  float loxStaticP(float loxDomeP, float highP) {
    return -1.158 + 1.029 * loxDomeP - 0.02228 * highP;
  }

  float propStaticP(float propDomeP, float highP) {
    return -20.08 + 1.413 * propDomeP + 0.02343 * highP;
  }

  float interpolateHigh(long rawValue) {
      double values[2][2] =  {
                { 0, 0 },
                { 64850, 5000 }
    };

    // return std::lerp(0.0, 5000.0, (double)rawValue/64850);
    float upperBound = values[1][0];
    float lowerBound = values[0][0];
    float upperBoundPressure = values[1][1];
    float lowerBoundPressure = values[0][1];
    float proportion = (rawValue - lowerBound)/(upperBound - lowerBound);
    float convertedValue = proportion * (upperBoundPressure - lowerBoundPressure) + lowerBoundPressure;
    return convertedValue;
  }

  float interpolateLow(long rawValue) {
    double values[2][2] = { // [x, y] pairs
                {0, -123.89876445934394},
                {64901, 1131.40825} // 2^23 - 1
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
  // All the following reads are blocking calls.
  // this function takes ~ _numSensors * 15ms given the data rate of 90
  void readAllPressures(float *data) {
    for (int i = 0; i < _numSensors; i++) {
      int type = _ptTypes[i];
      if (type == 1) {
        #ifdef DEBUG
          Serial.println("reading low pressure data from ADC" + String(_adcIndices[i]) + " Ain" + String(_adcChannels[i]));
          Serial.flush();
        #endif
        data[i] = interpolateLow(_adcs[_adcIndices[i]]->readData(_adcChannels[i]));
      } else {
        #ifdef DEBUG
          Serial.println("reading high pressure data from ADC" + String(_adcIndices[i]) + " Ain" + String(_adcChannels[i]));
          Serial.flush();
        #endif
        data[i] = interpolateHigh(_adcs[_adcIndices[i]]->readData(_adcChannels[i]));
        //data[i] = _adcs[_adcIndices[i]]->readData(_adcChannels[i]);
      }
      _latestReads[i] = data[i];
      i++;
    }
    data[_numSensors] = -1;
  }
}

#endif
