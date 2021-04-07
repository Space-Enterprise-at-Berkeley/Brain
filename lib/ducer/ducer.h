/*
  ducer.h - A c++ library to interface with the analog pressure transducers via the
  Created by Vamshi Balanaga, Sept 18, 2020.
*/

#ifndef __DUCERS__
#define __DUCERS__

#include <Arduino.h>
#include <Wire.h>
#include <ADS1219.h>

using namespace std;

namespace Ducers {

  ADS1219 * _adcs;

  int * _adcIndices; // array of size _numSensors
  int * _adcChannels;
  int * _ptTypes;

  int _numSensors; // number of analog thermocouples, not number of adcs

  uint8_t buffer[4];

  void init (int numSensors, int * adcIndices, int * adcChannels, int * ptTypes, ADS1219 * adcs) {
    _numSensors = numSensors;
    _adcIndices = adcIndices;
    _adcChannels = adcChannels;
    _ptTypes = ptTypes;
    _adcs = adcs;
  }
  float interpolateHigh(int rawValue) {
      double values[32][2] =  {
                { 1634771.9270400004, 0 },
                { 1771674.0096000005, 150 },
                { 2544768.12288, 700 },
                { 2601139.56864, 730 },
                { 2681670.205440001, 805 },
                { 2802466.1606400004, 890 },
                { 2931315.1795200002, 990 },
                { 3076270.325760001, 1100 },
                { 3189013.217280001, 1200 },
                { 3414499.0003200006, 1351 },
                { 3543348.019200001, 1450 },
                { 3704409.292800001, 1580 },
                { 3946001.203200001, 1760 },
                { 4155380.858880001, 1930 },
                { 4380866.641920001, 2100 },
                { 4509715.660800001, 2180 },
                { 4751307.571200002, 2400 },
                { 4976793.35424, 2550 },
                { 5202279.13728, 2700 },
                { 5411658.792960001, 2870 },
                { 5621038.448640001, 3020 },
                { 5846524.231680001, 3190 },
                { 6015638.568960002, 3333 },
                { 6128381.460480002, 3426 },
                { 6394132.561920001, 3620 },
                { 6474663.198720002, 3700 },
                { 6667936.727040001, 3850 },
                { 6893422.510080002, 4000 },
                { 6990059.274240003, 4080 },
                { 7135014.420480002, 4186 },
                { 7183332.80256, 4221 },
                { 7392712.4582400005, 4365 }
    };
    bool check = true;
    int index = 0;
    while(check){
      if(rawValue > values[index][0]){index++;}
      else if (rawValue < values[0][0]){return 0;}

        check = false;
      }
      float upperBound = values[index][0];
      float lowerBound = values[index-1][0];
      float upperBoundPressure = values[index][1];
      float lowerBoundPressure = values[index-1][1];
      float proportion = (rawValue - lowerBound)/(upperBound - lowerBound);
      float convertedValue = proportion * (upperBoundPressure - lowerBoundPressure) + lowerBoundPressure;
        //float convertedValue = lerp(values[index-1][1], values[index][1], proportion);

      return convertedValue;
  }
  float interpolateLow(int rawValue) {
    double values[2][2] = { // [x, y] pairs
                {0, -123.89876445934394},
                {8388607, 1131.40825} // 2^23 - 1
              };
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
    Serial.println("readAllPressures");
    int i = 0;
    while (i < _numSensors) {
      int type = _ptTypes[i];
      if (type==1){
        Serial.println("low");
        Serial.flush();
        data[i] = interpolateLow(_adcs[_adcIndices[i]].readData(_adcChannels[i]));
        i++;
      } else {
        Serial.println("high");
        Serial.flush();
        // data[i] = interpolateHigh(_adcs[_adcIndices[i]]->readData(_adcChannels[i]));
        data[i] = _adcs[_adcIndices[i]].readData(_adcChannels[i]);
        i++;
      }
    }
    data[i] = -1;
  }
}

#endif
