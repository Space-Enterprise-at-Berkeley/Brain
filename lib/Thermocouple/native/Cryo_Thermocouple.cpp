/*
  Cryo_Thermocouple.cpp - A c++ library to interface with heavy duty cryo/high temp
  thermocouples using the MCP9600 chip.
  Created by Vamshi Balanaga, Feb 5, 2021.
*/



#include <Cryo_Thermocouple.h>
#include <common.h>

namespace Thermocouple {

    int Cryo::init(uint8_t numSensors, Adafruit_MCP9600 *cryo_boards, uint8_t * addrs, _themotype * types, TwoWire *theWire, float *latestReads) { // assume that numSensors is < max Size of packet. Add some error checking here
      _numSensors = numSensors;
      return 0;
    }

    void Cryo::readCryoTemps(float *data) {
      for (int i = 0; i < _numSensors; i++) {
        data[i] = 99;
      }
      data[_numSensors] = -1;
    }

    void Cryo::readSpecificCryoTemp(uint8_t index, float *data) {
      data[0] = 99;
      data[1] = -1;
    }

    int Cryo::freeAllResources() {
        return 0;
    }

}
