/*
  Cryo_Thermocouple.h - A c++ library to interface with heavy duty cryo/high temp
  thermocouples using the MCP9600 chip.
  Created by Vamshi Balanaga, Jan 20, 2021.
*/

#ifndef __CRYO_THERMS__
#define __CRYO_THERMS__

#include <Adafruit_MCP9600.h>

namespace Thermocouple {
  class Cryo {
    public:
      int init(uint8_t numSensors, Adafruit_MCP9600 *cryo_boards, uint8_t * addrs, _themotype * types, TwoWire *theWire, float *latestReads); // assume that numSensors is < max Size of packet. Add some error checking here
      int lowerI2CSpeed(TwoWire *theWire);
      void readCryoTemps(float *data);
      void readSpecificCryoTemp(uint8_t index, float *data);
      int freeAllResources();
    private:
      Adafruit_MCP9600 * _cryo_amp_boards;
      uint8_t * _addrs;
      uint8_t _numSensors;
      float * _latestReads;
      bool _lowerI2CSpeed = false;
  };

}

#endif
