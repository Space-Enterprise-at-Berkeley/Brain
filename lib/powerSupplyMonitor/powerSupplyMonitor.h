/*
  batteryMonitor.h - A c++ library to interface w/ the ina226 chip and read battery stats.
  Created by Vamshi Balanaga, Sept 15, 2020.
*/
#ifndef __PS_MON__
#define __PS_MON__

#include <INA219.h> //https://github.com/jarzebski/Arduino-INA219
#include <common.h>

using namespace std;

namespace powerSupplyMonitor {

  uint8_t _numSupplies;

  INA ** _supplyMonitors;
  uint8_t * _addrs;

  TwoWire * _localWire;

  float * _energyConsumed;
  long last_checked;

  void init(uint8_t numSupplies, INA ** supplyMons, uint8_t * addrs, float rShunt, float maxExpectedCurrent, TwoWire *localWire) {
    _numSupplies = numSupplies;
    _localWire = localWire;
    _addrs = addrs;
    _supplyMonitors = supplyMons;

    _energyConsumed = (float *) malloc(numSupplies * sizeof(float));

    for (int i = 0; i < _numSupplies; i++) {
      // _supplyMonitors[i]->begin(localWire, addrs[i]);
      // _supplyMonitors[i]->configure(INA219_RANGE_32V, INA219_GAIN_40MV, INA219_BUS_RES_12BIT, INA219_SHUNT_RES_12BIT_1S);
      // _supplyMonitors[i]->calibrate(rShunt, maxExpectedCurrent);
      debug("started INA219");
      debug("testinh");
      _energyConsumed[i] = 0;
      debug("testing2");
    }
  }

  void readVoltage(float *data) {
    for (int i = 0; i < _numSupplies; i++){
      data[i] = _supplyMonitors[i]->readBusVoltage();
    }
    data[_numSupplies] = -1;
  }

  void readCurrent(float *data) {
    for (int i = 0; i < _numSupplies; i++){
      data[i] = _supplyMonitors[i]->readShuntCurrent();
    }
    data[_numSupplies] = -1;
  }

  void readPowerConsumption(float *data) {
    for (int i = 0; i < _numSupplies; i++) {
      data[i] = _supplyMonitors[i]->readBusPower();
      _energyConsumed[i] += data[i] * (millis() - last_checked) / 1000;
    }
    last_checked = millis();
  }

  void getEnergyConsumption(float *data) {
    readPowerConsumption(data);
    for (int i = 0; i < _numSupplies; i++) {
      data[i + _numSupplies] = _energyConsumed[i];
    }
    data[_numSupplies * 2] = -1;
  }

  void readAllBatteryStats(float *data) {
    readVoltage(data);
    readCurrent(data + _numSupplies);
  }

}


#endif /* end of include guard: BATT_MON */
