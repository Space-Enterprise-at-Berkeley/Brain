/*
  solenoids.h - A c++ library to interface with all of the solenoids on our rocket.
  Created by Vamshi Balanaga, Sept 15, 2020.
*/

#include "solenoids.h"

using namespace std;

namespace Solenoids {

  uint8_t lox_2_pin, lox_5_pin, lox_gems_pin;
  uint8_t prop_2_pin, prop_5_pin, prop_gems_pin;
  uint8_t high_sol_pin, high_sol_enable_pin;

  uint8_t high_sol_enable_state = 0;
  uint8_t *_commandIds;
  uint8_t _numCommands;

  uint8_t *_outputMonitorAddrs;

  uint8_t high_sol_state = 0;

  uint8_t lox2_state = 0;
  uint8_t lox5_state = 0;
  uint8_t lox_gems_state = 0;

  uint8_t prop2_state = 0;
  uint8_t prop5_state = 0;
  uint8_t prop_gems_state = 0;

  float _pressurantSolenoidMonitorShuntR;
  LTC4151 *_pressurantSolenoidMonitor;

  void init(uint8_t numSolenoids, uint8_t * solenoidPins, uint8_t numCommands, uint8_t * commandIds, uint8_t * outputMonitorAddrs, TwoWire *wire, float shuntR, float maxExpectedCurrent,  LTC4151 *pressurantMonitor, float pressurantSolMonShuntR) {
    lox2_state = 0;
    lox5_state = 0;
    lox_gems_state = 0;

    prop2_state = 0;
    prop5_state = 0;
    prop_gems_state = 0;

    high_sol_state = 0;
    high_sol_enable_state = 0;

    lox_2_pin = solenoidPins[0];
    lox_5_pin = solenoidPins[1];
    lox_gems_pin = solenoidPins[2];

    prop_2_pin = solenoidPins[3];
    prop_5_pin = solenoidPins[4];
    prop_gems_pin = solenoidPins[5];

    high_sol_pin = solenoidPins[6];
    if(numSolenoids > 7) {
      high_sol_enable_pin = solenoidPins[7];
    }

    _numCommands = numCommands;
    _commandIds = commandIds;

    _outputMonitorAddrs = outputMonitorAddrs;
    _pressurantSolenoidMonitor = pressurantMonitor;
    _pressurantSolenoidMonitorShuntR = pressurantSolMonShuntR;

    pinMode(lox_2_pin, OUTPUT);
    pinMode(lox_5_pin, OUTPUT);
    pinMode(lox_gems_pin, OUTPUT);

    pinMode(prop_2_pin, OUTPUT);
    pinMode(prop_5_pin, OUTPUT);
    pinMode(prop_gems_pin, OUTPUT);

    pinMode(high_sol_pin, OUTPUT);
    pinMode(high_sol_enable_pin, OUTPUT);

    digitalWrite(lox_2_pin, lox2_state);
    digitalWrite(lox_5_pin, lox5_state);
    digitalWrite(lox_gems_pin, lox_gems_state);

    digitalWrite(prop_2_pin, prop2_state);
    digitalWrite(prop_5_pin, prop5_state);
    digitalWrite(prop_gems_pin, prop_gems_state);

    digitalWrite(high_sol_pin, high_sol_state);
    digitalWrite(high_sol_enable_pin, high_sol_enable_state);

    lox_2.setId(_commandIds[0]);
    lox_5.setId(_commandIds[1]);
    lox_G.setId(_commandIds[2]);

    prop_2.setId(_commandIds[3]);
    prop_5.setId(_commandIds[4]);
    prop_G.setId(_commandIds[5]);

    high_p.setId(_commandIds[6]);

    arm_rocket.setId(_commandIds[7]);
    launch.setId(_commandIds[8]);
    if(_numCommands > 9) {
      high_p_enable.setId(_commandIds[9]);
    }

    lox_2.initINA219(wire, outputMonitorAddrs[0], shuntR, maxExpectedCurrent);
    lox_5.initINA219(wire, outputMonitorAddrs[1], shuntR, maxExpectedCurrent);
    lox_G.initINA219(wire, outputMonitorAddrs[2], shuntR, maxExpectedCurrent);
    prop_2.initINA219(wire, outputMonitorAddrs[3], shuntR, maxExpectedCurrent);
    prop_5.initINA219(wire, outputMonitorAddrs[4], shuntR, maxExpectedCurrent);
    prop_G.initINA219(wire, outputMonitorAddrs[5], shuntR, maxExpectedCurrent);

  }

  void getAllStates(float *data) {
    #ifdef DEBUG
      Serial.println("Solenoids, get all states");
      Serial.flush();
    #endif
    data[0] = lox2_state;
    data[1] = prop2_state;
    data[2] = lox5_state;
    data[3] = prop5_state;
    data[4] = lox_gems_state;
    data[5] = prop_gems_state;
    data[6] = high_sol_state;
    data[7] = high_sol_enable_state;
    data[8] = -1;
  }

  void getAllCurrents(float *data) {
    #ifdef DEBUG
      Serial.println("Solenoids, get all currents");
      Serial.flush();
    #endif
    data[0] = lox_2.outputMonitor.readShuntCurrent();
    data[1] = prop_2.outputMonitor.readShuntCurrent();
    data[2] = lox_5.outputMonitor.readShuntCurrent();
    data[3] = prop_5.outputMonitor.readShuntCurrent();
    data[4] = lox_G.outputMonitor.readShuntCurrent();
    data[5] = prop_G.outputMonitor.readShuntCurrent();
    data[6] = _pressurantSolenoidMonitor->getLoadCurrent(_pressurantSolenoidMonitorShuntR);
    data[7] = -1;
    data[8] = -1;
  }

  void getAllVoltages(float *data) {
    #ifdef DEBUG
      Serial.println("Solenoids, get all currents");
      Serial.flush();
    #endif
    data[0] = lox_2.outputMonitor.readBusVoltage();
    data[1] = prop_2.outputMonitor.readBusVoltage();
    data[2] = lox_5.outputMonitor.readBusVoltage();
    data[3] = prop_5.outputMonitor.readBusVoltage();
    data[4] = lox_G.outputMonitor.readBusVoltage();
    data[5] = prop_G.outputMonitor.readBusVoltage();
    data[6] = _pressurantSolenoidMonitor->getADCInVoltage() * 162.7 / 4.7;
    data[7] = _pressurantSolenoidMonitor->getInputVoltage();
    data[8] = -1;
  }

  bool loxArmed() {
    return lox2_state == 1;
  }

  bool propArmed() {
    return prop2_state == 1;
  }

  int toggleHighPressureSolenoid() {
    if(high_sol_state == 0){
      high_sol_state = 1;
    } else {
      high_sol_state = 0;
    }
    digitalWrite(high_sol_pin, high_sol_state);
    return high_sol_state;
  }

  int toggleLOX2Way() {
    if (lox2_state == 0){
      lox2_state = 1;
    } else {
      lox2_state = 0;
    }
    digitalWrite(lox_2_pin, lox2_state);
    return lox2_state;
  }

  int toggleLOX5Way() {
    if (lox5_state == 0){
      lox5_state = 1;
    } else {
      lox5_state = 0;
    }
    digitalWrite(lox_5_pin, lox5_state);
    return lox5_state;
  }

  int toggleLOXGems() {
    if (lox_gems_state == 0){
      lox_gems_state = 1;
    } else {
      lox_gems_state = 0;
    }
    digitalWrite(lox_gems_pin, lox_gems_state);
    return lox_gems_state;
  }

  int toggleProp2Way() {
    if (prop2_state == 0){
      prop2_state = 1;
    } else {
      prop2_state = 0;
    }
    digitalWrite(prop_2_pin, prop2_state);
    return prop2_state;
  }

  int toggleProp5Way() {
    if (prop5_state == 0){
      prop5_state = 1;
    } else {
      prop5_state = 0;
    }
    digitalWrite(prop_5_pin, prop5_state);
    return prop5_state;
  }

  int togglePropGems() {
    if (prop_gems_state == 0){
      prop_gems_state = 1;
    } else {
      prop_gems_state = 0;
    }
    digitalWrite(prop_gems_pin, prop_gems_state);
    return prop_gems_state;
  }

  int openHighPressureSolenoid() {
    if(high_sol_state == 0){
      toggleHighPressureSolenoid();
    } else {
      // already active, do nothing.
    }
    return high_sol_state;
  }

  int closeHighPressureSolenoid() {
    if(high_sol_state == 1){
      toggleHighPressureSolenoid();
    } else {
      // already closed, do nothing.
    }
    return high_sol_state;
  }

  int enableHighPressureSolenoid() {
    high_sol_enable_state = 1;
    digitalWrite(high_sol_enable_pin, high_sol_enable_state);
    return high_sol_state;
  }

  int disableHighPressureSolenoid() {
    high_sol_enable_state = 0;
    digitalWrite(high_sol_enable_pin, high_sol_enable_state);
    return high_sol_state;
  }

  int ventLOXGems(){
    if (lox_gems_state == 0) {
      toggleLOXGems();
    } else {
      // already active, do nothing.
    }
    return lox_gems_state;
  }

  int closeLOXGems(){
    if (lox_gems_state == 1) {
      toggleLOXGems();
    } else {
      // already closed, do nothing.
    }
    return lox_gems_state;
  }

  int ventPropaneGems(){
    if (prop_gems_state == 0) {
      togglePropGems();
    } else {
      // already active, do nothing.
    }
    return prop_gems_state;
  }

  int closePropaneGems(){
    if (prop_gems_state == 1) {
      togglePropGems();
    } else {
      // already closed, do nothing.
    }
    return prop_gems_state;
  }

  int armLOX() {
    if (lox2_state == 0) {
      toggleLOX2Way();
    } else {
      // already active, do nothing.
    }
    return lox2_state;
  }

  int disarmLOX() {
    if (lox2_state == 1) {
      toggleLOX2Way();
    } else {
      // already closed, do nothing.
    }
    return lox2_state;
  }

  int armPropane() {
    if (prop2_state == 0) {
      toggleProp2Way();
    } else {
      // already active, do nothing.
    }
    return prop2_state;
  }

  int disarmPropane() {
    if (prop2_state == 1) {
      toggleProp2Way();
    } else {
      // already closed, do nothing.
    }
    return prop2_state;
  }

  int armAll() {
    return armLOX() & armPropane();
  }

  int disarmAll(){
    return (!disarmLOX() & !disarmPropane());
  }

  int openLOX() {
    if (lox5_state == 0) {
      toggleLOX5Way();
    } else {
      // already active, do nothing.
    }
    return lox5_state;
  }

  int closeLOX() {
    if (lox5_state == 1) {
      toggleLOX5Way();
    } else {
      // already closed, do nothing.
    }
    return lox5_state;
  }

  int openPropane() {
    if (prop5_state == 0) {
      toggleProp5Way();
    } else {
      // already active, do nothing.
    }
    return prop5_state;
  }

  int closePropane() {
    if(prop5_state == 1){
      toggleProp5Way();
    } else {
      // already closed, do nothing.
    }
    return prop5_state;
  }

  int LAUNCH() {
    openLOX();
    openPropane();
    return 1;
  }

  int endBurn() {
    closeLOX();
    closePropane();
    return 1;
  }

  int getHPS() {
    return high_sol_state;
  }

  int getLox2() {
    return lox2_state;
  }

  int getLox5() {
    return lox5_state;
  }

  int getLoxGems() {
    return lox_gems_state;
  }

  int getProp2() {
    return prop2_state;
  }

  int getProp5() {
    return prop5_state;
  }

  int getPropGems() {
    return prop_gems_state;
  }

  SolenoidCommand lox_2("LOX 2 Way", &armLOX, &disarmLOX);
  SolenoidCommand lox_5("LOX 5 Way", &openLOX, &closeLOX);
  SolenoidCommand lox_G("LOX Gems", &ventLOXGems, &closeLOXGems);
  SolenoidCommand prop_2("Propane 2 Way", &armPropane, &disarmPropane);
  SolenoidCommand prop_5("Propane 5 Way", &openPropane, &closePropane);
  SolenoidCommand prop_G("Propane Gems", &ventPropaneGems, &closePropaneGems);

  SolenoidCommand high_p_enable("Enable High Pressure Solenoid", &enableHighPressureSolenoid, &disableHighPressureSolenoid);
  SolenoidCommand high_p("High Pressure Solenoid", &openHighPressureSolenoid, &closeHighPressureSolenoid);

  SolenoidCommand arm_rocket("Arm Rocket", &armAll, &disarmAll);
  SolenoidCommand launch("Launch Rocket", &LAUNCH, &endBurn);
}
