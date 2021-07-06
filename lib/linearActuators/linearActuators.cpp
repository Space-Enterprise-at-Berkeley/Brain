/*
  solenoids.h - A c++ library to interface with all of the solenoids on our rocket.
  Created by Vamshi Balanaga, Sept 15, 2020.
*/

#include "linearActuators.h"

using namespace std;

namespace LinearActuators {

  uint8_t _numlinearActuators;
  uint8_t _numPairs; // not actually used... get rid?

  uint8_t * _in1Pins;
  uint8_t * _in2Pins;

  uint8_t * _in1PinState;
  uint8_t * _in2PinState;

  // has the id of the other actuator that is paired w/ the given actuator.
  // single actuators have their own id
  uint8_t * _pairIds;
  uint8_t * _commandIds;

  LinActCommand zero("0", [&] () { return driveForward(0); }, [&] () { return driveBackward(0); }, [&] () { return switchOff(0); }, [&] () { return brake(0); });
  LinActCommand one("1", [&] () { return driveForward(1); }, [&] () { return driveBackward(1); }, [&] () { return switchOff(1); }, [&] () { return brake(1); });
  LinActCommand two("2", [&] () { return driveForward(2); }, [&] () { return driveBackward(2); }, [&] () { return switchOff(2); }, [&] () { return brake(2); });
  LinActCommand three("3", [&] () { return driveForward(3); }, [&] () { return driveBackward(3); }, [&] () { return switchOff(3); }, [&] () { return brake(3); });
  LinActCommand four("4", [&] () { return driveForward(4); }, [&] () { return driveBackward(4); }, [&] () { return switchOff(4); }, [&] () { return brake(4); });
  LinActCommand five("5", [&] () { return driveForward(5); }, [&] () { return driveBackward(5); }, [&] () { return switchOff(5); }, [&] () { return brake(5); });
  LinActCommand six("6", [&] () { return driveForward(6); }, [&] () { return driveBackward(6); }, [&] () { return switchOff(6); }, [&] () { return brake(6); });

  LinActCommand * _linActCommands[7] = {&zero, &one, &two, &three, &four, &five, &six};

  uint8_t _allStates[7];
  uint8_t _prevStates[7];

  void init(uint8_t numActuators, uint8_t numPairs, uint8_t * in1Pins, uint8_t * in2Pins, uint8_t * pairIds, uint8_t * commandIds, TwoWire *wire, uint8_t *outputMonAddrs, float shuntR, float maxExCurrent) {
    init(numActuators, numPairs, in1Pins, in2Pins, pairIds, commandIds);
    for (int i = 0; i < _numlinearActuators; i++){
      _linActCommands[i]->initINA219(wire, outputMonAddrs[i], shuntR, maxExCurrent);
    }
  }


  void init(uint8_t numActuators, uint8_t numPairs, uint8_t * in1Pins, uint8_t * in2Pins, uint8_t * pairIds, uint8_t * commandIds) {
    if(numPairs > numActuators || numPairs % 2 != 0) {
      Serial.println("Error Initializing Linear numActuators! numPairs should be less than numActuators and numPairs should be an even number.");
      Serial.flush();
      exit(1);
    }

    _numlinearActuators = numActuators;
    _numPairs = numPairs;

    _in1Pins = in1Pins;
    _in2Pins = in2Pins;

    _pairIds = pairIds;
    _commandIds = commandIds;

    _in1PinState = (uint8_t *)malloc(_numlinearActuators);
    _in2PinState = (uint8_t *)malloc(_numlinearActuators);

    for (int i = 0; i < _numlinearActuators; i++) {
      pinMode(_in1Pins[i], OUTPUT);
      pinMode(_in2Pins[i], OUTPUT);

      _in1PinState[i] = 0;
      _in2PinState[i] = 0;

      digitalWrite(_in1Pins[i], _in1PinState[i]);
      digitalWrite(_in2Pins[i], _in2PinState[i]);
      _linActCommands[i]->setId(_commandIds[i]);
    }

    for (int i = 0; i < _numlinearActuators; i++) {
      //setting all states to 0 (closed)
      _allStates[i] = 0;
      _prevStates[i] = 0;
    }

  }

  /** In 1, In 2, state
   *   0  , 0   , 0     OFF
   *   0  , 1   , 1     FORWARD
   *   1  , 0   , 2     BACKWARD
   *   1  , 1   , 3     BRAKE
   *  The In1 state is the msb and the In2 state is the lsb.
   *  combine them to form a number.
   */
  void getAllStates(float *data) {
    #ifdef DEBUG
      Serial.println("Actuator Channels, get all states");
      Serial.flush();
    #endif
    for (int i = 0; i < _numlinearActuators; i++){
      int state = 2 * _in1PinState[i] + _in2PinState[i];
      data[i] = state;
      if (state == 0 && _prevStates[i] != 0) {
        //if previously moving forward, new state is open (1)
        if (_prevStates[i] == 1) {
          _allStates[i] = 1;
        //if previously moving backward, new state is closed (0)
        } else if (_prevStates[i] == 2) {
          _allStates[i] = 0;
        }
      }
      _prevStates[i] = state;
    }
    data[_numlinearActuators] = -1;
  }

  void getAllActuatorStates(float *data) {
    #ifdef DEBUG
      Serial.println("Actuators, get all states");
      Serial.flush();
    #endif
    for (int i = 0; i < _numlinearActuators; i++) {
      data[i] = _allStates[i];
    }
    data[_numlinearActuators] = -1;
  }

  void getAllCurrentDraw(float *data) {
    #ifdef DEBUG
      Serial.println("Actuators, get all currents");
      Serial.flush();
    #endif
    for (int i = 0; i < _numlinearActuators; i++){
      data[i] = _linActCommands[i]->outputMonitor.readShuntCurrent();
    }
    data[_numlinearActuators] = -1;
  }


  void driveForward(uint8_t actuatorId, bool activatePair = true) {
    _in1PinState[actuatorId] = 0;
    _in2PinState[actuatorId] = 1;
    digitalWrite(_in1Pins[actuatorId], _in1PinState[actuatorId]);
    digitalWrite(_in2Pins[actuatorId], _in2PinState[actuatorId]);
    if(activatePair) {
      driveForward(_pairIds[actuatorId], false);
    }
    //setting state to "somewhere in between"
    _allStates[actuatorId] = 2;
  }

  void driveBackward(uint8_t actuatorId, bool activatePair = true) {
    _in1PinState[actuatorId] = 1;
    _in2PinState[actuatorId] = 0;
    digitalWrite(_in1Pins[actuatorId], _in1PinState[actuatorId]);
    digitalWrite(_in2Pins[actuatorId], _in2PinState[actuatorId]);
    if(activatePair) {
      driveBackward(_pairIds[actuatorId], false);
    }
    //setting state to "somewhere in between"
    _allStates[actuatorId] = 2;
  }

  void brake(uint8_t actuatorId, bool activatePair = true) {
    _in1PinState[actuatorId] = 1;
    _in2PinState[actuatorId] = 1;
    digitalWrite(_in1Pins[actuatorId], _in1PinState[actuatorId]);
    digitalWrite(_in2Pins[actuatorId], _in2PinState[actuatorId]);
    if(activatePair) {
      brake(_pairIds[actuatorId], false);
    }
  }

  void switchOff(uint8_t actuatorId, bool activatePair = true) {
    _in1PinState[actuatorId] = 0;
    _in2PinState[actuatorId] = 0;
    digitalWrite(_in1Pins[actuatorId], _in1PinState[actuatorId]);
    digitalWrite(_in2Pins[actuatorId], _in2PinState[actuatorId]);
    if(activatePair) {
      switchOff(_pairIds[actuatorId], false);
    }
  }

}
