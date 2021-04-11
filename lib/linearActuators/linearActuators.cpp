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
    }
    zero.setId(_commandIds[0]);
    one.setId(_commandIds[1]);
    two.setId(_commandIds[2]);
    three.setId(_commandIds[3]);
    four.setId(_commandIds[4]);
    five.setId(_commandIds[5]);
    six.setId(_commandIds[6]);
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
      Serial.println("Actuators, get all states");
      Serial.flush();
    #endif
    for (int i = 0; i < _numlinearActuators; i++){
      data[i] = 2 * _in1PinState[i] + _in2PinState[i];
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
  }

  void driveBackward(uint8_t actuatorId, bool activatePair = true) {
    _in1PinState[actuatorId] = 1;
    _in2PinState[actuatorId] = 0;
    digitalWrite(_in1Pins[actuatorId], _in1PinState[actuatorId]);
    digitalWrite(_in2Pins[actuatorId], _in2PinState[actuatorId]);
    if(activatePair) {
      driveBackward(_pairIds[actuatorId], false);
    }
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
    _in1PinState[actuatorId] = 1;
    _in2PinState[actuatorId] = 0;
    digitalWrite(_in1Pins[actuatorId], _in1PinState[actuatorId]);
    digitalWrite(_in2Pins[actuatorId], _in2PinState[actuatorId]);
    if(activatePair) {
      switchOff(_pairIds[actuatorId], false);
    }
  }

  LinActCommand zero("0", [&] () { return driveForward(0); }, [&] () { return driveBackward(0); }, [&] () { return switchOff(0); }, [&] () { return brake(0); });
  LinActCommand one("1", [&] () { return driveForward(1); }, [&] () { return driveBackward(1); }, [&] () { return switchOff(1); }, [&] () { return brake(1); });
  LinActCommand two("2", [&] () { return driveForward(2); }, [&] () { return driveBackward(2); }, [&] () { return switchOff(2); }, [&] () { return brake(2); });
  LinActCommand three("3", [&] () { return driveForward(3); }, [&] () { return driveBackward(3); }, [&] () { return switchOff(3); }, [&] () { return brake(3); });
  LinActCommand four("4", [&] () { return driveForward(4); }, [&] () { return driveBackward(4); }, [&] () { return switchOff(4); }, [&] () { return brake(4); });
  LinActCommand five("5", [&] () { return driveForward(5); }, [&] () { return driveBackward(5); }, [&] () { return switchOff(5); }, [&] () { return brake(5); });
  LinActCommand six("6", [&] () { return driveForward(6); }, [&] () { return driveBackward(6); }, [&] () { return switchOff(6); }, [&] () { return brake(6); });
}
