/*
  solenoids.h - A c++ library to interface with all of the solenoids on our rocket.
  Created by Vamshi Balanaga, Sept 15, 2020.
*/
#ifndef __LINACT__
#define __LINACT__

#include <Arduino.h>
#include <command.h>
#include <Wire.h>
#include <INA219.h>

using namespace std;

namespace LinearActuators {

  extern uint8_t _numlinearActuators;
  extern uint8_t _numPairs;

  extern uint8_t * _in1Pins;
  extern uint8_t * _in2Pins;

  extern uint8_t * _in1PinState;
  extern uint8_t * _in2PinState;

  extern uint8_t * _pairIds;
  extern uint8_t * _commandIds;


  void init(uint8_t numActuators, uint8_t numPairs, uint8_t * in1Pins, uint8_t * in2Pins, uint8_t * pairIds, uint8_t * commandIds, TwoWire *wire, uint8_t *outputMonAddrs, float shuntR, float maxExCurrent);
  void init(uint8_t numActuators, uint8_t numPairs, uint8_t * in1Pins, uint8_t * in2Pins, uint8_t * pairIds, uint8_t * commandIds);
  void getAllStates(float *data);
  void getAllCurrentDraw(float *data);
  void driveForward(uint8_t actuatorId, bool activatePair = true);
  void driveBackward(uint8_t actuatorId, bool activatePair = true);
  void brake(uint8_t actuatorId, bool activatePair = true);
  void switchOff(uint8_t actuatorId, bool activatePair = true);
  void getAllActuatorStates(float *data);

  class LinActCommand : public Command {

    typedef void (*func_t)();

    public:
      LinActCommand(std::string name, uint8_t id, bool pair, func_t forward, func_t back, func_t off, func_t damp):
        Command(name, id),
        _forward(forward),
        _backward(back),
        _off(off),
        _brake(damp)
      {}

      LinActCommand(std::string name, func_t forward, func_t back, func_t off, func_t damp):
        Command(name),
        _forward(forward),
        _backward(back),
        _off(off),
        _brake(damp)
      {}

      void initINA219(TwoWire *wire, uint8_t inaAddr, float shuntR, float maxExpectedCurrent) {
        outputMonitor.begin(wire, inaAddr);
        outputMonitor.configure(INA219_RANGE_16V, INA219_GAIN_40MV, INA219_BUS_RES_12BIT, INA219_SHUNT_RES_12BIT_1S);
        outputMonitor.calibrate(shuntR, maxExpectedCurrent);
      }

      void parseCommand(float *data) {
        #ifdef DEBUG
          Serial.println("lin act, parse command");
          Serial.flush();
        #endif

         if (data[0] == 0) { // off
          _off();
        } else if (data[0] == 1)  { // forward
          _forward();
          endtime = -1;
        } else if (data[0] == -1)  { // backward
          _backward();
          endtime = -1;
        } else { // timed
          Serial.println("TIMED COMMAND");
          if (data[0] > 1) { // forward, timed
            _forward();
            endtime = millis() + data[0];
          } else if(data[0] < -1) { // backward, timed
            _backward();
            endtime = millis() - data[0];
          }
        }

      }

      void confirmation(float *data) {
        LinearActuators::getAllStates(data);
      }

      INA219 outputMonitor;
      long endtime;
      func_t _off;

    private:
      func_t _forward;
      func_t _backward;
      func_t _brake;

  };

  extern LinActCommand zero;
  extern LinActCommand one;
  extern LinActCommand two;
  extern LinActCommand three;
  extern LinActCommand four;
  extern LinActCommand five;
  extern LinActCommand six;
  extern LinActCommand seven;

  extern LinActCommand * _linActCommands[];

}

#endif /* end of include guard: LINACT */
