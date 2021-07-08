/*
  tempControl.h - A c++ algorithm that maintains the temperature at a given set point.
  Two versions: One very simple, naive algorithm. (Needs to be empirically tested to verify).
  Another, more complex control loop that should reject all disturbances quite well.
  Created by Vamshi Balanaga, Oct 7, 2020.
*/

#ifndef __TEMP_CONTROLLER__
#define __TEMP_CONTROLLER__
#include <cmath>
#include "pid.h"
#include <command.h>
#include <GpioExpander.h>
#include <INA219.h>

using namespace std;


class TempController {
  private:
    int _algorithmChoice; // 1 - naive, 2 - actual control theory
    int _setPointTemp;
    int _heaterPin = 0;
    int _heaterOutput = 0;

    float k_p = 100;
    float k_i = 0.0000001; // low because t is in millis
    float k_d = 1000;

    PID *controller = new PID(255, 0, k_p, k_i, k_d);

    GpioExpander *_expander;
    int8_t _channel;

  protected:
    bool humanOverride = true;
    uint16_t humanSpecifiedValue = 0;
    INA219 outputMonitor;
    bool inaExists = false;

  public:

    TempController(int tempSetPoint, int algorithmChoice, int heaterPin);
    TempController(int tempSetPoint, int algorithmChoice, GpioExpander * expander, int8_t channel);
    int calculateOutput(float currTemp);
    float controlTemp(float currTemp);

};


class HeaterCommand : public Command, public TempController {

  public:
    HeaterCommand(std::string name, uint8_t id, int tempSetPoint, int algorithmChoice, int heaterPin, TwoWire *wire, uint8_t inaAddr, float shuntR, float maxExpectedCurrent):
      Command(name, id),
      TempController(tempSetPoint, algorithmChoice, heaterPin)
    {
      initINA219(wire, inaAddr, shuntR, maxExpectedCurrent);
      inaExists = true;
    }

    HeaterCommand(std::string name, uint8_t id, int tempSetPoint, int algorithmChoice, int heaterPin):
      Command(name, id),
      TempController(tempSetPoint, algorithmChoice, heaterPin)
    {
      inaExists = false;
    }

    HeaterCommand(std::string name, uint8_t id, int tempSetPoint, int algorithmChoice, GpioExpander * expander, int8_t channel):
      Command(name, id),
      TempController(tempSetPoint, algorithmChoice, expander, channel)
    {
      inaExists = false;
    }


    HeaterCommand(std::string name, uint8_t id, int tempSetPoint, int algorithmChoice, GpioExpander * expander, int8_t channel, TwoWire *wire, uint8_t inaAddr, float shuntR, float maxExpectedCurrent):
      Command(name, id),
      TempController(tempSetPoint, algorithmChoice, expander, channel)
    {
      initINA219(wire, inaAddr, shuntR, maxExpectedCurrent);
      inaExists = true;
    }

    HeaterCommand(std::string name, int tempSetPoint, int algorithmChoice, int heaterPin, TwoWire *wire, uint8_t inaAddr, float shuntR, float maxExpectedCurrent):
      Command(name),
      TempController(tempSetPoint, algorithmChoice, heaterPin)
    {
      initINA219(wire, inaAddr, shuntR, maxExpectedCurrent);
      inaExists = true;
    }

    HeaterCommand(std::string name, int tempSetPoint, int algorithmChoice, int heaterPin):
      Command(name),
      TempController(tempSetPoint, algorithmChoice, heaterPin)
    {
      inaExists = false;
    }

    void parseCommand(float *data) {
      if (data[0] == 300)
        humanOverride = false;
      else {
        humanOverride = true;
        humanSpecifiedValue = max(0, min(255, data[0]));
      }
      controlTemp(0);
    }

    void confirmation(float *data) {
      data[0] = humanOverride;
      data[1] = (humanOverride)? humanSpecifiedValue : -1;
      if(inaExists){
        data[2] = outputMonitor.readShuntCurrent();
        data[3] = outputMonitor.readBusVoltage();
      } else {
        data[2] = -1;
      }
      data[4] = -1;
    }

    float readCurrentDraw() {
      return outputMonitor.readShuntCurrent();
    }

    float readBusVoltage() {
      return outputMonitor.readBusVoltage();
    }

    void initINA219(TwoWire *wire, uint8_t inaAddr, float shuntR, float maxExpectedCurrent) {
      outputMonitor.begin(wire, inaAddr);
      outputMonitor.configure(INA219_RANGE_32V, INA219_GAIN_40MV, INA219_BUS_RES_12BIT, INA219_SHUNT_RES_12BIT_1S);
      outputMonitor.calibrate(shuntR, maxExpectedCurrent);
    }
};
#endif
