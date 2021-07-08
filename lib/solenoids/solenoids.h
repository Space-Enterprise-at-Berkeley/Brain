/*
  solenoids.h - A c++ library to interface with all of the solenoids on our rocket.
  Created by Vamshi Balanaga, Sept 15, 2020.
*/
#ifndef __SOLENOIDS__
#define __SOLENOIDS__

#include <Arduino.h>
#include <command.h>
#include <INA219.h>
#include <LTC4151.h>

using namespace std;

namespace Solenoids {


  extern uint8_t lox_2_pin, lox_5_pin, lox_gems_pin;
  extern uint8_t prop_2_pin, prop_5_pin, prop_gems_pin;
  extern uint8_t high_sol_pin, high_sol_enable_pin;

  extern uint8_t high_sol_enable_state;
  extern uint8_t high_sol_state;

  extern uint8_t lox2_state;
  extern uint8_t lox5_state;
  extern uint8_t lox_gems_state;

  extern uint8_t prop2_state;
  extern uint8_t prop5_state;
  extern uint8_t prop_gems_state;

  extern float pressurantSolenoidMonitorShuntR;
  extern LTC4151 *_pressurantSolenoidMonitor;

  void init(uint8_t numSolenoids, uint8_t * solenoidPins, uint8_t numCommands, uint8_t * commandIds, uint8_t * outputMonitorAddrs, TwoWire *wire, float shuntR, float maxExpectedCurrent, LTC4151 *pressurantMonitor, float pressurantSolMonShuntR);

  int toggleHighPressureSolenoid();
  int toggleLOX2Way();
  int toggleLOX5Way();
  int toggleLOXGems();
  int toggleProp2Way();
  int toggleProp5Way();
  int togglePropGems();
  int openHighPressureSolenoid();
  int closeHighPressureSolenoid();
  int enableHighPressureSolenoid();
  int disableHighPressureSolenoid();
  int ventLOXGems();
  int closeLOXGems();
  int ventPropaneGems();
  int closePropaneGems();
  int armLOX();
  int disarmLOX();
  int armPropane();
  int disarmPropane();
  int armAll();
  int disarmAll();
  int openLOX();
  int closeLOX();
  int openPropane();
  int closePropane();
  int LAUNCH();
  int endBurn();
  int getHPS();
  int getLox2();
  int getLox5();
  int getLoxGems();
  int getProp2();
  int getProp5();
  int getPropGems();
  void getAllStates(float *data);
  void getAllCurrents(float *data);
  void getAllVoltages(float *data);
  bool loxArmed();
  bool propArmed();


  class SolenoidCommand : public Command {

    typedef int (*func_t)();

    public:
      INA219 outputMonitor;

      SolenoidCommand(std::string name, uint8_t id, func_t openFunc, func_t closeFunc):
        Command(name, id),
        openSolenoid(openFunc),
        closeSolenoid(closeFunc)
      {}

      SolenoidCommand(std::string name, func_t openFunc, func_t closeFunc):
        Command(name),
        openSolenoid(openFunc),
        closeSolenoid(closeFunc)
      {}

      void initINA219(TwoWire *wire, uint8_t inaAddr, float shuntR, float maxExpectedCurrent) {
        outputMonitor.begin(wire, inaAddr);
        outputMonitor.configure(INA219_RANGE_16V, INA219_GAIN_40MV, INA219_BUS_RES_12BIT, INA219_SHUNT_RES_12BIT_1S);
        outputMonitor.calibrate(shuntR, maxExpectedCurrent);
      }

      void parseCommand(float *data) {
        Serial.println("Solenoid, parse command");
        Serial.flush();
        if (data[0] == 1) {
          Serial.println("open");
          openSolenoid();
        } else {
          Serial.println("close");
          closeSolenoid();
        }
      }

      void confirmation(float *data) {
        Solenoids::getAllStates(data);
      }

    private:
      func_t openSolenoid;
      func_t closeSolenoid;
  };

  extern SolenoidCommand lox_2;
  extern SolenoidCommand lox_5;
  extern SolenoidCommand lox_G;
  extern SolenoidCommand prop_2;
  extern SolenoidCommand prop_5;
  extern SolenoidCommand prop_G;

  extern SolenoidCommand high_p;
  extern SolenoidCommand high_p_enable;
  extern SolenoidCommand arm_rocket;
  extern SolenoidCommand launch;

}
#endif /* end of include guard: SOLENOIDS */
