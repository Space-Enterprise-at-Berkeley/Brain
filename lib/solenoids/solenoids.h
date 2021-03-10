/*
  solenoids.h - A c++ library to interface with all of the solenoids on our rocket.
  Created by Vamshi Balanaga, Sept 15, 2020.
*/
#ifndef __SOLENOIDS__
#define __SOLENOIDS__

#include <Arduino.h>
#include <actuator.h>

using namespace std;

namespace Solenoids {


  extern int lox_2_pin, lox_5_pin, lox_gems_pin;
  extern int prop_2_pin, prop_5_pin, prop_gems_pin;
  extern int high_sol_pin;

  extern int high_sol_state;

  extern int lox2_state;
  extern int lox5_state;
  extern int lox_gems_state;

  extern int prop2_state;
  extern int prop5_state;
  extern int prop_gems_state;

  void init(int lox2, int lox5, int loxg, int prop2, int prop5, int propg, int high);

  int toggleHighPressureSolenoid();
  int toggleLOX2Way();
  int toggleLOX5Way();
  int toggleLOXGems();
  int toggleProp2Way();
  int toggleProp5Way();
  int togglePropGems();

  int activateHighPressureSolenoid();
  int deactivateHighPressureSolenoid();
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
  bool loxArmed();
  bool propArmed();


  class SolenoidActuator : public Actuator {

    typedef int (*func_t)();

    public:
      SolenoidActuator(std::string name, uint8_t id, func_t openFunc, func_t closeFunc):
        Actuator(name, id),
        openSolenoid(openFunc),
        closeSolenoid(closeFunc)
      {}

      SolenoidActuator(std::string name, func_t openFunc, func_t closeFunc):
        Actuator(name),
        openSolenoid(openFunc),
        closeSolenoid(closeFunc)
      {}

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

  extern SolenoidActuator lox_2;
  extern SolenoidActuator lox_5;
  extern SolenoidActuator lox_G;
  extern SolenoidActuator prop_2;
  extern SolenoidActuator prop_5;
  extern SolenoidActuator prop_G;

  extern SolenoidActuator high_p;
  extern SolenoidActuator arm_rocket;
  extern SolenoidActuator launch;

}
#endif /* end of include guard: SOLENOIDS */
