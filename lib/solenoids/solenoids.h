/*
  solenoids.h - A c++ library to interface with all of the solenoids on our rocket.
  Created by Vamshi Balanaga, Sept 15, 2020.
*/
#ifndef __SOLENOIDS__
#define __SOLENOIDS__

#include <Arduino.h>

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

  void getAllStates(float *data);

  bool loxArmed();

  bool propArmed();

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

}


#endif /* end of include guard: SOLENOIDS */
