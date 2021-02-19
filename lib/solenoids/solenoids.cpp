/*
  solenoids.h - A c++ library to interface with all of the solenoids on our rocket.
  Created by Vamshi Balanaga, Sept 15, 2020.
*/

#include "solenoids.h"

using namespace std;

namespace Solenoids {


  int lox_2_pin, lox_5_pin, lox_gems_pin;
  int prop_2_pin, prop_5_pin, prop_gems_pin;
  int high_sol_pin;

  int high_sol_state = 0;

  int lox2_state = 0;
  int lox5_state = 0;
  int lox_gems_state = 0;

  int prop2_state = 0;
  int prop5_state = 0;
  int prop_gems_state = 0;


  void init(int lox2, int lox5, int loxg, int prop2, int prop5, int propg, int high) {
    lox2_state = 0;
    lox5_state = 0;
    lox_gems_state = 0;

    prop2_state = 0;
    prop5_state = 0;
    prop_gems_state = 0;

    high_sol_state = 0;

    lox_2_pin = lox2;
    lox_5_pin = lox5;
    lox_gems_pin = loxg;

    prop_2_pin = prop2;
    prop_5_pin = prop5;
    prop_gems_pin = propg;

    high_sol_pin = high;

    digitalWrite(lox_2_pin, lox2_state);
    digitalWrite(lox_5_pin, lox5_state);
    digitalWrite(lox_gems_pin, lox_gems_state);

    digitalWrite(prop_2_pin, prop2_state);
    digitalWrite(prop_5_pin, prop5_state);
    digitalWrite(prop_gems_pin, prop_gems_state);

    digitalWrite(high_sol_pin, high_sol_state);
  }

  void getAllStates(float *data){
    data[0] = lox2_state;
    data[1] = prop2_state;
    data[2] = lox5_state;
    data[3] = prop5_state;
    data[4] = lox_gems_state;
    data[5] = prop_gems_state;
    data[6] = high_sol_state;
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

  int activateHighPressureSolenoid() {
    if(high_sol_state == 0){
      toggleHighPressureSolenoid();
    } else {
      // already active, do nothing.
    }
    return high_sol_state;
  }

  int deactivateHighPressureSolenoid() {
    if(high_sol_state == 1){
      toggleHighPressureSolenoid();
    } else {
      // already closed, do nothing.
    }
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

}
