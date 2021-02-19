/*
  Automation.cpp - A c++ library including automated sequences triggered by
  commands or particular events (e.g. end of flow, overpressurization)
*/


#ifndef __Automation__
#define __Automation__

#include <Arduino.h>
#include <solenoids.h>


using namespace std;

namespace Automation {

//-----------------------Variables-----------------------

  extern bool _startup;
  extern int _startupPhase;
  extern uint32_t _startupTimer;

  extern bool _flowing;

  extern bool _shutdown;
  extern int _shutdownPhase;
  extern uint32_t _shutdownTimer;


  /* Delays during startup sequence:
    1 - Between open pressure and open LOX Main
    2 - Between open LOX Main and open Prop Main
    3 - Between open Prop Main and close Main Valve Arm
  */
  extern int _startupDelays[3];

  /* Delays during shutdown sequence:
    1 - Between arm 2-way, close high pressure, close Prop and close LOX
    2 - Between close LOX and closing Arming valve & opening Gems vent
  */
  extern int _shutdownDelays[2];


  // Endflow Detection
  extern float prevPressures[2][5]; //array containing 2 arrays, which contain the previous 5 pressure values of lox, prop, respectively.
  extern int sizes[2];


//------------------------Structs------------------------

/*
 * Stores details about an event used to determine what to do & when
 */
struct autoEvent {
  int duration;
  int (*action)();
  bool report;
};


struct autoEventList {
  autoEvent* events;
  int length;
  int maxEvents;
  int timer;
};

extern struct autoEventList* _eventList;


//------------------Function Definitions-----------------

  bool init();

  // Relating to State of the Rocket
  bool inStartup();
  bool inFlow();
  bool inShutdown();

  bool addEvent(autoEvent* e);
  bool removeEvent();

  // Actions that can be taken/combined as desired
  int beginBothFlow();
  int endBothFlow();

  //event scheduling functions
  int openLox();
  int closeLox();
  int beginLoxFlow();
  int endLoxFlow();

  // state functions to set appropriate state variables
  int state_setFlowing();

  // action functions that combine multiple valve openings/closing simultaneously
  int act_pressurizeTanks();
  int act_openGems();
  int act_armOpenLox();
  int act_armOpenProp();
  int act_armOpenBoth();
  int act_armCloseLox();
  int act_armCloseProp();
  int act_armCloseBoth();

  // int beginPropFlow();
  // int endPropFlow();
  // int openProp();
  // int closeProp();
  
  void flowConfirmation(float *data);
  
  // Automatic Detection
  float findAverage(int index);
  void autoShutdown(int index);
  void detectPeak(float currentPressure, int recordingIndex);
  void detectPeaks(float loxInjector, float propInjector);


}

#endif
