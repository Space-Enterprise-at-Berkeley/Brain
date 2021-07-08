/*
  Automation.cpp - A c++ library including automated sequences triggered by
  commands or particular events (e.g. end of flow, overpressurization)
*/


#ifndef __Automation__
#define __Automation__

#include <Arduino.h>
#include <solenoids.h>
#include <command.h>

using namespace std;

namespace Automation {
//-----------------------Variables-----------------------


// typedef enum {
//     ERROR = -1,
//     PRESS = 0,
//     FLOWING = 1,
//     SHUTOFF = 2,
//     DEPRESSURIZE = 3
//   } cold_flow_state_t;

// typedef enum {
//     LOX_ONLY = 0,
//     BOTH_COLD = 1,
//     HOT = 2
//   } flow_type_t;
//
// typedef enum {
//     ABORT = -2,
//     ERROR = -1,
//     ON_PAD = 0,
//     PRESS = 1,
//     IGNITED = 2,
//     LOX_FLOWING = 3,
//     PROP_FLOWING = 4,
//     BOTH_FLOWING = 5,
//     SHUTOFF = 6,
//     DEPRESSURIZE = 7
//   } flow_state_t;
//
//   extern flow_type_t flowtype;
//   extern flow_state_t flowstate;

  extern int _autoEventTracker;

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

extern struct autoEventList _eventList;


//------------------Function Definitions-----------------


  bool init();

  // Relating to State of the Rocket
  bool inStartup();
  bool inFlow();
  bool inShutdown();

  bool addEvent(autoEvent* e);
  bool addEvent(int duration, int (*action)(), bool report);
  bool removeEvent();

  // Actions that can be taken/combined as desired
  int beginBothFlow();
  int endBothFlow();

  int beginHotfire();
  int endHotfire();

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
  int act_closeGems();
  int act_armOpenLox();
  int act_armOpenProp();
  int act_armOpenBoth();
  int act_armCloseLox();
  int act_armCloseProp();
  int act_armCloseBoth();
  int act_depressurize();

  // int beginPropFlow();
  // int endPropFlow();
  // int openProp();
  // int closeProp();

  void flowConfirmation(float *data);
  void flowStatus(float *data);

  // Automatic Detection
  float findAverage(int index);
  void autoShutdown(int index);
  void detectPeak(float currentPressure, int recordingIndex);
  void detectPeaks(float loxInjector, float propInjector);
}

class AutomationSequenceCommand : public Command {

  typedef int (*func_t)();

  public:
    AutomationSequenceCommand(std::string name, uint8_t id, func_t startFunc, func_t endFunc):
      Command(name, id),
      startSequence(startFunc),
      endSequence(endFunc)
    {}

    AutomationSequenceCommand(std::string name, func_t startFunc, func_t endFunc):
      Command(name),
      startSequence(startFunc),
      endSequence(endFunc)
    {}

    void parseCommand(float *data) {
      if (data[0] == 1)
        startSequence();
      else
        endSequence();
    }

    void confirmation(float *data) {
      Automation::flowConfirmation(data);
    }

  private:
    func_t startSequence;
    func_t endSequence;
};

#endif
