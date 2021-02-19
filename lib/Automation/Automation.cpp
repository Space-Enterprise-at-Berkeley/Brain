/*
  Automation.h - A c++ library including automated sequences triggered by
  commands or particular events (e.g. end of flow, overpressurization)
*/

#include "Automation.h"

using namespace std;

namespace Automation {

//-----------------------Variables-----------------------

  bool _startup = false;
  int _startupPhase = 0;
  uint32_t _startupTimer;

  bool _flowing = false;

  bool _shutdown = false;
  int _shutdownPhase = 0;
  uint32_t _shutdownTimer;

 struct autoEventList* _eventList;

  /* Delays during startup sequence:
    1 - Between open pressure and open LOX Main
    2 - Between open LOX Main and open Prop Main
    3 - Between open Prop Main and close Main Valve Arm
  */
  int _startupDelays[3] = {1000, 0 ,1000};

  /* Delays during shutdown sequence:
    1 - Between arm 2-way, close high pressure, close Prop and close LOX
    2 - Between close LOX and closing Arming valve & opening Gems vent
  */
  int _shutdownDelays[2] = {0, 750};


  float prevPressures[2][5]; //array containing 2 arrays, which contain the previous 5 pressure values of lox, prop, respectively.
  int sizes[2]= {0,0};


//-----------------------Functions-----------------------

  bool init() {
    _eventList = new autoEventList;
    _eventList->maxEvents = 15; //arbitrary max of 10 events right now.
    _eventList->events = new autoEvent[_eventList->maxEvents]; 

    _eventList->length = 0;
    return true;
  }


  bool inStartup() {
    return _startup;
  }
  
  bool inFlow() {
    return _flowing;
  }
  
  bool inShutdown() {
    return _shutdown;
  }


  /* 
   * For now copies the passed into autoEvent into the eventList. Copying requires slightly more
   * overhead, but avoids dealing with allocating & deallocation memory for each event 
   */
  bool addEvent(autoEvent* e) {
    if (_eventList->length < 10) {
      (_eventList->events)[_eventList->length] = *e;
      _eventList->length++;
      Serial.println("eventList len!");
      Serial.println(_eventList->length);
      // if first event is being added then need to restart timer.
      if (_eventList->length == 1) {
        _eventList->timer = millis();
      }
      return true;
    } else {
      return false;
    }
  }

  bool removeEvent() {
    if (_eventList->length > 0) {
      //move events 1 - 9 & move to slots 0 - 8, effectively "popping" first event
      memmove(_eventList->events, _eventList->events + 1, sizeof(autoEvent)*(_eventList->maxEvents - 1));

      Automation::_eventList->length--;
    }
  }

  int state_setFlowing() {
    _startup = false;
    _flowing = true;
    Serial.println("Flow has begun");
  }

  int state_setShutdown() {
    _flowing = false;
    _shutdown = true;
  }

  int act_pressurizeTanks() {
    Solenoids::closeLOXGems();
    Solenoids::closePropaneGems();
    Solenoids::activateHighPressureSolenoid();
  }

  int act_openGems() {
    Solenoids::ventLOXGems();
    Solenoids::ventPropaneGems();
  }

  int act_armOpenLox() {
    Solenoids::armLOX();
    Solenoids::openLOX();
  }

  int act_armOpenProp() {
    Solenoids::armLOX();
    Solenoids::openPropane();
  }

  int act_armOpenBoth() {
    Solenoids::armLOX();
    Solenoids::openLOX();
    Solenoids::openPropane();
  }

  int act_armCloseProp() {
    Solenoids::armLOX();
    Solenoids::closePropane();
  }

  int act_armCloseLox() {
    Solenoids::armLOX();
    Solenoids::closeLOX();
  }

  int act_armCloseBoth() {
    Solenoids::armLOX();
    Solenoids::closeLOX();
    Solenoids::closePropane();
  }

  int beginLoxFlow() {
    autoEvent events[4];
    events[0] = {0, &(act_pressurizeTanks), false};
    events[1] = {1000, &(act_armOpenLox), false};
    events[2] = {750, &(Solenoids::disarmLOX), false};
    events[3] = {1000, &(state_setFlowing), false};
    for (int i = 0; i < 4; i++) addEvent(&events[i]);
  }

  int endLoxFlow() {
    autoEvent events[4];
    events[0] = {0, &(act_armCloseLox), false};
    events[1] = {0, &(Solenoids::deactivateHighPressureSolenoid), false};
    events[2] = {750, &(Solenoids::disarmLOX), false};
    events[3] = {0, &(act_openGems), false};
    for (int i = 0; i < 4; i++) addEvent(&events[i]);
  }

  int openLox() {
    autoEvent events[3];
    events[0] = {0, &(act_armOpenLox), false};
    events[1] = {750, &(Solenoids::disarmLOX), false};
    events[2] = {1000, &(state_setFlowing), false};
    for (int i = 0; i < 2; i++) addEvent(&events[i]);
  }

  int closeLox() {
    autoEvent e1 = autoEvent{2000, &(Solenoids::closeLOX), false};
    addEvent(&e1);
    return 1;
  }

  int beginBothFlow() {
    /* Check if rocket is in required state for a flow:
      Pressure - Closed
      LOX GEMS & Prop GEMS - Open
      Arming Valve - Closed
      LOX Main Valve & Prop Main Valve - Closed
    */
    _startup = !Solenoids::getHPS() &&
        !Solenoids::getLox2() && !Solenoids::getLox5() && !Solenoids::getProp5();
    if (_startup) {
      Serial.println("Eureka-1 is in Startup");

      autoEvent events[4];
      events[0] = {0, &(act_pressurizeTanks), false};
      events[1] = {1000, &(act_armOpenBoth), false};
      events[2] = {750, &(Solenoids::disarmLOX), false};
      events[3] = {1000, &(state_setFlowing), false};
      //TODO @Ben: after ~1sec delay change startup to false & shutdown to true so shutdownDetection can start
      for (int i = 0; i < 4; i++) addEvent(&events[i]);
    }
    return -1;
  }

  int endBothFlow() {
    _flowing = false;
    _shutdown = true;
    Serial.println("Eureka-1 is in Shutdown");

    autoEvent events[4];
    events[0] = {0, &(act_armCloseBoth), false};
    events[1] = {0, &(Solenoids::deactivateHighPressureSolenoid), false};
    events[2] = {750, &(Solenoids::disarmLOX), false};
    events[3] = {0, &(act_openGems), false};
    //TODO: set shutdown to be false
    for (int i = 0; i < 4; i++) addEvent(&events[i]);

    return -1;
  }

  // Responds to initial command to begin/end flow
  void flowConfirmation(float *data) {
    data[0] = _startup ? 1 : 0;
    data[1] = _shutdown ? 1 : 0;
    data[2] = -1;
  }


  float findAverage(int index) {
    float sum = 0;
    float avg;
    for (int i=0; i<5; i++) {
      sum += prevPressures[index][i];
    }
    avg = sum / 5;
    return avg;
  }

  /*
   * recordingIndex:
   *   0 - LOX
   *   1 - Propane
   */ 
  void autoShutdown(int index) {
    Serial.print("Shutting down valve ");
    Serial.println(index);
    if (index == 0) { // Lox is out
      if (Solenoids::getProp5()) { // Prop is still open
        autoEvent events[2];
        events[0] = {0, &(act_armCloseLox), false};  //TODO @Ben: act_armCloseLox is crashing
        events[1] = {750, &(Solenoids::disarmLOX), false};
        for (int i = 0; i < 2; i++) addEvent(&events[i]);

      } else { // Prop is already closed, shutdown
        _flowing = false;
        autoEvent events[4];
        events[0] = {0, &(act_armCloseLox), false};
        events[1] = {0, &(Solenoids::deactivateHighPressureSolenoid), false};
        events[2] = {750, &(Solenoids::disarmLOX), false};
        events[3] = {0, &(act_openGems), false};
        for (int i = 0; i < 4; i++) addEvent(&events[i]);
      }
    } else { // Prop is out
      if (Solenoids::getProp5()) { // Lox is still open
        autoEvent events[2];
        events[0] = {0, &(act_armCloseProp), false};
        events[1] = {750, &(Solenoids::disarmLOX)};
        for (int i = 0; i < 2; i++) addEvent(&events[i]);

      } else { // Lox is already closed, shutdown
        _flowing = false;
        autoEvent events[4];
        events[0] = {0, &(act_armCloseProp), false};
        events[1] = {0, &(Solenoids::deactivateHighPressureSolenoid), false};
        events[2] = {750, &(Solenoids::disarmLOX), false};
        events[3] = {0, &(act_openGems), false};
        for (int i = 0; i < 4; i++) addEvent(&events[i]);
      }
    }
      
  }

  void detectPeaks(float loxInjector, float propInjector) {    

    if (Solenoids::getLox5()) { //if LOX Main valve open
      detectPeak(loxInjector, 0);
    }
    if (Solenoids::getProp5()) { //if Prop Main valve open
      detectPeak(propInjector, 1);
    }
  }


  /*
   * recordingIndex:
   *   0 - LOX
   *   1 - Propane
   *   Assumes this is only called if the main given valve is actually open
   */ 
  void detectPeak(float currentPressure, int recordingIndex) {


    if (sizes[recordingIndex]==5) {

      float average = findAverage(recordingIndex);
      if (currentPressure > 1.20 * average) {

        Serial.print("Spike detected - "); Serial.print(currentPressure); Serial.print(" vs. avg. "); Serial.println(average);
        Serial.print("[");
        for (int i = 0; i < 5; i++) {
          Serial.print(Automation::prevPressures[recordingIndex][i]);
          Serial.print(",");
        }
        Serial.println("]");

        //initiate shutdown
        autoShutdown(recordingIndex);
      }
      //removing first element of previous Pressures array, adding new Pressure to the end
      memmove(prevPressures[recordingIndex], prevPressures[recordingIndex] + 1, sizeof(float)*(4));
      prevPressures[recordingIndex][4] = currentPressure;

      
    } else {
      prevPressures[recordingIndex][sizes[recordingIndex]] = currentPressure;
      sizes[recordingIndex]++;
    }
  }

  
} //Automation