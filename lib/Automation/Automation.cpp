/*
  Automation.h - A c++ library including automated sequences triggered by
  commands or particular events (e.g. end of flow, overpressurization)
*/

#include "Automation.h"

using namespace std;

namespace Automation {

//-----------------------Variables-----------------------

  int _autoEventTracker = 0;

  bool _startup = false;
  int _startupPhase = 0;
  uint32_t _startupTimer;

  bool _flowing = false;
  bool _shutdown = false;
  int _shutdownPhase = 0;
  uint32_t _shutdownTimer;

  // flow_type_t flowtype;
  // flow_state_t flowstate = ON_PAD;

 const uint8_t maxNumEvents = 15;
 struct autoEventList _eventList;
 autoEvent events[maxNumEvents];

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
    //_eventList = new autoEventList;
    _eventList.maxEvents = maxNumEvents; //arbitrary max of 10 events right now.
    _eventList.events = new autoEvent[maxNumEvents];

    for (int i = 0; i < maxNumEvents; i++) {
      _eventList.events[i].duration = 750 + i;
      _eventList.events[i].action = &(Solenoids::armLOX);
      _eventList.events[i].report = false;
    }

    _eventList.length = 0;
    Serial.println(_eventList.length);
    Serial.flush();
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
    Serial.println("add Event; len: " + String(_eventList.length));
    Serial.flush();
    if (_eventList.length < maxNumEvents) {
      Serial.println("duration: " + e->duration);
      Serial.println("report: " + e->report);
      //_eventList.events[_eventList.length] = *e;
      memmove(&_eventList.events[_eventList.length], e, sizeof(autoEvent));
      _eventList.length++;
      Serial.println("eventList len!");
      Serial.println(_eventList.length);
      Serial.flush();
      // if first event is being added then need to restart timer.
      if (_eventList.length == 1) {
        _eventList.timer = millis();
      }

      return true;
    } else {
      return false;
    }
  }

  bool addEvent(int duration, int (*action)(), bool report) {
    Serial.println("add Event; len: " + String(_eventList.length));
    Serial.flush();
    if (_eventList.length < maxNumEvents) {
      Serial.println("duration: " + String(duration));
      Serial.println("report: " + String(report));
      autoEvent *tmpE = new autoEvent{duration, action, report};
      //_eventList.events[_eventList.length] = new autoEvent{duration, action, report};
      memmove(&_eventList.events[_eventList.length], tmpE, sizeof(autoEvent));
      free(tmpE);

      _eventList.length++;
      Serial.println("eventList len!");
      Serial.println(_eventList.length);
      Serial.flush();
      // if first event is being added then need to restart timer.
      if (_eventList.length == 1) {
        _eventList.timer = millis();
      }

      return true;
    } else {
      return false;
    }
  }

  bool removeEvent() {
    Serial.println("remvoe Event");
    Serial.flush();
    if (_eventList.length > 0) {
      //move events 1 - 9 & move to slots 0 - 8, effectively "popping" first event
      memmove(_eventList.events, _eventList.events + 1, sizeof(autoEvent)*(_eventList.maxEvents - 1));

      Automation::_eventList.length--;
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
    // flowstate = PRESS;
    Solenoids::closeLOXGems();
    Solenoids::closePropaneGems();
    Solenoids::openHighPressureSolenoid();
  }

  int act_openGems() {
    Solenoids::ventLOXGems();
    Solenoids::ventPropaneGems();
  }

   int act_closeGems() {
    Solenoids::closeLOXGems();
    Solenoids::closePropaneGems();
    return 0;
  }

  int act_armOpenLox() {
    // flowstate = (flowstate == PROP_FLOWING)? BOTH_FLOWING : LOX_FLOWING;
    Solenoids::armLOX();
    Solenoids::openLOX();
  }

  int act_armOpenProp() {
    // flowstate = (flowstate == LOX_FLOWING)? BOTH_FLOWING : PROP_FLOWING;
    Solenoids::armLOX();
    Solenoids::openPropane();
  }

  int act_armOpenBoth() {
    // flowstate = BOTH_FLOWING;
    Solenoids::armLOX();
    Solenoids::openLOX();
    Solenoids::openPropane();
  }

  int act_armCloseProp() {
    // flowstate = (flowstate == BOTH_FLOWING)? LOX_FLOWING : SHUTOFF;
    Solenoids::armLOX();
    Solenoids::closePropane();
  }

  int act_armCloseLox() {
    // flowstate = (flowstate == BOTH_FLOWING)? PROP_FLOWING : SHUTOFF;
    Solenoids::armLOX();
    Solenoids::closeLOX();
  }

  int act_armCloseBoth() {
    // flowstate = SHUTOFF;
    Solenoids::armLOX();
    Solenoids::closeLOX();
    Solenoids::closePropane();
  }

  int act_depressurize() {
    // flowstate = DEPRESSURIZE;
    Solenoids::disarmLOX();
    Solenoids::closeHighPressureSolenoid();
    Solenoids::ventLOXGems();
    Solenoids::ventPropaneGems();
  }

  int beginLoxFlow() {
    // flowtype = LOX_ONLY;
    // autoEvent events[4];
    events[0] = {0, &(act_pressurizeTanks), false};
    events[1] = {1000, &(act_armOpenLox), false};
    events[2] = {750, &(Solenoids::disarmLOX), false};
    events[3] = {1000, &(state_setFlowing), false};
    for (int i = 0; i < 4; i++) addEvent(&events[i]);
  }

  int endLoxFlow() {
    // autoEvent events[4];
    events[0] = {0, &(act_armCloseLox), false};
    events[1] = {0, &(Solenoids::closeHighPressureSolenoid), false};
    events[2] = {750, &(Solenoids::disarmLOX), false};
    events[3] = {0, &(act_openGems), false};
    for (int i = 0; i < 4; i++) addEvent(&events[i]);
  }

  int openLox() {
    // autoEvent events[3];
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
    #if DEBUG
      Serial.println("begin both flow");
      Serial.flush();
    #endif
    // flowtype = BOTH_COLD;
    Serial.println("eventlist len: " + String(_eventList.length));
    Serial.flush();
    _startup = !Solenoids::getHPS() &&
        !Solenoids::getLox2() && !Solenoids::getLox5() && !Solenoids::getProp5();
    Serial.println("startup: " + String(_startup));
    if (_startup) {
      Serial.println("Eureka-1 is in Startup");
      Serial.flush();

      _autoEventTracker = 0;

      _startupTimer = millis();

      // autoEvent events[4];
      // events[0] = {0, &(act_pressurizeTanks), false};
      // addEvent(0, &(act_pressurizeTanks), false);
      // Serial.println("add press to list");
      // Serial.flush();
      // // addEvent(&events[0]);
      // Serial.println("add event press");
      // Serial.flush();
      // // events[1] = {1000, &(act_armOpenBoth), false};
      // addEvent(1000, &(act_armOpenBoth), false);
      // Serial.println("add arm to list");
      // Serial.flush();
      // // addEvent(&events[1]);

      // // events[2] = {750, &(Solenoids::disarmLOX), false};
      // addEvent(750, &(Solenoids::disarmLOX), false);
      // Serial.println("add disarm to list");
      // Serial.flush();
      // // addEvent(&events[2]);

      // // events[3] = {1000, &(state_setFlowing), false};
      // addEvent(1000, &(state_setFlowing), false);
      // Serial.println("add set state flowing");
      // Serial.flush();
      // addEvent(&events[3]);

      //TODO @Ben: after ~1sec delay change startup to false & shutdown to true so shutdownDetection can start
      // for (int i = 0; i < 4; i++) addEvent(&events[i]);
    } else {
    #if DEBUG
      Serial.println("not startup");
      Serial.flush();
    #endif
  }
    return -1;
  }

  int endBothFlow() {
    _startup = false;
    _flowing = false;
    _shutdown = true;

    _autoEventTracker = 8;

    Serial.println("Eureka-1 is in Shutdown");

    // autoEvent events[4];
    // events[0] = {0, &(act_armCloseBoth), false};
    // events[1] = {0, &(Solenoids::closeHighPressureSolenoid), false};
    // events[2] = {750, &(Solenoids::disarmLOX), false};
    // events[3] = {0, &(act_openGems), false};
    // //TODO: set shutdown to be false
    // for (int i = 0; i < 4; i++) addEvent(&events[i]);

    return -1;
  }

  int beginHotfire() {
    /* Check if rocket is in required state for hotfire:
      Pressure - Closed
      LOX GEMS & Prop GEMS - Open
      Arming Valve - Closed
      LOX Main Valve & Prop Main Valve - Closed
    */
    // flowtype = HOT;
    _startup = !Solenoids::getHPS() &&
        !Solenoids::getLox2() && !Solenoids::getLox5() && !Solenoids::getProp5();
    if (_startup) {
      Serial.println("Ignition");

      autoEvent events[6];
      events[0] = {0, &(act_pressurizeTanks), false};
      events[1] = {1000, &(Solenoids::armAll), false};
      // igniter
      events[2] = {1200, &(act_armOpenLox), false};
      events[3] = {127, &(act_armOpenProp), false};
      events[4] = {500, &(Solenoids::disarmLOX), false};
      events[5] = {300, &(state_setFlowing), false};

      for (int i = 0; i < 6; i++) addEvent(&events[i]);
    } else {
      // flowstate = ERROR;
    }

    Serial.println("If no fire, PUSH RED BUTTON");
    return -1;
  }

  int endHotfire() {
    _flowing = false;
    _shutdown = true;
    Serial.println("Eureka-1 is in Shutdown");

    autoEvent events[3];
    events[0] = {0, &(act_armCloseProp), false};
    events[1] = {200, &(act_armCloseLox), false};
    events[2] = {0, &(act_depressurize), false};

    for (int i = 0; i < 3; i++) addEvent(&events[i]);

    _shutdown = false;

    Serial.println("Eureka-1 is secure");

    return -1;
  }

  // Responds to initial command to begin/end flow
  void flowConfirmation(float *data) {
    data[0] = _startup ? 1 : 0;
    data[1] = _shutdown ? 1 : 0;
    data[2] = -1;
  }

  void flowStatus(float *data) {
    data[0] = 0;//flowtype;
    data[1] = 0;//flowstate;
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
        events[1] = {0, &(Solenoids::closeHighPressureSolenoid), false};
        events[2] = {750, &(Solenoids::disarmLOX), false};
        events[3] = {0, &(act_openGems), false};
        for (int i = 0; i < 4; i++) addEvent(&events[i]);
      }
    } else { // Prop is out
      if (Solenoids::getLox5()) { // Lox is still open
        autoEvent events[2];
        events[0] = {0, &(act_armCloseProp), false};
        events[1] = {750, &(Solenoids::disarmLOX)};
        for (int i = 0; i < 2; i++) addEvent(&events[i]);

      } else { // Lox is already closed, shutdown
        _flowing = false;
        autoEvent events[4];
        events[0] = {0, &(act_armCloseProp), false};
        events[1] = {0, &(Solenoids::closeHighPressureSolenoid), false};
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

    if (sizes[recordingIndex] == 5) {

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
