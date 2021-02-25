/*
  tempControl.cpp - A c++ algorithm that maintains the temperature at a given set point.
  Two versions: One very simple, naive algorithm. (Needs to be empirically tested to verify).
  Another, more complex control loop that should reject all disturbances quite well.
  Created by Vamshi Balanaga, Oct 7, 2020.
*/
#include "tempController.h"

TempController::TempController(int tempSetPoint, int algorithmChoice, int heaterPin):
  _heaterPin(heaterPin),
  _algorithmChoice(algorithmChoice),
  _setPointTemp(tempSetPoint)
 {
  if (algorithmChoice > 2 || algorithmChoice < 0) {
    exit(1);
  }
}

int TempController::calculateOutput(float currTemp) {
  int voltageOut = 0;
  if (_algorithmChoice == 1) { // naive
    voltageOut = (int)(k_p * (_setPointTemp - currTemp));
    return max(0, min(255, voltageOut));
  } else if (_algorithmChoice == 2) { // linear control theory solution
    return controller->calculate(_setPointTemp, currTemp);
  } else {
    return -1;
  }
}

int TempController::overwriteControlValue(uint16_t setValue) {
  humanOverride = (setValue != 300);
  humanSpecifiedValue = setValue;
}

void TempController::confirmationPacket(float *data){
  data[0] = humanOverride;
  data[1] = (humanOverride)? humanSpecifiedValue : -1;
  data[2] = -1;
}

float TempController::controlTemp(float currTemp) {
  _heaterOutput = (humanOverride) ? humanSpecifiedValue : calculateOutput(currTemp);
  analogWrite(_heaterPin, _heaterOutput);
  return _heaterOutput;
}
