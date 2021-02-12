#ifndef __Detect__
#define __Detect__

#include <common_fw.h>
#include <solenoids.h>



using namespace std;
namespace detectEndFlow {

  float prevPressures[2][5]; //array containing 2 arrays, which contain the previous 5 pressure values of lox, prop, respectively.
  int size=0;
  int currentState; //State 0 means both valves are open, State 1 means only one is.

//void init?

  float findAverage(int index) {
    float sum = 0;
    float avg;
    for (int i=0; i<5; i++) {
      sum += prevPressures[index][i];
    }
    avg = sum / 5;
    return avg;
  }

  void shutdown(int index) {
    //if current State is both valves open:
    if (currentState==0) {
      //if current pressure reading is lox injector pressure:
      if (index == 0) {
        Solenoids::armLOX();
        Solenoids::closeLOX();
        Solenoids::armLOX();
        Solenoids::getAllStates(farrbconvert.sensorReadings);
      } else {
        Solenoids::armPropane();
        Solenoids::closePropane();
        Solenoids::armPropane();
        Solenoids::getAllStates(farrbconvert.sensorReadings);
      }
      currentState +=1; //index == 3
      //if current state is one valve closed:
    } else if (currentState==1) {
      //if current pressure reading is lox injecor pressure
      if (index == 0) {
        Solenoids::armLOX();
        Solenoids::closeLOX();
        Solenoids::armLOX();
        Solenoids::getAllStates(farrbconvert.sensorReadings);
      } else {
        Solenoids::armPropane();
        Solenoids::closePropane();
        Solenoids::armPropane();
        Solenoids::getAllStates(farrbconvert.sensorReadings);
      }
      Solenoids::deactivateHighPressureSolenoid();
      Solenoids::ventLOXGems();
      Solenoids::ventPropaneGems();
      Solenoids::getAllStates(farrbconvert.sensorReadings);
      currentState += 1;
      //once both flow is over, reset size.
    } else if (currentState == 2) {
      size = 0;
    }
    }

  void detectPeak(float currentPressure, int recordingIndex) {
    if (size>5) {
      float average = findAverage(recordingIndex);
      if (currentPressure > 1.15 * average) {
        //initiate shutdown
        shutdown(recordingIndex);
      }
      //removing first element of previous Pressures array, adding new Pressure to the end
      copy(prevPressures[recordingIndex] + 1, prevPressures[recordingIndex] + 5, prevPressures[recordingIndex]);
      prevPressures[recordingIndex][4] = currentPressure;
      size ++;
    } else {
      prevPressures[recordingIndex][size] = currentPressure;
      size ++;
    }
  }

}
#endif
