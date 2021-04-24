
#include "common_fw.h"
#include "realTimeClock.h"
#include <recovery.h>

#define FLIGHT_BRAIN_ADDR 0x00

const uint8_t drogue_pin = 9;
const uint8_t main_chute_pin = 10;

std::string str_file_name;
const char * file_name;

const uint8_t numSensors = 9;
sensorInfo sensors[numSensors];

// Constants
const float batteryMonitorShuntR = 0.002; // ohms
const float batteryMonitorMaxExpectedCurrent = 10; // amps

const double altVar = 0.5;
const double accVar = 0.5;
const double avgSampleRate = 20e-3;

double initAlt;
double initAcc_z;

const uint8_t recoverAckId = 10;

double mainChuteDeployLoc = 300; // m above start location.

namespace config {
  void setup() {
    debug("File Name:");
    str_file_name = "LAD4_1_" + RealTimeClock::getFileTime() + ".txt";
    file_name = str_file_name.c_str();
    debug(file_name);

    //debug("Initializing sensors", DEBUG);
    // the ordering in this array defines order of operation, not id
    sensors[0] = {"Barometer", FLIGHT_BRAIN_ADDR, 13, 1};
    sensors[1] = {"IMU Acceleration",   FLIGHT_BRAIN_ADDR, 14, 1};
    sensors[2] = {"IMU Orientation", FLIGHT_BRAIN_ADDR, 15, 1};
    sensors[3] = {"Battery Stats", FLIGHT_BRAIN_ADDR, 2, 3};
    sensors[4] = {"GPS Lat Long",  FLIGHT_BRAIN_ADDR, 11, 3};
    sensors[5] = {"GPS AUX",      FLIGHT_BRAIN_ADDR, 12, 3};
    sensors[6] = {"Number Packets Sent", FLIGHT_BRAIN_ADDR, 5, 10};
    sensors[7] = {"Recovery Acknowledgement", FLIGHT_BRAIN_ADDR, 10, 25};
    sensors[8] = {"Flight state", FLIGHT_BRAIN_ADDR, 9, 25};

    pinMode(drogue_pin, OUTPUT);
    pinMode(main_chute_pin, OUTPUT);
  }
}
