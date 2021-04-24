#include <solenoids.h>
#include <command.h>
#include <Analog_Thermocouple.h>
#include <Cryo_Thermocouple.h>
#include <tempController.h>
#include "common_fw.h"
#include <ADS1219.h>
#include "Automation.h"
#include "realTimeClock.h"

#define FLIGHT_BRAIN_ADDR 0x00

std::string str_file_name;
const char * file_name;

#ifdef ETH
IPAddress ip(10, 0, 0, 177); // dependent on local network
#endif

const uint8_t numCryoTherms = 4;
// therm[2] = lox adapter tree pt, therm[3] = lox adapter tree gems
// ADDR = GND, VDD, 10k & 4.3K, 10K & 13K
uint8_t cryoThermAddrs[numCryoTherms] = {0x60, 0x67, 0x62, 0x64};
_themotype cryoTypes[numCryoTherms] = {MCP9600_TYPE_J, MCP9600_TYPE_T, MCP9600_TYPE_T, MCP9600_TYPE_K};
Adafruit_MCP9600 _cryo_boards[numCryoTherms];
float cryoReadsBackingStore[numCryoTherms];

const uint8_t numADCSensors = 2;
uint8_t ADSAddrs[numADCSensors] = {0b1001010, 0b1001000};
uint8_t adcDataReadyPins[numADCSensors] = {29, 28};
ADS1219 ads[numADCSensors];
ADC *adsPointers[numADCSensors];

const uint8_t numAnalogThermocouples = 1;
uint8_t thermAdcIndices[numAnalogThermocouples] = {1};
uint8_t thermAdcChannels[numAnalogThermocouples] = {2};

const uint8_t numPressureTransducers = 7;
uint8_t ptAdcIndices[numPressureTransducers] = {0, 0, 0, 0, 1, 1, 1}; //not using 1-0 or 1-3
uint8_t ptAdcChannels[numPressureTransducers] = {0, 1, 2, 3, 2, 1, 3};
uint32_t ptTypes[numPressureTransducers] = {1000, 1000, 1000, 1000, 5000, 1000, 1000};

const uint8_t numSensors = 6;
sensorInfo sensors[numSensors];

const uint8_t numSolenoids = 7;   // l2, l5, lg, p2, p5, pg, h
uint8_t solenoidPins[numSolenoids] = {0,  2,  4,  1,  3,  5, 6};
const uint8_t numSolenoidCommands = 9;    //       l2, l5, lg, p2, p5, pg,  h, arm, launch
uint8_t solenoidCommandIds[numSolenoidCommands] = {20, 21, 22, 23, 24, 25, 26,  27, 28};

const float batteryMonitorShuntR = 0.002; // ohms
const float batteryMonitorMaxExpectedCurrent = 10; // amps

const uint8_t loxAdapterPTHeaterPin = 9;
const uint8_t loxGemsHeaterPin = 7;

HeaterCommand loxPTHeater("LOX PT Heater", 40, 10, 2, loxAdapterPTHeaterPin); // setPoint = 10 C, alg = PID
HeaterCommand loxGemsHeater("LOX Gems Heater", 41, 10, 2, loxGemsHeaterPin); // setPoint = 10C, alg = PID

AutomationSequenceCommand fullFlow("Perform Flow", 29, &(Automation::beginBothFlow), &(Automation::endBothFlow));
AutomationSequenceCommand loxFlow("Perform LOX Flow", 30, &(Automation::beginLoxFlow), &(Automation::endLoxFlow));

const uint8_t numCommands = 13;
Command *backingStore[numCommands] = {&Solenoids::lox_2,  &Solenoids::lox_5,  &Solenoids::lox_G,
                                        &Solenoids::prop_2, &Solenoids::prop_5, &Solenoids::prop_G,
                                        &Solenoids::high_p, &Solenoids::arm_rocket, &Solenoids::launch,
                                        &fullFlow, &loxFlow, &loxPTHeater, &loxGemsHeater};
CommandArray commands(numCommands, backingStore);

namespace config {
  void setup() {
    RealTimeClock::init();
    
    debug("File Name:");
    str_file_name = "E1_Coldflow_" + RealTimeClock::getFileTime() + ".txt";
    file_name = str_file_name.c_str();
    debug(file_name);

    debug("Initializing ADCs");
    for (int i = 0; i < numADCSensors; i++) {
      ads[i].init(adcDataReadyPins[i], ADSAddrs[i], &Wire);
      ads[i].setConversionMode(SINGLE_SHOT);
      ads[i].setVoltageReference(REF_EXTERNAL);
      ads[i].setGain(ONE);
      ads[i].setDataRate(1000);
      pinMode(adcDataReadyPins[i], INPUT_PULLUP);
      adsPointers[i] = &ads[i];
      // ads[i].calibrate();
    }

    debug("Initializing sensors");
    // the ordering in this array defines order of operation, not id
    sensors[0] = {"All Pressure",  FLIGHT_BRAIN_ADDR, 1, 1};
    sensors[1] = {"Battery Stats", FLIGHT_BRAIN_ADDR, 2, 3};
    sensors[2] = {"Cryo Temps",      FLIGHT_BRAIN_ADDR, 4, 3};
    sensors[3] = {"Lox PT Temperature",   FLIGHT_BRAIN_ADDR, 0, 4}; //&(testTempRead)}, //&(Thermocouple::readTemperatureData)},
    sensors[4] = {"Number Packets Sent", FLIGHT_BRAIN_ADDR, 5, 10};
    sensors[5] = {"LOX Gems Temp", FLIGHT_BRAIN_ADDR, 6, 4};

    pinMode(loxAdapterPTHeaterPin, OUTPUT);
    pinMode(loxGemsHeaterPin, OUTPUT);
  }
}
