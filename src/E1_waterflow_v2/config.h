#include <solenoids.h>
#include <Analog_Thermocouple.h>
#include "common.h"
#include <ADS8167.h>
#include <INA219.h>
#include <GpioExpander.h>
#include <LTC4151.h>
#include <Automation.h>
#include <command.h>

#define FLIGHT_BRAIN_ADDR 0x00

std::string str_file_name = "E1_waterflow_v2.txt";
const char * file_name = str_file_name.c_str();

#ifdef ETH
IPAddress ip(10, 0, 0, 42); // dependent on local network
#endif

const uint8_t numADCSensors = 2;
uint8_t adcCSPins[numADCSensors] = {37, 36};
uint8_t adcDataReadyPins[numADCSensors] = {26, 27};
uint8_t adcAlertPins[numADCSensors] = {9, 10};
ADS8167 ads[numADCSensors];
ADC * adsPointers[numADCSensors];

const int numAnalogThermocouples = 1;
uint8_t thermAdcIndices[numAnalogThermocouples] = {0};
uint8_t thermAdcChannels[numAnalogThermocouples] = {4};

const uint8_t numPressureTransducers = 8;
uint8_t ptAdcIndices[numPressureTransducers] = {0, 0, 0, 0, 1, 1, 1, 1};
uint8_t ptAdcChannels[numPressureTransducers] = {0, 1, 2, 3, 4, 5, 6, 7};
uint32_t ptTypes[numPressureTransducers] = {1000, 1000, 1000, 1000, 5000, 1000, 1000, 1000};
const uint8_t pressurantIdx = 5;
const uint8_t loxDomeIdx = 6;
const uint8_t propDomeIdx = 7;

const uint8_t numPowerSupplyMonitors = 2;       //12v  , 8v
uint8_t powSupMonAddrs[numPowerSupplyMonitors] = {0x44, 0x45};
INA219 powerSupplyMonitors[numPowerSupplyMonitors];
INA * powSupMonPointers[numPowerSupplyMonitors];

uint8_t battMonINAAddr = 0x43;

const uint8_t numGPIOExpanders = 1;
uint8_t gpioExpAddr[numGPIOExpanders] = {TCA6408A_ADDR1};
int8_t gpioExpIntPin[numGPIOExpanders] = {-1};
GpioExpander heaterCtl(gpioExpAddr[0], gpioExpIntPin[0], &Wire);

const uint8_t numSensors = 4;
sensorInfo sensors[numSensors];

const uint8_t numSolenoids = 8;   // l2, l5, lg, p2, p5, pg, h, h enable
uint8_t solenoidPins[numSolenoids] = {5,  3,  1,  4,  2,  0, 6, 39};
const uint8_t numSolenoidCommands = 10;    //       l2, l5, lg, p2, p5, pg,  h, arm, launch , h enable
uint8_t solenoidCommandIds[numSolenoidCommands] = {20, 21, 22, 23, 24, 25, 26,  27, 28     , 31};

const float batteryMonitorShuntR = 0.002; // ohms
const float batteryMonitorMaxExpectedCurrent = 10; // amps

const float powerSupplyMonitorShuntR = 0.010; // ohms
const float powerSupplyMonitorMaxExpectedCurrent = 5; // amps

AutomationSequenceCommand fullFlow("Perform Flow", 29, &(Automation::beginBothFlow), &(Automation::endBothFlow));
AutomationSequenceCommand loxFlow("Perform LOX Flow", 30, &(Automation::beginLoxFlow), &(Automation::endLoxFlow));

const uint8_t numCommands = 12;
Command *backingStore[numCommands] = {&Solenoids::lox_2,  &Solenoids::lox_5,  &Solenoids::lox_G,
                                        &Solenoids::prop_2, &Solenoids::prop_5, &Solenoids::prop_G,
                                        &Solenoids::high_p, &Solenoids::high_p_enable, &Solenoids::arm_rocket, &Solenoids::launch,
                                        &fullFlow, &loxFlow};
CommandArray commands(numCommands, backingStore);

namespace config {
  void setup() {
    debug("Initializing ADCs");
    for (int i = 0; i < numADCSensors; i++) {
      ads[i].init(&SPI, adcCSPins[i], adcDataReadyPins[i], adcAlertPins[i]);
      ads[i].setManualMode();
      ads[i].setAllInputsSeparate();
      pinMode(adcDataReadyPins[i], INPUT_PULLUP);
      adsPointers[i] = &ads[i];
      // ads[i]->calibrate();
    }

    debug("Initializing Power Supply monitors");
    for (int i = 0; i < numPowerSupplyMonitors; i++) {
        powerSupplyMonitors[i].begin(&Wire, powSupMonAddrs[i]);
        powerSupplyMonitors[i].configure(INA219_RANGE_16V, INA219_GAIN_40MV, INA219_BUS_RES_12BIT, INA219_SHUNT_RES_12BIT_1S);
        powerSupplyMonitors[i].calibrate(powerSupplyMonitorShuntR, powerSupplyMonitorMaxExpectedCurrent);
        powSupMonPointers[i] = &powerSupplyMonitors[i];
    }

    debug("Initializing sensors");
    sensors[0] = {"Temperature",   FLIGHT_BRAIN_ADDR, 0, 3}; //&(testTempRead)}, //&(Thermocouple::readTemperatureData)},
    sensors[1] = {"All Pressure",  FLIGHT_BRAIN_ADDR, 1, 1};
    sensors[2] = {"Battery Stats", FLIGHT_BRAIN_ADDR, 2, 3};
    sensors[3] = {"Number Packets Sent", FLIGHT_BRAIN_ADDR, 5, 10};
    sensors[4] = {"Expected Static Pressure", FLIGHT_BRAIN_ADDR, 17, 15};

    // debug("Initializing valves");
    // valves = new valveInfo[numValves];
    // valves[0] = {"LOX 2 Way", 20, &(Solenoids::armLOX), &(Solenoids::disarmLOX), &(Solenoids::getAllStates)};
    // valves[1] = {"LOX 5 Way", 21, &(Solenoids::openLOX), &(Solenoids::closeLOX), &(Solenoids::getAllStates)};
    // valves[2] = {"LOX GEMS", 22, &(Solenoids::ventLOXGems), &(Solenoids::closeLOXGems), &(Solenoids::getAllStates)};
    // valves[3] = {"Propane 2 Way", 23, &(Solenoids::armPropane), &(Solenoids::disarmPropane), &(Solenoids::getAllStates)};
    // valves[4] = {"Propane 5 Way", 24, &(Solenoids::openPropane), &(Solenoids::closePropane), &(Solenoids::getAllStates)};
    // valves[5] = {"Propane GEMS", 25, &(Solenoids::ventPropaneGems), &(Solenoids::closePropaneGems), &(Solenoids::getAllStates)};
    // valves[6] = {"High Pressure Solenoid", 26, &(Solenoids::activateHighPressureSolenoid), &(Solenoids::deactivateHighPressureSolenoid), &(Solenoids::getAllStates)};
    // valves[7] = {"High Pressure Solenoid Enable", 31, &(Solenoids::enableHighPressureSolenoid), &(Solenoids::disableHighPressureSolenoid), &(Solenoids::getAllStates)};
    // valves[8] = {"Arm Rocket", 27, &(Solenoids::armAll), &(Solenoids::disarmAll), &(Solenoids::getAllStates)};
    // valves[9] = {"Launch Rocket", 28, &(Solenoids::LAUNCH), &(Solenoids::endBurn), &(Solenoids::getAllStates)};
    // valves[10] = {"Perform Flow", 29, &(Automation::beginBothFlow), &(Automation::endBothFlow), &(Automation::flowConfirmation)};
    // valves[11] = {"Perform LOX Flow", 30, &(Automation::beginLoxFlow), &(Automation::endLoxFlow), &(Automation::flowConfirmation)};
  }
}
