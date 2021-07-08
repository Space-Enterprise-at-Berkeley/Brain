#include <solenoids.h>
#include <Analog_Thermocouple.h>
#include <Cryo_Thermocouple.h>
#include <common_fw.h>
#include <ADS1219.h>
#include <ADS8167.h>
#include <Automation.h>
#include <INA219.h>
#include <GpioExpander.h>
#include <LTC4151.h>
#include <command.h>
#include <tempController.h>


#define FLIGHT_BRAIN_ADDR 0x00

std::string str_file_name = "E1_coldflow_v2.txt";
const char * file_name = str_file_name.c_str();

#ifdef ETH
IPAddress ip(10, 0, 0, 42); // dependent on local network
#endif

const uint8_t numCryoTherms = 4;
// therm[2] = lox adapter tree pt, therm[3] = lox adapter tree gems
// ADDR = GND, VDD, 10k & 4.3K, 10K & 13K
uint8_t cryoThermAddrs[numCryoTherms] = {0x60, 0x61, 0x62, 0x63};
_themotype cryoTypes[numCryoTherms] = {MCP9600_TYPE_J, MCP9600_TYPE_T, MCP9600_TYPE_T, MCP9600_TYPE_K};
Adafruit_MCP9600 _cryo_boards[numCryoTherms];
float cryoReadsBackingStore[numCryoTherms];

const uint8_t numADCSensors = 2;
uint8_t adcCSPins[numADCSensors] = {37, 36};
uint8_t adcDataReadyPins[numADCSensors] = {26, 27};
uint8_t adcAlertPins[numADCSensors] = {9, 10};
ADS8167 ads[numADCSensors];
ADC * adsPointers[numADCSensors];

const uint8_t numAnalogThermocouples = 1;
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

const uint8_t numSensors = 9;
sensorInfo sensors[numSensors];

const uint8_t numSolenoids = 8;   // l2, l5, lg, p2, p5, pg, h, h enable
uint8_t solenoidPins[numSolenoids] = {5,  3,  1,  4,  2,  0, 6, 39};
const uint8_t numSolenoidCommands = 10;    //       l2, l5, lg, p2, p5, pg,  h, arm, launch , h enable
uint8_t solenoidCommandIds[numSolenoidCommands] = {20, 21, 22, 23, 24, 25, 26,  27, 28     , 31};
uint8_t solenoidINAAddrs[numSolenoids] = {0x40, 0x42, 0x44, 0x41, 0x43, 0x45};

const float batteryMonitorShuntR = 0.002; // ohms
const float batteryMonitorMaxExpectedCurrent = 10; // amps

const float powerSupplyMonitorShuntR = 0.010; // ohms
const float powerSupplyMonitorMaxExpectedCurrent = 5; // amps

const float actuatorMonitorShuntR = 0.033; // ohms

// Heaters
const uint8_t numHeaters = 6;
uint8_t heaterChannels[numHeaters] = {2, 3, 1, 0, 4, 5};
uint8_t heaterCommandIds[numHeaters] = {40, 41, 42, 43, 44, 45};
uint8_t heaterINAAddr[numHeaters] = {0x4B, 0x4C, 0x4A, 0x49, 0x4D, 0x4E};

HeaterCommand loxTankPTHeater("loxTankPTHeater", heaterCommandIds[0], 10, 2, &heaterCtl, heaterChannels[0], &Wire1, heaterINAAddr[0], actuatorMonitorShuntR, powerSupplyMonitorMaxExpectedCurrent);
HeaterCommand loxGemsHeater("loxGemsHeater", heaterCommandIds[1], 10, 2, &heaterCtl, heaterChannels[1], &Wire1, heaterINAAddr[1], actuatorMonitorShuntR, powerSupplyMonitorMaxExpectedCurrent);
HeaterCommand propTankPTHeater("propTankPTHeater", heaterCommandIds[2], 10, 2, &heaterCtl, heaterChannels[2], &Wire1, heaterINAAddr[2], actuatorMonitorShuntR, powerSupplyMonitorMaxExpectedCurrent);
HeaterCommand propGemsHeater("propGemsHeater", heaterCommandIds[3], 10, 2, &heaterCtl, heaterChannels[3], &Wire1, heaterINAAddr[3], actuatorMonitorShuntR, powerSupplyMonitorMaxExpectedCurrent);
HeaterCommand loxInjectorPTHeater("loxInjectorPTHeater", heaterCommandIds[4], 10, 2, &heaterCtl, heaterChannels[4], &Wire1, heaterINAAddr[4], actuatorMonitorShuntR, powerSupplyMonitorMaxExpectedCurrent);
HeaterCommand propInjectorPTHeater("propInjectorPTHeater", heaterCommandIds[5], 10, 2, &heaterCtl, heaterChannels[5], &Wire1, heaterINAAddr[5], actuatorMonitorShuntR, powerSupplyMonitorMaxExpectedCurrent);

AutomationSequenceCommand fullFlow("Perform Flow", 29, &(Automation::beginBothFlow), &(Automation::endBothFlow));
AutomationSequenceCommand loxFlow("Perform LOX Flow", 30, &(Automation::beginLoxFlow), &(Automation::endLoxFlow));
AutomationSequenceCommand hotFire("Perform Hotfire", 32, &(Automation::beginHotfire), &(Automation::endHotfire));

const uint8_t numCommands = 18;
Command *backingStore[numCommands] = {&Solenoids::lox_2,  &Solenoids::lox_5,  &Solenoids::lox_G,
                                        &Solenoids::prop_2, &Solenoids::prop_5, &Solenoids::prop_G,
                                        &Solenoids::high_p, &Solenoids::high_p_enable, &Solenoids::arm_rocket, &Solenoids::launch,
                                        &fullFlow, &loxFlow, &loxTankPTHeater, &loxGemsHeater, &loxInjectorPTHeater, &propTankPTHeater,
                                        &propGemsHeater, &propInjectorPTHeater};
CommandArray commands(numCommands, backingStore);

namespace config {
  void setup() {

    debug("Initializing ADCs");
    for (int i = 0; i < numADCSensors; i++) {
      debug("init ADC" + String(i));
      debug(String(adcCSPins[i]));
      debug(String(adcDataReadyPins[i]));
      debug(String(adcAlertPins[i]));
      ads[i].init(&SPI, adcCSPins[i], adcDataReadyPins[i], adcAlertPins[i]);
      ads[i].setManualMode();
      ads[i].setAllInputsSeparate();
      debug("finish init ADC" + String(i));
      pinMode(adcDataReadyPins[i], INPUT_PULLUP);
      adsPointers[i] = &ads[i];
    }

    debug("Initializing Power Supply monitors");
    for (int i = 0; i < numPowerSupplyMonitors; i++) {
        powerSupplyMonitors[i].begin(&Wire, powSupMonAddrs[i]);
        powerSupplyMonitors[i].configure(INA219_RANGE_16V, INA219_GAIN_40MV, INA219_BUS_RES_12BIT, INA219_SHUNT_RES_12BIT_1S);
        powerSupplyMonitors[i].calibrate(powerSupplyMonitorShuntR, powerSupplyMonitorMaxExpectedCurrent);
        powSupMonPointers[i] = &powerSupplyMonitors[i];
    }

    debug("Initializing sensors");
    // the ordering in this array defines order of operation, not id
    sensors[0] = {"All Pressure",  FLIGHT_BRAIN_ADDR, 1, 1};
    sensors[1] = {"Battery Stats", FLIGHT_BRAIN_ADDR, 2, 3};
    sensors[2] = {"Cryo Temps",      FLIGHT_BRAIN_ADDR, 4, 3};
    sensors[3] = {"Lox PT Temperature",   FLIGHT_BRAIN_ADDR, 0, 10}; //&(testTempRead)}, //&(Thermocouple::readTemperatureData)},
    sensors[4] = {"Number Packets Sent", FLIGHT_BRAIN_ADDR, 5, 10};
    sensors[5] = {"LOX Gems Temp", FLIGHT_BRAIN_ADDR, 6, 10};
    sensors[6] = {"Prop Gems Temp", FLIGHT_BRAIN_ADDR, 8, 10};
    sensors[7] = {"Prop PT Temp", FLIGHT_BRAIN_ADDR, 16, 10};
    sensors[8] = {"Expected Static Pressure", FLIGHT_BRAIN_ADDR, 17, 15};
  }
}
