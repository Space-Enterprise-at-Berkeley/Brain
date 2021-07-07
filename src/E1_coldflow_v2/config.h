#include <solenoids.h>
#include <Analog_Thermocouple.h>
// #include <Cryo_Thermocouple.h>
#include <Adafruit_MCP9600.h>
#include <common.h>
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

const uint8_t numAnalogThermocouples = 4;
uint8_t thermAdcIndices[numAnalogThermocouples] = {0};
uint8_t thermAdcChannels[numAnalogThermocouples] = {4};

const uint8_t numPressureTransducers = 4;
uint8_t ptAdcIndices[numPressureTransducers] = {0, 0, 0, 0}; //, 1, 1, 1, 1};
uint8_t ptAdcChannels[numPressureTransducers] = {0, 1, 2, 3}; //, 4, 5, 6, 7};
uint32_t ptTypes[numPressureTransducers] = {1000, 100, 300, 5000}; //, 5000, 1000, 1000, 1000};
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

const uint8_t loxAdapterPTHeaterPin = 7;
const uint8_t loxGemsHeaterPin = 7;
const uint8_t propAdapterPTHeaterPin = 7;
const uint8_t propGemsHeaterPin = 7;

const float batteryMonitorShuntR = 0.002; // ohms
const float batteryMonitorMaxExpectedCurrent = 10; // amps

const float powerSupplyMonitorShuntR = 0.010; // ohms
const float powerSupplyMonitorMaxExpectedCurrent = 5; // amps

const float actuatorMonitorShuntR = 0.033;
const float actuatorMonitorMaxExpectedCurrent = 5;

HeaterCommand loxPTHeater("LOX PT Heater", 40, 10, 2, loxAdapterPTHeaterPin); // setPoint = 10 C, alg = PID
HeaterCommand loxGemsHeater("LOX Gems Heater", 41, 10, 2, loxGemsHeaterPin); // setPoint = 10C, alg = PID
HeaterCommand propPTHeater("Prop PT Heater", 42, 10, 2, propAdapterPTHeaterPin); // setPoint = 10 C, alg = PID
HeaterCommand propGemsHeater("Prop Gems Heater", 43, 10, 2, propGemsHeaterPin); // setPoint = 10C, alg = PID

AutomationSequenceCommand fullFlow("Perform Flow", 29, &(Automation::beginBothFlow), &(Automation::endBothFlow));
AutomationSequenceCommand loxFlow("Perform LOX Flow", 30, &(Automation::beginLoxFlow), &(Automation::endLoxFlow));

const uint8_t numCommands = 14;
Command *backingStore[numCommands] = {&Solenoids::lox_2,  &Solenoids::lox_5,  &Solenoids::lox_G,
                                        &Solenoids::prop_2, &Solenoids::prop_5, &Solenoids::prop_G,
                                        &Solenoids::high_p, &Solenoids::high_p_enable, &Solenoids::arm_rocket, &Solenoids::launch,
                                        &fullFlow, &loxFlow, &loxPTHeater, &loxGemsHeater};
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

    debug("initializing cryo therms");
    for (int i = 0; i < numCryoTherms; i++) {

      if (!_cryo_boards[i].begin(cryoThermAddrs[i], &Wire)) {
        Serial.println("Error initializing cryo board at Addr 0x" + String(cryoThermAddrs[i], HEX));
        return -1;
      }

      _cryo_boards[i].setADCresolution(MCP9600_ADCRESOLUTION_12);
      _cryo_boards[i].setThermocoupleType(cryoTypes[i]);
      _cryo_boards[i].setFilterCoefficient(3);
      _cryo_boards[i].enable(true);
    }

    debug("Initializing sensors");
    // the ordering in this array defines order of operation, not id
    sensors[0] = {"All Pressure",  FLIGHT_BRAIN_ADDR, 1, 1};
    sensors[1] = {"Battery Stats", FLIGHT_BRAIN_ADDR, 2, 3};
    sensors[2] = {"Cryo Temps",      FLIGHT_BRAIN_ADDR, 4, 3};
    sensors[3] = {"Lox PT Temperature",   FLIGHT_BRAIN_ADDR, 0, 4}; //&(testTempRead)}, //&(Thermocouple::readTemperatureData)},
    sensors[4] = {"Number Packets Sent", FLIGHT_BRAIN_ADDR, 5, 10};
    sensors[5] = {"LOX Gems Temp", FLIGHT_BRAIN_ADDR, 6, 4};
    sensors[6] = {"Prop Gems Temp", FLIGHT_BRAIN_ADDR, 8, 4};
    sensors[7] = {"Prop PT Temp", FLIGHT_BRAIN_ADDR, 16, 4};
    sensors[8] = {"Expected Static Pressure", FLIGHT_BRAIN_ADDR, 17, 15};
  }
}
