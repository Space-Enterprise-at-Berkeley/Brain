#include <Analog_Thermocouple.h>
#include <Cryo_Thermocouple.h>
#include <common.h>
#include <ADS8167.h>
#include <Automation.h>
#include <INA219.h>
#include <GpioExpander.h>
#include <LTC4151.h>
#include <loadCell.h>
#include <command.h>

#define FLIGHT_BRAIN_ADDR 0x00

std::string str_file_name = "DAQ.txt";
const char * file_name = str_file_name.c_str();

#ifdef ETH
IPAddress ip(10, 0, 0, 11); // dependent on local network
#endif

const uint8_t numCryoTherms = 4;
// therm[2] = lox adapter tree pt, therm[3] = lox adapter tree gems
// ADDR = GND, VDD, 10k & 4.3K, 10K & 13K
uint8_t cryoThermAddrs[numCryoTherms] = {0x60, 0x61, 0x62, 0x63};
_themotype cryoTypes[numCryoTherms] = {MCP9600_TYPE_J, MCP9600_TYPE_T, MCP9600_TYPE_T, MCP9600_TYPE_K};
Adafruit_MCP9600 _cryo_boards[numCryoTherms];
float cryoReadsBackingStore[numCryoTherms];

const uint8_t numADCSensors = 1;
uint8_t adcCSPins[numADCSensors] = {37};
uint8_t adcDataReadyPins[numADCSensors] = {14};
uint8_t adcAlertPins[numADCSensors] = {15};
ADS8167 ads[numADCSensors];
ADC * adsPointers[numADCSensors];

const uint8_t numAnalogThermocouples = 1;
uint8_t thermAdcIndices[numAnalogThermocouples] = {0};
uint8_t thermAdcChannels[numAnalogThermocouples] = {5};

const uint8_t numPressureTransducers = 1;
uint8_t ptAdcIndices[numPressureTransducers] = {0};
uint8_t ptAdcChannels[numPressureTransducers] = {3};
uint32_t ptTypes[numPressureTransducers] = {30};

const uint8_t numPowerSupplyMonitors = 3;       //5v  , 5V  , 3.3v
uint8_t powSupMonAddrs[numPowerSupplyMonitors] = {0x41, 0x42, 0x43};
INA219 powerSupplyMonitors[numPowerSupplyMonitors];
INA * powSupMonPointers[numPowerSupplyMonitors];

const uint8_t numLoadCells = 2;
byte lcSckPins[numLoadCells] = {38, 10};
byte lcDoutPins[numLoadCells] = {39, 9};
float lcCalVals[numLoadCells] = {4000, -4000};
HX711 loadcells[numLoadCells];
// HX711 loadcell1, loadcell2;

uint8_t battMonINAAddr = 0x40;

const uint8_t numSensors = 5;
sensorInfo sensors[numSensors];

const float batteryMonitorShuntR = 0.002; // ohms
const float batteryMonitorMaxExpectedCurrent = 10; // amps

const float powerSupplyMonitorShuntR = 0.010; // ohms
const float powerSupplyMonitorMaxExpectedCurrent = 5; // amps

namespace config {
  void setup() {
    debug("Initializing ADCs");
    for (int i = 0; i < numADCSensors; i++) {
      debug("init ADC" + String(i));
      debug(String(adcCSPins[i]));
      debug(String(adcDataReadyPins[i]));
      debug(String(adcAlertPins[i]));
      Serial.print("test");
      Serial.flush();
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

    debug("Initializing the Load Cells");
    // for (int i = 0; i < numLoadCells; i++) {
    //  debug("before begin");
    //  loadcells[i].begin(lcDoutPins[i], lcSckPins[i]);
    //  debug("after begin");
    //  loadcells[i].set_scale(lcCalVals[i]);
    //  debug("after set scale");
    //  loadcells[i].tare();
    //  debug("after tare");
    // }

    // debug("before begin");
    // loadcell1.begin(lcDoutPins[0], lcSckPins[0]);
    // debug("after begin");
    // loadcell1.set_scale(lcCalVals[0]);
    // debug("after set scale");
    // loadcell1.tare();
    // debug("after tare");
    // debug("before begin");
    // loadcell2.begin(lcDoutPins[1], lcSckPins[1]);
    // debug("after begin");
    // loadcell2.set_scale(lcCalVals[1]);
    // debug("after set scale");
    // loadcell2.tare();
    // debug("after tare");

    debug("Initializing sensors");
    // the ordering in this array defines order of operation, not id
    sensors[0] = {"Pressures",  FLIGHT_BRAIN_ADDR, 1, 1};
    // sensors[1] = {"Battery Stats", FLIGHT_BRAIN_ADDR, 2, 3};
    sensors[1] = {"Cryo Temps",      FLIGHT_BRAIN_ADDR, 4, 3};
    sensors[2] = {"Load Readings", FLIGHT_BRAIN_ADDR, 3, 2};
    sensors[3] = {"Number Packets Sent", FLIGHT_BRAIN_ADDR, 5, 10};
    sensors[4] = {"Analog Thermocouples", FLIGHT_BRAIN_ADDR, 19, 3};
    // sensors[6] = {"Power Supply Stats", FLIGHT_BRAIN_ADDR, 6, 3};

  }
}
