/*
   E1_waterflow.cpp - A c++ program that uses I2C to establish communication between
   the sensors and valves inside the rocket & the ground station. Able to send
   data to the ground station via RF. Can receive and process commands sent from
   ground station.
   Created by Vainavi Viswanath, Aug 21, 2020.
*/

#include "config.h"

#include <ducer.h>
#include <batteryMonitor.h>
#include <powerSupplyMonitor.h>

#ifdef SERIAL_INPUT_DEBUG
  #define RFSerial Serial
#else
  #define RFSerial Serial6
#endif

// within loop state variables

uint8_t val_index = 0;
char command[75]; //input command from GS

/*
    Stores how often we should be requesting data from each sensor.
*/
int sensor_checks[numSensors][2];

sensorInfo *sensor;

long startTime;
String packet;

int packet_count = 0;

void sensorReadFunc(int id);

// int autoEventTracker = 0;

// Thermocouple::Cryo _cryoTherms;

void setup() {
  Wire.begin();
  Serial.begin(57600);
  RFSerial.begin(57600);

  delay(3000);

  #ifdef ETH
  debug("Setup Ethernet");
  setupEthernetComms(mac, ip);
  #endif

  debug("Setting up Config");
  config::setup();

  debug("Initializing Sensor Frequencies");

  for (int i = 0; i < numSensors; i++) {
    sensor_checks[i][0] = sensors[i].clock_freq;
    sensor_checks[i][1] = 1;
  }

  debug("Starting SD");

  int res = sd.begin(SdioConfig(FIFO_SDIO));
  if (!res) {
    packet = make_packet(101, true);
    RFSerial.println(packet);
    #ifdef ETH
    sendEthPacket(packet.c_str());
    #endif
  }

  debug("Opening File");
  file.open(file_name, O_RDWR | O_CREAT);

  debug("Writing Dummy Data");
  sdBuffer = new Queue();

  std::string start = "beginning writing data";
  if(!write_to_SD(start, file_name)) { // if unable to write to SD, send error packet
    packet = make_packet(101, true);
    RFSerial.println(packet);
    #ifdef ETH
    sendEthPacket(packet.c_str());
    #endif
  }

  debug("Initializing Libraries");

  Solenoids::init(numSolenoids, solenoidPins, numSolenoidCommands, solenoidCommandIds, solenoidINAAddrs, &Wire1, actuatorMonitorShuntR, powerSupplyMonitorMaxExpectedCurrent, &pressurantSolenoidMonitor, pressurantSolMonShuntR);
  batteryMonitor::init(&Wire, batteryMonitorShuntR, batteryMonitorMaxExpectedCurrent, battMonINAAddr);
  powerSupplyMonitor::init(numPowerSupplyMonitors, powSupMonPointers, powSupMonAddrs, powerSupplyMonitorShuntR, powerSupplyMonitorMaxExpectedCurrent, &Wire);

  Ducers::init(numPressureTransducers, ptAdcIndices, ptAdcChannels, ptTypes, adsPointers);

  Thermocouple::Analog::init(numAnalogTempSens, tempSensAdcIndices, tempSensAdcChannels, adsPointers);

  // _cryoTherms = Thermocouple::Cryo();
  // _cryoTherms.init(numCryoTherms, _cryo_boards, cryoThermAddrs, cryoTypes, &Wire, cryoReadsBackingStore);

  Automation::init();

  commands.updateIds();

  Automation::_eventList.length = 2;




  /* START SET I2C CLOCK FREQ */
  // Wire.setClock(85000);
  #define CLOCK_STRETCH_TIMEOUT 15000
  IMXRT_LPI2C_t *port = &IMXRT_LPI2C1;
  port->MCCR0 = LPI2C_MCCR0_CLKHI(55) | LPI2C_MCCR0_CLKLO(59) |
    LPI2C_MCCR0_DATAVD(25) | LPI2C_MCCR0_SETHOLD(40);
  port->MCFGR1 = LPI2C_MCFGR1_PRESCALE(2);
  port->MCFGR2 = LPI2C_MCFGR2_FILTSDA(5) | LPI2C_MCFGR2_FILTSCL(5) |
    LPI2C_MCFGR2_BUSIDLE(3000); // idle timeout 250 us
  port->MCFGR3 = LPI2C_MCFGR3_PINLOW(CLOCK_STRETCH_TIMEOUT * 12 / 256 + 1);

  port->MCCR1 = port->MCCR0;
	port->MCFGR0 = 0;
	port->MFCR = LPI2C_MFCR_RXWATER(1) | LPI2C_MFCR_TXWATER(1);
	port->MCR = LPI2C_MCR_MEN;
  /* END SET I2C CLOCK FREQ */

  Wire1.setClock(400000);
}

// bool states[8] = {0,0,0,0,0,0,0,0};

void loop() {
  // process command
  #ifdef ETH
  if (Udp.parsePacket()) {
    debug("received udp packet");
    IPAddress remote = Udp.remoteIP();
    for (int i=0; i < 4; i++) {
      Serial.print(remote[i], DEC);
      if (i < 3) {
        Serial.print(".");
      }
    }
    for (uint8_t i = 0; i < numGrounds; i++) {
      if(Udp.remoteIP() == groundIP[i]) {
        debug("received packet came from groundIP");
        receivedCommand = true;
        Udp.read(command, 75);
        debug(String(command));
        break;
      }
    }
  }
  #endif
  if (RFSerial.available() > 0) {
    int i = 0;

    while (RFSerial.available()) {
      command[i] = RFSerial.read();
      Serial.print(command[i]);
      i++;
    }
    receivedCommand = true;
  }

  if(receivedCommand) {
    debug(String(command));
    int8_t id = processCommand(String(command));
    if (id != -1) {
      packet = make_packet(id, false);
      Serial.println(packet);
      Serial.flush();
      #ifndef SERIAL_INPUT_DEBUG
        RFSerial.println(packet);
      #endif
      #ifdef ETH
      sendEthPacket(packet.c_str());
      #endif
      write_to_SD(packet.c_str(), file_name);
    }
    receivedCommand = false;
  }

  if (Automation::inStartup() || Automation::inShutdown()) {

    Serial.println("waiting for: " + String(autoEvents[Automation::_autoEventTracker].duration));

    if (millis() - Automation::_startupTimer > autoEvents[Automation::_autoEventTracker].duration) {
      Serial.println("executing event");
      autoEvents[Automation::_autoEventTracker].action();
      Automation::_startupTimer = millis();

      Automation::_autoEventTracker++;

      Solenoids::getAllStates(farrbconvert.sensorReadings);
      packet = make_packet(20, false);
      Serial.println(packet);
      #ifdef ETH
      sendEthPacket(packet.c_str());
      #endif
    }

    if (Automation::_autoEventTracker == 9) {
      Automation::_startup = false;
    }
    if (Automation::_autoEventTracker == 14) {
      Automation::_shutdown = false;
      Automation::_autoEventTracker = 0;
    }
  }

  // Serial.println("eventlist length: " + String(Automation::_eventList.length));

  // if (Automation::_eventList.length > 0) {
  //   Serial.print(Automation::_eventList.length);
  //   Serial.println(" events remain");
  //   Automation::autoEvent* e = &(Automation::_eventList.events[0]);
  //   Serial.println("duration: " + String(e->duration));
  //   Serial.println("report: " + String(e->report));
  //   if (false && millis() - Automation::_eventList.timer > e->duration) {
  //     Serial.println(" taking action");

  //     e->action();

  //     //Update valve states after each action
  //     Solenoids::getAllStates(farrbconvert.sensorReadings);
  //     packet = make_packet(29, false);
  //     // Serial.println(packet);
  //     // RFSerial.println(packet);
  //     #ifdef ETH
  //     sendEthPacket(packet.c_str());
  //     #endif
  //     write_to_SD(packet.c_str(), file_name);

  //     Automation::flowStatus(farrbconvert.sensorReadings);
  //     packet = make_packet(18, false);
  //     // Serial.println(packet);
  //     // RFSerial.println(packet);
  //     #ifdef ETH
  //     sendEthPacket(packet.c_str());
  //     #endif
  //     write_to_SD(packet.c_str(), file_name);

  //     Automation::removeEvent();
  //     //reset timer
  //     Automation::_eventList.timer = millis();
  //   }
  // }



  /*
     Code for requesting data and relaying back to ground station
  */
  for (int j = 0; j < numSensors; j++) {
    if (sensor_checks[j][0] == sensor_checks[j][1]) {
      sensor_checks[j][1] = 1;
    } else {
      sensor_checks[j][1] += 1;
      continue;
    }

    sensor = &sensors[j];
    sensorReadFunc(sensor->id);
    packet = make_packet(sensor->id, false);
    // Serial.println(packet);
    #ifdef ETH
    sendEthPacket(packet.c_str());
    #endif
    #ifndef SERIAL_INPUT_DEBUG
        // RFSerial.println(packet);
    #endif
    write_to_SD(packet.c_str(), file_name);

    // After getting new pressure data, check injector pressures to detect end of flow:
    if (false && sensor->id==1 && Automation::inFlow()) {

      float loxInjector = farrbconvert.sensorReadings[2];
      float propInjector = farrbconvert.sensorReadings[3];

      Automation::detectPeaks(loxInjector, propInjector);
    }
  }

}


/**
 *
 */
void sensorReadFunc(int id) {
  switch (id) {
    case 0:
      debug("lox tank pt");
      // these hardcode ids are going to royally fuck us soon
      farrbconvert.sensorReadings[0] = Thermocouple::Analog::readSpecificTemperatureData(1);
      farrbconvert.sensorReadings[1] = loxTankPTHeater.controlTemp(farrbconvert.sensorReadings[0]); // heater is not used for waterflows.
      farrbconvert.sensorReadings[2] = loxTankPTHeater.readCurrentDraw();
      farrbconvert.sensorReadings[3] = loxTankPTHeater.readBusVoltage();
      farrbconvert.sensorReadings[4] = -1;
      break;
    case 1:
      debug("Ducers");
      Ducers::readAllPressures(farrbconvert.sensorReadings);
      break;
    case 2:
      debug("Batt");
      batteryMonitor::readAllBatteryStats(farrbconvert.sensorReadings);
      break;
    case 4:
      debug("Cryo all");
      // _cryoTherms.readCryoTemps(farrbconvert.sensorReadings);
      for (int i = 0; i < numCryoTherms; i++) {
        farrbconvert.sensorReadings[i] = _cryo_boards[i].readThermocouple();
      }
      farrbconvert.sensorReadings[numCryoTherms] = -1;
      break;
    case 5:
      readPacketCounter(farrbconvert.sensorReadings);
      break;
    case 6:
      debug("lox gems");
      farrbconvert.sensorReadings[0] = Thermocouple::Analog::readSpecificTemperatureData(0);
      farrbconvert.sensorReadings[1] = loxGemsHeater.controlTemp(farrbconvert.sensorReadings[0]); // heater is not used for waterflows.
      farrbconvert.sensorReadings[2] = loxGemsHeater.readCurrentDraw();
      farrbconvert.sensorReadings[3] = loxGemsHeater.readBusVoltage();
      farrbconvert.sensorReadings[4] = -1;
      break;
    case 8:
      debug("prop gems");
      farrbconvert.sensorReadings[0] = Thermocouple::Analog::readSpecificTemperatureData(5);
      farrbconvert.sensorReadings[1] = propGemsHeater.controlTemp(farrbconvert.sensorReadings[0]); // heater is not used for waterflows.
      farrbconvert.sensorReadings[2] = propGemsHeater.readCurrentDraw();
      farrbconvert.sensorReadings[3] = propGemsHeater.readBusVoltage();
      farrbconvert.sensorReadings[4] = -1;
      break;
    case 16:
      debug("prop tank pt");
      farrbconvert.sensorReadings[0] = Thermocouple::Analog::readSpecificTemperatureData(4);
      farrbconvert.sensorReadings[1] = propTankPTHeater.controlTemp(farrbconvert.sensorReadings[0]); // heater is not used for waterflows.
      farrbconvert.sensorReadings[2] = propTankPTHeater.readCurrentDraw();
      farrbconvert.sensorReadings[3] = propTankPTHeater.readBusVoltage();
      farrbconvert.sensorReadings[4] = -1;
      break;
    case 17:
      debug("static P");
      farrbconvert.sensorReadings[0] = Ducers::loxStaticP(Ducers::_latestReads[loxDomeIdx], Ducers::_latestReads[pressurantIdx]);
      farrbconvert.sensorReadings[1] = Ducers::propStaticP(Ducers::_latestReads[propDomeIdx], Ducers::_latestReads[pressurantIdx]);
      farrbconvert.sensorReadings[2] = -1;
      break;
    case 19:
      debug("lox injector");
      farrbconvert.sensorReadings[0] = Thermocouple::Analog::readSpecificTemperatureData(2);
      farrbconvert.sensorReadings[1] = loxInjectorPTHeater.controlTemp(farrbconvert.sensorReadings[0]); // heater is not used for waterflows.
      farrbconvert.sensorReadings[2] = loxInjectorPTHeater.readCurrentDraw();
      farrbconvert.sensorReadings[3] = loxInjectorPTHeater.readBusVoltage();
      farrbconvert.sensorReadings[4] = -1;
      break;
    case 21:
      debug("solenoid currents");
      Solenoids::getAllCurrents(farrbconvert.sensorReadings);
      break;
    case 22:
      debug("solenoid voltages");
      Solenoids::getAllVoltages(farrbconvert.sensorReadings);
      break;
    case 60:
      debug("Prop Injector");
      farrbconvert.sensorReadings[0] = Thermocouple::Analog::readSpecificTemperatureData(3);
      farrbconvert.sensorReadings[1] = propInjectorPTHeater.controlTemp(farrbconvert.sensorReadings[0]); // heater is not used for waterflows.
      farrbconvert.sensorReadings[2] = propInjectorPTHeater.readCurrentDraw();
      farrbconvert.sensorReadings[3] = propInjectorPTHeater.readBusVoltage();
      farrbconvert.sensorReadings[4] = -1;
      break;
    default:
      Serial.println("some other sensor");
      break;
  }
}
