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
    sensor_checks[i][0] = sensors[i].cycle_period;
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

  Solenoids::init(numSolenoids, solenoidPins, numSolenoidCommands, solenoidCommandIds);
  batteryMonitor::init(&Wire, batteryMonitorShuntR, batteryMonitorMaxExpectedCurrent, battMonINAAddr);
  powerSupplyMonitor::init(numPowerSupplyMonitors, powSupMonPointers, powSupMonAddrs, powerSupplyMonitorShuntR, powerSupplyMonitorMaxExpectedCurrent, &Wire);

  Ducers::init(numPressureTransducers, ptAdcIndices, ptAdcChannels, ptTypes, adsPointers);

  Thermocouple::Analog::init(numAnalogTempSens, tempSensAdcIndices, tempSensAdcChannels, adsPointers);

  // _cryoTherms = Thermocouple::Cryo();
  // _cryoTherms.init(numCryoTherms, _cryo_boards, cryoThermAddrs, cryoTypes, &Wire, cryoReadsBackingStore);

  Automation::init();

  commands.updateIds();

  Automation::_eventList.length = 2;
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

    if (Automation::_autoEventTracker == 8) {
      Automation::_startup = false;
    }
    if (Automation::_autoEventTracker == 13) {
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
      Thermocouple::Analog::readSpecificTemperatureData(1, farrbconvert.sensorReadings);
      farrbconvert.sensorReadings[1] = loxTankPTHeater.controlTemp(farrbconvert.sensorReadings[0]); // heater is not used for waterflows.
      farrbconvert.sensorReadings[2] = -1;
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
      Thermocouple::Analog::readSpecificTemperatureData(0, farrbconvert.sensorReadings);
      farrbconvert.sensorReadings[1] = loxGemsHeater.controlTemp(farrbconvert.sensorReadings[0]); // heater is not used for waterflows.
      farrbconvert.sensorReadings[2] = -1;
      break;
    case 8:
      debug("prop gems");
      Thermocouple::Analog::readSpecificTemperatureData(5, farrbconvert.sensorReadings);
      farrbconvert.sensorReadings[1] = propGemsHeater.controlTemp(farrbconvert.sensorReadings[0]); // heater is not used for waterflows.
      farrbconvert.sensorReadings[2] = -1;
      break;
    case 16:
      debug("prop tank pt");
      Thermocouple::Analog::readSpecificTemperatureData(4, farrbconvert.sensorReadings);
      farrbconvert.sensorReadings[1] = propTankPTHeater.controlTemp(farrbconvert.sensorReadings[0]); // heater is not used for waterflows.
      farrbconvert.sensorReadings[2] = -1;
      break;
    case 17:
      debug("static P");
      farrbconvert.sensorReadings[0] = Ducers::loxStaticP(Ducers::_latestReads[loxDomeIdx], Ducers::_latestReads[pressurantIdx]);
      farrbconvert.sensorReadings[1] = Ducers::propStaticP(Ducers::_latestReads[propDomeIdx], Ducers::_latestReads[pressurantIdx]);
      farrbconvert.sensorReadings[2] = -1;
      break;
    case 19:
      debug("lox injector");
      Thermocouple::Analog::readSpecificTemperatureData(2, farrbconvert.sensorReadings);
      farrbconvert.sensorReadings[1] = loxInjectorPTHeater.controlTemp(farrbconvert.sensorReadings[0]); // heater is not used for waterflows.
      farrbconvert.sensorReadings[2] = -1;
      break;
    case 60:
      debug("Prop Injector");
      Thermocouple::Analog::readSpecificTemperatureData(3, farrbconvert.sensorReadings);
      farrbconvert.sensorReadings[1] = propInjectorPTHeater.controlTemp(farrbconvert.sensorReadings[0]); // heater is not used for waterflows.
      farrbconvert.sensorReadings[2] = -1;
      break;
    default:
      Serial.println("some other sensor");
      break;
  }
}
