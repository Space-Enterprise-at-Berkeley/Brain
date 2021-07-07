/*
   E1_coldflow.cpp - A c++ program that uses I2C to establish communication between
   the sensors and valves inside the rocket & the ground station. Able to send
   data to the ground station via RF. Can receive and process commands sent from
   ground station.
   Created by Vainavi Viswanath, Aug 21, 2020.
*/

#include "config.h"

#include <ducer.h>
#include <batteryMonitor.h>
#include <powerSupplyMonitor.h>
#include <numeric>

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
//
// TempController loxPTHeater(10, 2, loxAdapterPTHeaterPin); // setPoint = 10 C, alg = PID, heaterPin = 7
// TempController loxGemsHeater(10, 2, loxGemsHeaterPin); // setPoint = 10 C, alg = PID

void sensorReadFunc(int id);

void setup() {
  Wire.begin();
  Serial.begin(57600);

  delay(3000);

  // pinMode(23, OUTPUT);
  // analogWrite(23, 255);

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
    Serial.println(packet);
    #ifdef ETH
    sendEthPacket(packet.c_str());
    #endif
  }

  // debug("Opening File");
  // file.open(file_name, O_RDWR | O_CREAT);
  //
  // debug("Writing Dummy Data");
  // sdBuffer = new Queue();

  std::string start = "beginning writing data";
  // if(!write_to_SD(start, file_name)) { // if unable to write to SD, send error packet
  //   packet = make_packet(101, true);
  //   Serial.println(packet);
  //   #ifdef ETH
  //   sendEthPacket(packet.c_str());
  //   #endif
  // }

  debug("Initializing Libraries");

  debug("Initializing Solenoids");
  // ACSolenoids::init(numSolenoids, solenoidPins, solenoidCommandIds);
  LinearActuators::init(numLinActs, numLinActPairs, in1Pins, in2Pins, linActPairIds, linActCommandIds, &Wire, linActINAAddrs, powerSupplyMonitorShuntR, powerSupplyMonitorMaxExpectedCurrent);
  debug("Initializing battery monitor");
  batteryMonitor::init(&Wire1, batteryMonitorShuntR, batteryMonitorMaxExpectedCurrent, battMonINAAddr);
  debug("Initializing power supply monitors");
  powerSupplyMonitor::init(numPowerSupplyMonitors, powSupMonPointers, powSupMonAddrs, powerSupplyMonitorShuntR, powerSupplyMonitorMaxExpectedCurrent, &Wire);
  debug("passed");
  Automation::init();
  debug("passed2");
  commands.updateIds();
  debug("passed4");
}

void loop() {
  // process command
  debug("top of loop");
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
  if (Serial.available() > 0) {
    int i = 0;

    while (Serial.available()) {
      command[i] = Serial.read();
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
      #ifdef ETH
      sendEthPacket(packet.c_str());
      #endif
      //write_to_SD(packet.c_str(), file_name);
    }
    receivedCommand = false;
  }


  // if (Automation::_eventList->length > 0) {
  //   Serial.print(Automation::_eventList->length);
  //   Serial.println(" events remain");
  //   Automation::autoEvent* e = &(Automation::_eventList->events[0]);
  //   if (millis() - Automation::_eventList->timer > e->duration) {
  //
  //     e->action();
    //
    //   //Update valve states after each action
    //   Solenoids::getAllStates(farrbconvert.sensorReadings);
    //   packet = make_packet(29, false);
    //   Serial.println(packet);
    //   #ifdef ETH
    //   sendEthPacket(packet.c_str());
    //   #endif
    //
    //   Automation::removeEvent();
    //   //reset timer
    //   Automation::_eventList->timer = millis();
    // }
  // }

  LinearActuators::getAllStates(farrbconvert.sensorReadings); // any given index will be a non-zero value if the actuator is not off.
  for (int i = 0; i < numLinActs; i++) {
    if(farrbconvert.sensorReadings[i] > 0) { // if actuator is not off
      // Only autoshutoff if not time-based command
      if(LinearActuators::_linActCommands[i]->endtime == -1 && LinearActuators::_linActCommands[i]->outputMonitor.readShuntCurrent() < 0.1){ //below threshold, turning off
        LinearActuators::_linActCommands[i]->_off();
        LinearActuators::_linActCommands[i]->endtime = -1;
      } else if(LinearActuators::_linActCommands[i]->endtime != -1 && millis() > LinearActuators::_linActCommands[i]->endtime) {
        LinearActuators::_linActCommands[i]->_off();
        LinearActuators::_linActCommands[i]->endtime = -1;
      }
      sensors[3].cycle_period = 20;
    }
  }

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
    debug("finished sensor read");
    // for (int i = 0; i < maxReadings; i++) {
    //   Serial.print(farrbconvert.sensorReadings[i]);
    //   Serial.print(", ");
    // }
    // Serial.println("");
    packet = make_packet(sensor->id, false);

    Serial.println(packet);
    #ifdef ETH
    sendEthPacket(packet.c_str());
    #endif
    //write_to_SD(packet.c_str(), file_name);

      // After getting new pressure data, check injector pressures to detect end of flow:
    // if (sensor->id==1 && Automation::inFlow()){
    //   float loxInjector = farrbconvert.sensorReadings[2];
    //   float propInjector = farrbconvert.sensorReadings[3];
    //
    //   Automation::detectPeaks(loxInjector, propInjector);
    // }
  }
  delay(10);
}


/**
 *
 */
void sensorReadFunc(int id) {
  switch (id) {
    case 1:
      // ACSolenoids::getAllCurrentDraw(farrbconvert.sensorReadings);
      // break;
      // if (std::accumulate(farrbconvert.sensorReadings, farrbconvert.sensorReadings + numSolenoids, 0) > 1) {
      //   sensors[3].cycle_period = 5;
      // } else {
      //   sensors[3].cycle_period = 20;
      // }
      debug("heater current draw");
      #ifdef AC1

      heater1.readCurrentDraw(farrbconvert.sensorReadings, 0);
      heater2.readCurrentDraw(farrbconvert.sensorReadings, 1);
      farrbconvert.sensorReadings[2] = 0;
      farrbconvert.sensorReadings[3] = 0;
      farrbconvert.sensorReadings[4] = -1;

      #elif AC2

      heater1.readCurrentDraw(farrbconvert.sensorReadings, 0);
      heater2.readCurrentDraw(farrbconvert.sensorReadings, 1);
      farrbconvert.sensorReadings[2] = 0;
      farrbconvert.sensorReadings[3] = 0;
      farrbconvert.sensorReadings[4] = -1;

      #elif AC3

      heater1.readCurrentDraw(farrbconvert.sensorReadings, 0);
      heater2.readCurrentDraw(farrbconvert.sensorReadings, 1);
      farrbconvert.sensorReadings[2] = 0;
      farrbconvert.sensorReadings[3] = 0;
      farrbconvert.sensorReadings[4] = -1;

      #endif


    case 2:
      debug("battery stats");
      //batteryMonitor::readAllBatteryStats(farrbconvert.sensorReadings);
      break;
    case 3:
      debug("lin act current draw");
      LinearActuators::getAllCurrentDraw(farrbconvert.sensorReadings);

      if (std::accumulate(farrbconvert.sensorReadings, farrbconvert.sensorReadings + numLinActs, 0) > 1) {
        sensors[3].cycle_period = 1;
      } else {
        sensors[3].cycle_period = 20;
      }
      break;
    case 4:
      debug("lin act all states");
      LinearActuators::getAllStates(farrbconvert.sensorReadings);
      break;
    case 5:
      debug("packet count");
      readPacketCounter(farrbconvert.sensorReadings);
      break;
    default:
      Serial.println("some other sensor");
      break;
  }
}
