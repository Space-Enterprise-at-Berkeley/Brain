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

#define SERIAL_INPUT 0 // 1 is flight config, 0 is for debug

#if SERIAL_INPUT
  #define RFSerial Serial
#else
  #define RFSerial Serial6
#endif

// within loop state variables

uint8_t val_index = 0;
char command[50]; //input command from GS

/*
    Stores how often we should be requesting data from each sensor.
*/
int sensor_checks[numSensors][2];

// valveInfo valve;
sensorInfo *sensor;

long startTime;
String packet;

int packet_count = 0;

void sensorReadFunc(int id);

void setup() {
  Wire.begin();
  Serial.begin(57600);
  RFSerial.begin(57600);

  while(!Serial);
  while(!RFSerial);

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
    packet_count++;
    debug(String(packet_count));
  }

  debug("Opening File");
  file.open(file_name, O_RDWR | O_CREAT);
  file.close();

  debug("Writing Dummy Data");
  // NEED TO DO THIS BEFORE ANY CALLS TO write_to_SD
  sdBuffer = new Queue();

  std::string start = "beginning writing data";
  if(!write_to_SD(start, file_name)) { // if unable to write to SD, send error packet
    packet = make_packet(101, true);
    RFSerial.println(packet);
    packet_count++;
    debug(String(packet_count));
  }

  // config::setup();

  debug("Initializing Libraries");

  Solenoids::init(LOX_2_PIN, LOX_5_PIN, LOX_GEMS_PIN, PROP_2_PIN, PROP_5_PIN, PROP_GEMS_PIN, HIGH_SOL_PIN);
  batteryMonitor::init(&Wire, batteryMonitorShuntR, batteryMonitorMaxExpectedCurrent);

  Ducers::init(numPressureTransducers, ptAdcIndices, ptAdcChannels, ptTypes, ads);

  Thermocouple::Analog::init(numAnalogThermocouples, thermAdcIndices, thermAdcChannels, ads);
}

void loop() {
  // process command
  if (RFSerial.available() > 0) {
    int i = 0;

    while (RFSerial.available()) {
      command[i] = RFSerial.read();
      Serial.print(command[i]);
      i++;
    }

    debug(String(command));
    int8_t id = parseCommand(String(command));
    if (id != -1) {
      //take_action(&valve, action);
      packet = make_packet(id, false);
      Serial.println(packet);
      #if SERIAL_INPUT != 1
        RFSerial.println(packet);
      #endif

      write_to_SD(packet.c_str(), file_name);
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
    packet = make_packet(sensor->id, false);
    Serial.println(packet);

    #if SERIAL_INPUT != 1
        RFSerial.println(packet);
    #endif
    write_to_SD(packet.c_str(), file_name);
  }
  delay(50);
}


/**
 *
 */
void sensorReadFunc(int id) {
  switch (id) {
    case 0:
      debug("Heater");
      Thermocouple::Analog::readTemperatureData(farrbconvert.sensorReadings);
      farrbconvert.sensorReadings[1] = 99; // heater is not used for waterflows.
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
    case 5:
      readPacketCounter(farrbconvert.sensorReadings);
      break;
    default:
      Serial.println("some other sensor");
      break;
  }
}
