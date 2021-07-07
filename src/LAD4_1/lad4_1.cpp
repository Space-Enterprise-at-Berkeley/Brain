#include "apogeeDetection.h"
#include <Arduino.h>
#include <batteryMonitor.h>
#include "Barometer.h"
#include "config.h"
#include "IMU.h"
#include "GPS.h"
#include "recovery.h"

#ifdef SERIAL_INPUT_DEBUG
  #define RFSerial Serial
#else
  #define RFSerial Serial6
#endif

#define GPS_Serial Serial8

uint8_t val_index = 0;
char command[75];

// Stores how often we should be requesting data from each sensor.
int sensor_checks[numSensors][2];

sensorInfo *sensor;

long startTime;
String packet;

// Helper functions
void sensorReadFunc(int id);
void recoveryPacket();

IMU _imu;
Barometer bmp;
GPS gps(GPS_Serial);
ApogeeDetection detector;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  RFSerial.begin(57600);

  delay(3000);

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
  }

  debug("Opening File");
  file.open(file_name, O_RDWR | O_CREAT);

  debug("Writing Dummy Data");
  sdBuffer = new Queue();

  std::string start = "beginning writing data";
  if(!write_to_SD(start, file_name)) { // if unable to write to SD, send error packet
    packet = make_packet(101, true);
    RFSerial.println(packet);
  }

  // Initialization of Components
  debug("Initializing Libraries");
  batteryMonitor::init(&Wire, batteryMonitorShuntR, batteryMonitorMaxExpectedCurrent);

  _imu.init(&Wire);
  bmp.init(&Wire);
  gps.init();

  bmp.readAllData(farrbconvert.sensorReadings);
  initAlt = farrbconvert.sensorReadings[0];
  _imu.readAccelerationData(farrbconvert.sensorReadings);
  initAcc_z = farrbconvert.sensorReadings[0];

  detector.init(avgSampleRate, altVar, accVar, initAlt, initAcc_z, mainChuteDeployLoc);
  Recovery::init(drogue_pin, main_chute_pin);
}

void loop() {

  for (int i = 0; i < 100; i++) {
    gps.readChar();
    gps.checkNMEA();
  }

  if (RFSerial.available() > 0) {
    int i = 0;

    while (RFSerial.available()) {
      command[i] = RFSerial.read();
      Serial.print(command[i]);
      i++;
    }
  }

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

  if(sensor->id == 14) { // imu accel
    if (detector.onGround()){
      detector.engineStarted();
    } else if(detector.engineLit() && !detector.onGround()) {
      detector.MeCo();
    } else if (detector.engineOff() && !detector.drogueReleased() && !detector.onGround() && detector.atApogee()) {
      //deploy chute
      Recovery::releaseDrogueChute();
      recoveryPacket();
      delay(300);

      sensors[4].cycle_period = 1;  //GPS Lat Long
      sensors[5].cycle_period = 1;  //GPS Aux
      sensors[1].cycle_period = 10; //IMU Orientation
      sensors[6].cycle_period = 20; //Number Packets

      Recovery::closeDrogueActuator();
    } else if (detector.drogueReleased() && !detector.mainReleased() && !detector.onGround() && detector.atMainChuteDeployLoc()){
      Recovery::releaseMainChute();
      recoveryPacket();
      delay(500);
      Recovery::closeMainActuator();
    }
  }

  delay(10);
}

void sensorReadFunc(int id) {
  switch (id) {
    case 2:
      batteryMonitor::readAllBatteryStats(farrbconvert.sensorReadings);
      break;
    case 5:
      readPacketCounter(farrbconvert.sensorReadings);
      break;
    case 9:
      detector.getFlightState(farrbconvert.sensorReadings);
      break;
    case 10:
      Recovery::getAllStates(farrbconvert.sensorReadings);
      break;
    case 11:
      gps.readPositionData(farrbconvert.sensorReadings);
      break;
    case 12:
      gps.readAuxilliaryData(farrbconvert.sensorReadings);
      break;
    case 13:
      bmp.readAllData(farrbconvert.sensorReadings);
      detector.updateAlt(farrbconvert.sensorReadings[0]);
      break;
    case 14:
      _imu.readAccelerationData(farrbconvert.sensorReadings);
      detector.updateAcc(farrbconvert.sensorReadings[0]);
      break;
    case 15:
      _imu.readQuaternionData(farrbconvert.sensorReadings);
      break;
    default:
      Serial.println("some other sensor");
      break;
  }
}

void recoveryPacket() {
  Recovery::getAllStates(farrbconvert.sensorReadings);
  packet = make_packet(recoverAckId, false);
  Serial.println(packet);
  RFSerial.println(packet);
  write_to_SD(packet.c_str(), file_name);
}
