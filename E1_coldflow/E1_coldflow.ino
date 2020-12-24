/*
   Brain_I2C.ino - A c++ program that uses I2C to establish communication between
   the sensors and valves inside to the rocket with the ground station. Able to send
   data to the ground station via RF. Can receive and process commands sent from
   ground station.
   Created by Vainavi Viswanath, Aug 21, 2020.
*/

#include <Wire.h>
#include "brain_utils.h"
#include <GPS.h>
#include <Barometer.h>
#include <ducer.h>
#include <SD.h>
#include <SPI.h>
#include <TimeLib.h>

#define RFSerial Serial6
#define GPSSerial Serial8

#define FLIGHT_BRAIN_ADDR 0x00

// within loop state variables
uint8_t board_address = 0;
uint8_t sensor_id = 0;
uint8_t val_index = 0;
char command[50]; //input command from GS

const uint8_t numSensors = 8;

/*
   Array of all sensors we would like to get data from.
*/
sensorInfo sensors[numSensors] = {
  // local sensors
  {"Temperature",                FLIGHT_BRAIN_ADDR, 0, 3}, //&(testTempRead)}, //&(Thermocouple::readTemperatureData)},
  {"All Pressure",               FLIGHT_BRAIN_ADDR, 1, 1},
  {"Battery Stats",              FLIGHT_BRAIN_ADDR, 2, 3},
//  {"Load Cells",                 FLIGHT_BRAIN_ADDR, 3, 5},
  {"Aux temp",                   FLIGHT_BRAIN_ADDR, 4, 1},

//  {"Solenoid Ack",               FLIGHT_BRAIN_ADDR, 4, -1},
//  {"Recovery Ack",               FLIGHT_BRAIN_ADDR, 5, -1},

  //  {"GPS",                        -1, -1, 7, 5, NULL}, //&(GPS::readPositionData)},
  //  {"GPS Aux",                    -1, -1, 8, 8, NULL}, //&(GPS::readAuxilliaryData)},
  //  {"Barometer",                  -1, -1, 8, 6, NULL}, //&(Barometer::readAltitudeData)},
  //  {"Load Cell Engine Left",      -1, -1, 9,  5, NULL},
  //  {"Load Cell Engine Right",     -1, -1, 10, 5, NULL}
};

/*
    Stores how often we should be requesting data from each sensor.
*/
int sensor_checks[numSensors][2];

valveInfo valve;
sensorInfo sensor = {"temp", 23, 23, 23};

std::string str_file_name = "E1_speed_test_results.txt";
const char * file_name = str_file_name.c_str();

long startTime;
String packet;

void setup() {
  Wire.begin();
  Serial.begin(57600);
  RFSerial.begin(57600);
  GPSSerial.begin(4608000);

  delay(1000);

  for (int i = 0; i < numSensors; i++) {
    sensor_checks[i][0] = sensors[i].clock_freq;
    sensor_checks[i][1] = 1;
  }

  int res = sd.begin(SdioConfig(FIFO_SDIO));
  if (!res) {
      // make packet with this status. print, and send over rf
      // do additional error handling; what if no sd?
  }
  file.open(file_name, O_RDWR | O_CREAT);
  file.close();
  sdBuffer = new Queue();

  // initialize all ADCs
  ads = (ADS1219 **) malloc(numADCSensors * sizeof(ADS1219));
  for (int i = 0; i < numADCSensors; i++) {
    ads[i] = new ADS1219(adcDataReadyPins[i], ADSAddrs[i], &Wire);
  }

  Recovery::init();
  Solenoids::init();
  // Ducers::init(&Wire);
  batteryMonitor::init();

  Thermocouple::Analog::init(1, {{1}}, {ads[1]});
  Thermocouple::Cryo::init(numCryoTherms, cryoThermAddrs, cryoTypes);
  tempController::init(10, 2, 7); // setPoint = 10 C, alg = PID, heaterPin = 7
}

void loop() {
  if (RFSerial.available() > 0) {
    int i = 0;
    while (RFSerial.available()) {
      command[i] = RFSerial.read();
      Serial.print(command[i]);
      i++;
    }
    Serial.println();
    Serial.println(String(command));
    int action = decode_received_packet(String(command), &valve);
    if (action != -1) {
      take_action(&valve, action);
      packet = make_packet(valve.id);
      Serial.println(packet);
      RFSerial.println(packet);
      if(!write_to_SD(packet.c_str())){
        // send some error over rf
      }
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
    sensor = sensors[j];
    board_address = sensor.board_address;
    sensor_id = sensor.id;
    sensorReadFunc(sensor.id);
    packet = make_packet(sensor.id);
    Serial.println(packet);
    RFSerial.println(packet);
    if (!write_to_SD(packet.c_str())){
      // send some error packet
    }
  }
}

bool write_to_SD(std::string message) {
  // every reading that we get from sensors should be written to sd and saved.

    sdBuffer->enqueue(message);
    if(sdBuffer->length >= 40) {
      if(file.open(file_name, O_RDWR | O_APPEND)) {
        int initialLength = sdBuffer->length;
        for(int i = 0; i < initialLength; i++) {
          char *msg = sdBuffer->dequeue();
          file.write(msg);
          free(msg);
        }
        file.close();
        return true;
      } else {                                                            //If the file didn't open
        return false;
      }
    }
    return true;
}