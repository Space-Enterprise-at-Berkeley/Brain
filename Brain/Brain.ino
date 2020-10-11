/*
 * Brain_I2C.ino - A c++ program that uses I2C to establish communication between 
 * the sensors and valves inside to the rocket with the ground station. Able to send
 * data to the ground station via RF. Can receive and process commands sent from 
 * ground station. 
 * Created by Vainavi Viswanath, Aug 21, 2020.
 */

#include <Wire.h>
#include <SoftwareSerial.h>
#include "brain_utils.h"

SoftwareSerial RFSerial(2,3);

int board_address = 0;
byte sensor_id = 0;
uint8_t index = 0;

/*
 * Array of all sensors we would like to get data from.
 */
sensorInfo all_ids[8] = {
  sensorInfo("Low Pressure",8,0,1,1) //example
};

sensorInfo sensor = sensorInfo("",0,0,0,0);

/* 
 *  Stores how often we should be requesting data from each sensor.
 */
int sensor_checks[sizeof(all_ids)/sizeof(sensorInfo)][2];

valveInfo valve_ids[7] = {
  valveInfo("LOX 2 Way", 20) //example
};

valveInfo valve = valveInfo("",0);

void setup() {
  Wire.begin();       
  Serial.begin(9600);
  RFSerial.begin(57600);
  for (int i=0; i<sizeof(all_ids)/sizeof(sensorInfo); i++) {
    sensor_checks[i][0] = all_ids[i].clock_freq;
    sensor_checks[i][1] = 1;
  }
}

void loop() { 
  String command = "";
  command = RFSerial.readString();
  int action = decode_received_packet(command, &valve);
  take_action(&valve, action);
  
  /*
   * Code for requesting data and relaying back to ground station
   */
  for (int j = 0; j < sizeof(all_ids)/sizeof(sensorInfo); j++) {
    if (sensor_checks[j][0] == sensor_checks[j][1]) {
      sensor_checks[j][1] = 1;
    } else {
      sensor_checks[j][1] += 1;
      continue;
    }
    sensor = all_ids[j];
    board_address = sensor.board_address;
    sensor_id = sensor.sensor_id;

    Wire.beginTransmission(board_address);
    Wire.write(sensor_id);
    Wire.endTransmission();
    Wire.requestFrom(board_address, 24); 
  
    index = 0;
    while (Wire.available()){
      farrbconvert.buffer[index] = Wire.read();
      index++;
    }
    String packet = make_packet(sensor);
    RFSerial.println(packet);
  }
  delay(100);
}

bool write_to_SD(String message){
  // every reading that we get from sensors should be written to sd and saved.
  // TODO: Someone's code here
}