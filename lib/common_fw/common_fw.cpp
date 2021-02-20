/*
 * common_fw.cpp- A c++ program that houses the firmware that is reasonably hardware
 * agnostic and can be repeated across different boards and flight hardware.
 * Created by Vainavi Viswanath, Aug 21, 2020.
 */
#include "common_fw.h"

SdFat sd;
File file;
int packetCounter = 0;
struct Queue *sdBuffer;
union floatArrToBytes farrbconvert;

/**
 *
 */
bool write_to_SD(std::string message, const char * file_name) {
    sdBuffer->enqueue(message);
    if(sdBuffer->length >= 40) {
      if(file.open(file_name, O_RDWR | O_APPEND)) {
        int initialLength = sdBuffer->length;
        for(int i = 0; i < initialLength; i++) {
          char *msg = sdBuffer->dequeue();
          file.write(msg, sizeof(msg));
          free(msg);
        }
        file.close();
        return true;
      } else {        //If the file didn't open
          return false;
      }
    }
    return true;
}

/*
 * Constructs packet in the following format:
 * {<sensor_ID>,<data1>,<data2>, ...,<dataN>|checksum}
 */
String make_packet(int id, bool error) {
  String packet_content = (String)id;
  packet_content += ",";
  if (!error) {
    for (int i=0; i<7; i++) {
      float reading = farrbconvert.sensorReadings[i];
      if (reading != -1) {
        packet_content += (String)reading;
        packet_content += ",";
      } else {
        break;
      }
    }
  } else {
    packet_content += "0,";
  }

  packet_content.remove(packet_content.length()-1);
  int count = packet_content.length();
  char const *data = packet_content.c_str();
  uint16_t checksum = Fletcher16((uint8_t *) data, count);
  packet_content += "|";
  String check_ = String(checksum, HEX);
  while(check_.length() < 4) {
    check_ = "0" + check_;
  }
  packet_content += check_;
  String packet = "{" + packet_content + "}";
  incrementPacketCounter();
  return packet;
}

/*
 * Decodes a packet sent from ground station in the following format:
 * {<valve_ID>,<open(1) or close(0)|checksum}
 * Populated the fields of the valve and returns the action to be taken
 * This is a pretty beefy function; can we split this up
 */
int decode_received_packet(String packet, valveInfo *valve, valveInfo valves[], int numValves) {
  Serial.println(packet);
  int data_start_index = packet.indexOf(',');
  if(data_start_index == -1) {
    return -1;
  }
  int valve_id = packet.substring(1,data_start_index).toInt();
  const int data_end_index = packet.indexOf('|');
  if(data_end_index == -1) {
    return -1;
  }
  int action = packet.substring(data_start_index + 1,data_end_index).toInt();

  String checksumstr = packet.substring(data_end_index + 1, packet.length()-1);
  const char *checksum_char = checksumstr.c_str();
  int checksum = strtol(checksum_char, NULL, 16);
  Serial.println(checksum);

  const int count = packet.substring(1, data_end_index).length(); // sanity check; is this right? off by 1 error?
  String str_data= packet.substring(1,data_end_index);
  char const *data = str_data.c_str();
  Serial.println(data);

  int _check = (int)Fletcher16((uint8_t *) data, count);
  Serial.println(_check);
  if (_check == checksum) {
    Serial.println("Checksum correct, taking action");
    chooseValveById(valve_id, valve, valves, numValves);
    return action;
  } else {
    return -1;
  }
}

void readPacketCounter(float *data) {
    data[0] = packetCounter;
    data[1] = -1;
  }

void incrementPacketCounter() {
    packetCounter+=1;
}

/**
 *
 */
void chooseValveById(int id, valveInfo *valve, valveInfo valves[], int numValves) {
  for (int i = 0; i < numValves; i++) {
    if (valves[i].id == id) {
      *valve = valves[i];
      break;
    }
  }
}

/*
 * Calls the corresponding method for this valve with the appropriate
 * action in solenoids.h
 */
void take_action(valveInfo *valve, int action) {
  if (action >= 1) {
    valve->openValve();
  } else if (action == 0) {
    valve->closeValve();
  }
  if(action != -1) {
    valve->ackFunc(farrbconvert.sensorReadings);
  }
}

/*
 * Calculates checksum for key values being sent to ground station:
 * sensor_ID and it's corresponding data points
 */
uint16_t Fletcher16(uint8_t *data, int count) {

  uint16_t sum1 = 0;
  uint16_t sum2 = 0;

  for (int index=0; index<count; index++) {
    if (data[index] > 0) {
      sum1 = (sum1 + data[index]) % 255;
      sum2 = (sum2 + sum1) % 255;
    }
  }
  return (sum2 << 8) | sum1;
}

void debug(String str, int flag){
  if (flag == 0){
    Serial.println(str);
    Serial.flush();
  }
}
