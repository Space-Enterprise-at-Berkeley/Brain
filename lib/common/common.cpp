/*
 * common_fw.cpp- A c++ program that houses the firmware that is reasonably hardware
 * agnostic and can be repeated across different boards and flight hardware.
 * Created by Vainavi Viswanath, Aug 21, 2020.
 */
#include "common.h"

SdFat sd;
File file;

bool receivedCommand = false;

int packetCounter = 0;

char buffer[75];
struct Queue *sdBuffer;
union floatArrToBytes farrbconvert;

#ifdef ETH
EthernetUDP Udp;
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
unsigned int port = 42069; // try to find something that can be the same on gs
const uint8_t numGrounds = 2;
IPAddress groundIP[numGrounds] = {IPAddress(10, 0, 0, 69), IPAddress(10, 0, 0, 70)};
#endif

Command *tmpCommand;

/*
 * Constructs packet in the following format:
 * {<sensor_ID>,<timestamp>,<data1>,<data2>, ...,<dataN>|checksum}
 */
String make_packet(int id, bool error) {
  String packet_content = (String)id;
  packet_content += ",";
  // packet_content += String(millis());
  // packet_content += ",";
  if (!error) {
    for (int i=0; i<8; i++) {
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
int8_t processCommand(String packet) {
  debug("Parse Command");
  debug(packet);
  int data_start_index = packet.indexOf(',');
  if(data_start_index == -1) {
    return -1;
  }
  debug(String(data_start_index));
  int command_id = packet.substring(1,data_start_index).toInt();
  const int data_end_index = packet.indexOf('|');
  if(data_end_index == -1) {
    return -1;
  }
  debug(String(data_end_index));
  debug(packet.substring(data_start_index + 1,data_end_index).c_str());
  char data_string[data_end_index - data_start_index + 1];
  packet.substring(data_start_index + 1,data_end_index).toCharArray(data_string, data_end_index - data_start_index + 1);
  char *tmp;
  uint8_t i = 0;
  float command_data[maxReadings];
  debug(data_string);
  tmp = std::strtok(data_string, ",");
  while(tmp != NULL){
    command_data[i++] = atof(tmp);
    debug(String(command_data[i-1]));
    tmp = std::strtok(NULL, ",");
  }

  String checksumstr = packet.substring(data_end_index + 1, packet.length()-1);
  const char *checksum_char = checksumstr.c_str();
  int checksum = strtol(checksum_char, NULL, 16);
  debug("checksum: ");
  debug(checksum);

  const int count = packet.substring(1, data_end_index).length();
  String str_data= packet.substring(1,data_end_index);
  char const *data = str_data.c_str();
  debug(String(data));

  int _check = (int)Fletcher16((uint8_t *) data, count);
  if (_check == checksum) {
    debug("Checksum correct, taking action");
    tmpCommand = commands.get(command_id); //chooseValveById(valve_id, valve, valves, numValves);
    if (tmpCommand != nullptr) {
      debug("valid command");
      tmpCommand->parseCommand(command_data);
      tmpCommand->confirmation(farrbconvert.sensorReadings);
      Serial.println("got valid conf");
      if (tmpCommand->ID() == 20 || tmpCommand->ID() == 21 || tmpCommand->ID() == 22 || tmpCommand->ID() == 23 || tmpCommand->ID() == 24 || tmpCommand->ID() == 25 || tmpCommand->ID() == 26 || tmpCommand->ID() == 27 || tmpCommand->ID() == 28 || tmpCommand->ID() == 31) {
        return 20;
      }
      Serial.println("returning: " + String(tmpCommand->ID()));
      Serial.flush();
      return tmpCommand->ID();
    } else {
      debug("couldn't find valid command.");
      return -1;
    }
  } else {
    return -1;
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

/**
 * Add messages to a queue, and every n messages,
 * dequeue everything and dump it onto the sd.
 */
bool write_to_SD(std::string message, const char * file_name) {
    std::string newMessage = std::string(itoa(millis(), buffer, 10)) + ", " + message;
    sdBuffer->enqueue(newMessage);
    if(sdBuffer->length >= qMaxSize) {
        int initialLength = sdBuffer->length;
        for(int i = 0; i < initialLength; i++) {
          uint8_t strLength = sdBuffer->dequeue(buffer);
          // Serial.println("dequeue: ");
          // Serial.print(String(buffer));
          file.write(buffer, strLength);
        }
        debug("flushing file");
        file.flush();
        return true;
    }
    return true;
}

void readPacketCounter(float *data) {
    data[0] = packetCounter;
    data[1] = -1;
  }

void incrementPacketCounter() {
    packetCounter+=1;
}

#ifdef ETH
bool setupEthernetComms(byte * mac, IPAddress ip){
  Ethernet.begin(mac, ip);

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    Serial.flush();
    exit(1);
  } else if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
    Serial.flush();
    exit(1);
  }

  Udp.begin(port);
  return true;
}

void sendEthPacket(std::string packet) {
  for (uint8_t i = 0; i < numGrounds; i++){
    Udp.beginPacket(groundIP[i], port);
    Udp.write(packet.c_str());
    Udp.endPacket();
  }
}
#endif

void debug(String str) {
  #ifdef DEBUG
    Serial.println(str);
    Serial.flush();
    // #ifdef ETH
    //   sendEthPacket(std::string(str.c_str()));
    // #endif
  #endif
}
