#include "comms/Comms.h"

uint16_t Comms::checksum(uint8_t *data, int count) {
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

// TODO: make this function part of decode_packet so that this doesnt get computed twice
bool Comms::validate_packet(string raw_packet) {
  size_t pipe_position = raw_packet.find("|");
  size_t len = raw_packet.length();
  if(pipe_position == std::string::npos) return false;
  std::string data = raw_packet.substr(1, pipe_position-1);
  uint16_t recv_checksum = strtol(raw_packet.substr(pipe_position+1, len - pipe_position -2).c_str(), NULL, 16);
  return checksum((uint8_t *)data.c_str(), data.length()) == recv_checksum;
}

Comms::packet Comms::decode_raw_packet(string raw_packet) {
  size_t pipe_position = raw_packet.find("|");
  size_t data_start = raw_packet.find(",");
  size_t len = raw_packet.length();
  packet p;
  p.id = strtol(raw_packet.substr(1,data_start-1).c_str(), NULL, 10);
  p.checksum = strtol(raw_packet.substr(pipe_position+1, len - pipe_position -2).c_str(), NULL, 16);
  string data = raw_packet.substr(data_start+1,pipe_position - data_start - 1);
  size_t next_comma = data.find(",");
  while(next_comma != string::npos) {
    p.values.push_back(strtof(data.substr(0, next_comma).c_str(), NULL));
    data = data.substr(next_comma+1);
    next_comma = data.find(",");
  }
  p.values.push_back(strtof(data.c_str(), NULL)); // last item list list
  return p;
}

void Comms::loop(EmitterFunc emit, SenderFunc send) {
  if(!packet_available()) return;
  string raw_packet = receive_raw_packet();
  if(!validate_packet(raw_packet)) return; // corrupt packet check
  packet decoded = decode_raw_packet(raw_packet);
  send(decoded.id, decoded.values);
}

void Comms::handle_packet(int id, vector<float> args, EmitterFunc emit, SenderFunc send) {
  // send packet
  string raw_packet = "" + string(((String)id).c_str());
  for(float e : args) {
    raw_packet += "," + string(((String)e).c_str());
  }
  uint16_t raw_checksum = checksum((uint8_t *)raw_packet.c_str(), raw_packet.length());
  char c_checksum[5];
  sprintf(c_checksum, "%x", raw_checksum);
  raw_packet += "|" + string(c_checksum);
  raw_packet = "{" + raw_packet + "}";
  send_raw_packet(raw_packet);
}

void Comms::handle_event(string event, vector<float> args, EmitterFunc emit, SenderFunc send) {}

Comms::Comms() : Subsystem() {}
