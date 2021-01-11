#include "comms/USBSerialComms.h"

USBSerialComms::USBSerialComms(usb_serial_class *port, int baud) :
  Comms(), port(port) { port->begin(baud); }

bool USBSerialComms::packet_available() {
  return port->available() > 0;
}

string USBSerialComms::receive_raw_packet() {
  return string(port->readStringUntil('\n').c_str());
}

// TODO: change this function to return false if fails? Maybe flush serial?
bool USBSerialComms::send_raw_packet(string raw_packet) {
  port->println(raw_packet.c_str());
  return true;
}
