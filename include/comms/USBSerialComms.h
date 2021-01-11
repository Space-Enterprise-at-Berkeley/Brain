#pragma once
#include "Comms.h"

#include <Arduino.h>

#include <string>

using namespace std;

class USBSerialComms : public Comms {
  private:
    usb_serial_class *port;
    string receive_raw_packet() override;
    bool send_raw_packet(string raw_packet) override;
    bool packet_available() override;
  public:
    /**
     * Construct a USB Serial comms subsystem.
     * 
     * @param port The usb serial port to connect to
     * @param baud the baud rate
     */
    USBSerialComms(usb_serial_class *port, int baud);
};
