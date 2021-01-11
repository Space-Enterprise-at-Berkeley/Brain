#pragma once
#include "subsystems/Subsystem.h"

#include <Arduino.h>

#include <functional>
#include <string>
#include <vector>

using namespace std;

class Comms : public Subsystem {
  private:
    struct packet {
      int id;
      vector<float> values;
      uint16_t checksum;
    };
    uint16_t checksum(uint8_t *data, int count); // Fletcher16 checksum
    bool validate_packet(string raw_packet);
    packet decode_raw_packet(string raw_packet);
  protected:
    /**
     * Check for available packet.
     * 
     * @return whether a packet is waiting to be received and processed.
     */
    virtual bool packet_available() = 0;
    /**
     * Receives raw packet.
     * 
     * @return raw packet in string form
     */
    virtual string receive_raw_packet() = 0;
    /**
     * Send raw packet.
     * 
     * @param raw_packet The raw packet to be sent
     */
    virtual bool send_raw_packet(string raw_packet) = 0;
  public:
    Comms();
    /**
     * Handle packet event.
     * 
     * This function is special for the Comms class, and sends the specified
     * packet.
     */
    void handle_packet(int id, vector<float> args, EmitterFunc emit, SenderFunc send) override;
    /**
     * Handle registered events.
     * 
     * Should never be called for the comms class.
     */
    void handle_event(string event, vector<float> args, EmitterFunc emit, SenderFunc send) override;
    /**
     * Comms loop function.
     * 
     * Since the Comms class is special, this function gets called every
     * cycle of the event loop. It checks for incoming packets, and if one
     * is received, the send function is called.
     */
    void loop(EmitterFunc emit, SenderFunc send) override;
};
