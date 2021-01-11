#pragma once
#include "Subsystem.h"
#include "sensors/Sensor.h"

using namespace std;

class Measurement : public Subsystem {
  private:
    Sensor *s;
    uint8_t channel;
    int packet;
  public:
    /**
     * Construct a measurement object.
     * 
     * @param s The sensor object to read from
     * @param channel The channel to read from the sensor object
     * @param packet The id of the packet on which to send the measurement
     * @param rate The frequency that this measurement should be updated at
     */
    Measurement(Sensor *s, uint8_t channel, int packet, uint8_t rate);
    void handle_packet(int id, vector<float> args, EmitterFunc emit, SenderFunc send);
    void handle_event(string event, vector<float> args, EmitterFunc emit, SenderFunc send);
    void loop(EmitterFunc emit, SenderFunc send);
};
