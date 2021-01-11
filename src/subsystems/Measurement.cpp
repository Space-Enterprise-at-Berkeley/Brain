#include "subsystems/Measurement.h"

Measurement::Measurement(Sensor *s, uint8_t channel, int packet, uint8_t rate)
  : Subsystem({}, {}, rate), s(s), channel(channel), packet(packet) {}

void Measurement::handle_packet(int id, vector<float> args, EmitterFunc emit, SenderFunc send) {}

void Measurement::handle_event(string event, vector<float> args, EmitterFunc emit, SenderFunc send) {}

void Measurement::loop(EmitterFunc emit, SenderFunc send) {
  send(packet, {s->read(channel)});
}
