#include "subsystems/Valve.h"

Valve::Valve(DigitalInterface *di, uint8_t channel, int id)
  : Subsystem({}, {id}, 0), di(di), channel(channel), id(id)
{}

void Valve::handle_packet(int id, vector<float> args, EmitterFunc emit, SenderFunc send) {
  bool success = (args[0]==1.0?di->set_output(channel, 1):di->set_output(channel, 0));
  if(success) {
    send(id, args);
  }
}

void Valve::handle_event(string event, vector<float> args, EmitterFunc emit, SenderFunc send) {}

void Valve::loop(EmitterFunc emit, SenderFunc send) {}
