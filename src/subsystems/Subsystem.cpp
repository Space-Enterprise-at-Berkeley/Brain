#include "subsystems/Subsystem.h"

// if rate is set to 0, the loop fn will be ignored
Subsystem::Subsystem(vector<string> events, vector<int> packets, uint8_t rate)
  : events(events), packets(packets), rate(rate)
{
}

Subsystem::Subsystem() : events(), packets(), rate(0) {}
