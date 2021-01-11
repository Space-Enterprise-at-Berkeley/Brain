#pragma once

#include <functional>
#include <string>
#include <vector>

using namespace std;

typedef function<void(string,vector<float>)> EmitterFunc;
typedef function<void(int,vector<float>)> SenderFunc;

class Subsystem {
  public:
    vector<string> events;
    vector<int> packets;
    uint8_t rate;

    Subsystem();
    Subsystem(vector<string> events, vector<int> packets, uint8_t rate);
    virtual void handle_packet(int id, vector<float> args, EmitterFunc emit, SenderFunc send) = 0;
    virtual void handle_event(string event, vector<float> args, EmitterFunc emit, SenderFunc send) = 0;
    virtual void loop(EmitterFunc emit, SenderFunc send) = 0;
};
