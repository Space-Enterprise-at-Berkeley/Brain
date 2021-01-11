#include "EventLoop.h"

EventLoop::EventLoop() {
  emit_f = bind(&EventLoop::emit, this, placeholders::_1, placeholders::_2);
  send_f = bind(&EventLoop::send, this, placeholders::_1, placeholders::_2);
  emit_packet_f = bind(&EventLoop::emit_packet, this, placeholders::_1, placeholders::_2);
}

void EventLoop::adds(Subsystem *s) {
  vector<int> packets = s->packets;
  vector<string> events = s->events;
  uint8_t rate = s->rate;
  // add subsystem to appropriate packet handlers
  for(int p : packets) {
    if(packet_handlers.count(p) == 0) {
      packet_handlers.insert({p, {s}});
    } else {
      packet_handlers.at(p).push_back(s);
    }
  }
  // add subsystem to appropriate event handlers
  for(string e : events) {
    if(event_handlers.count(e) == 0) {
      event_handlers.insert({e, {s}});
    } else {
      event_handlers.at(e).push_back(s);
    }
  }
  // add subsystem task
  if(rate > 0) {
    Task t;
    t.when = millis();
    t.period = 1000 / rate;
    t.s = s;
    taskq.push(t);
  }
}

void EventLoop::addc(Comms *c) {
  comms.push_back(c);
}

void EventLoop::eloop() {
  if(!packetq.empty()) {
    if(packetq.top().when <= millis()) {
      Packet next = packetq.top();
      packetq.pop();
      next.s->handle_packet(next.id, next.args, emit_f, send_f);
    }
  } else if(!eventq.empty()) {
    if(eventq.top().when <= millis()) {
      Event next = eventq.top();
      eventq.pop();
      next.s->handle_event(next.event, next.args, emit_f, send_f);
    }
  } else if(!taskq.empty()) {
    if(taskq.top().when <= millis()) {
      Task next = taskq.top();
      taskq.pop();
      taskq.push({.when = millis() + next.period, .period = next.period, .s = next.s});
      next.s->loop(emit_f, send_f);
    }
  }
  for(Comms *c : comms) {
    c->loop(emit_f, emit_packet_f);
  }
}

void EventLoop::emit(string event, vector<float> args) {
  if(event_handlers.count(event) > 0) {
    for(Subsystem *s : event_handlers.at(event)) {
      eventq.push({.when = millis(), .event = event, .args = args, .s = s});
    }
  }
}

void EventLoop::send(int id, vector<float> args) {
  for(Comms *c : comms) {
    packetq.push({.when = millis(), .id = id, .args = args, .s = c});
  }
}

void EventLoop::emit_packet(int id, vector<float> args) {
  if(packet_handlers.count(id) > 0) {
    for(Subsystem *s : packet_handlers.at(id)) {
      packetq.push({.when = millis(), .id = id, .args = args, .s = s});
    }
  }
}
