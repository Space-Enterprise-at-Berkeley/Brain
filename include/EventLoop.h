#pragma once
#include "comms/Comms.h"
#include "subsystems/Subsystem.h"

#include <Arduino.h>

#include <unordered_map>
#include <string>
#include <vector>
#include <functional>
#include <queue>

using namespace std;

typedef function<void(string,vector<float>)> EmitterFunc;
typedef function<void(int,vector<float>)> SenderFunc;

/**
 * Main event loop class.
 * 
 * This class ties all the subsystems and comms together,
 * combining the concept of a scheduler and an event queue
 * into one.
 */
class EventLoop {
  private:
    struct Packet {
      uint32_t when;
      int id;
      vector<float> args;
      Subsystem *s;

      bool operator >(const Packet& a) const {
        return when > a.when;
      }
      bool operator <(const Packet& a) const {
        return when < a.when;
      }
      bool operator ==(const Packet& a) const {
        return when == a.when;
      }
    };
    struct Event {
      uint32_t when;
      string event;
      vector<float> args;
      Subsystem *s;

      bool operator >(const Event& a) const {
        return when > a.when;
      }
      bool operator <(const Event& a) const {
        return when < a.when;
      }
      bool operator ==(const Event& a) const {
        return when == a.when;
      }
    };
    struct Task {
      uint32_t when;
      uint32_t period; // milliseconds between executions
      Subsystem *s;

      bool operator >(const Task& a) const {
        return when > a.when;
      }
      bool operator <(const Task& a) const {
        return when < a.when;
      }
      bool operator ==(const Task& a) const {
        return when == a.when;
      }
    };
    priority_queue<Packet, vector<Packet>, greater<Packet>> packetq;
    priority_queue<Event, vector<Event>, greater<Event>> eventq;
    priority_queue<Task, vector<Task>, greater<Task>> taskq;
    unordered_map<int,vector<Subsystem*>> packet_handlers;
    unordered_map<string,vector<Subsystem*>> event_handlers;
    vector<Comms*> comms;
    EmitterFunc emit_f;
    SenderFunc send_f;
    SenderFunc emit_packet_f;

    /**
     * Emit an event.
     * 
     * This finds all event handlers that respond to the named
     * event and queues them in the event queue. It is private,
     * because it is passed as a std::function object to Subsystems'
     * loop and handle_* functions.
     * 
     * @param string Name of the emitted event
     * @param args Arguments of the event
     */
    void emit(string event, vector<float> args);
    /**
     * Send a packet.
     * 
     * This queues a packet to send. It doesn't send the packet
     * synchronously, but queues a send task for each of the comms
     * subsystems.
     * 
     * @param id The id of the packet to send
     * @param args The packet contents
     */
    void send(int id, vector<float> args);
    /**
     * Emit a packet event.
     * 
     * This emits a packet event, queuing up all the packet handlers
     * that respond to this packet id. It's private, because it is
     * passed only to registered Comms class instances. This function
     * should only ever be called by the Comms class loop function.
     */
    void emit_packet(int id, vector<float> args);
  public:
    /**
     * Register a subsystem.
     * 
     * Adds the specified subsystem class instance to the task queue
     * with the specified update frequency. It also registers the
     * subsystem with the specified event handlers and packet handlers.
     * 
     * @param s A pointer to the subsystem to register. Should be called
     *          only from the constructor of the subsystem class.
     */
    void adds(Subsystem *s);
    /**
     * Register a comms subsystem.
     * 
     * Registers an instance of the comms class, which is a special type of
     * subsystem that responds to packets waiting to be sent. It's loop
     * method gets called at the end of every EventLoop loop, and checks if
     * a packet is waiting to be received. If a packet is received, it parses
     * the packet and emits a packet event with the corresponding data received
     * in the packet.
     * 
     * @param c A pointer to the comms class instance
     */
    void addc(Comms *c);
    /**
     * Main event loop.
     * 
     * Runs the main event loop. Should be called repeatedly in a loop in
     * the main program thread. NOT THREAD SAFE.
     * 
     * The priority of the queues goes:
     * - packet queue
     * - event queue
     * - task queue
     * For a given queue to start being processed, the queues above it need
     * to be empty first.
     */
    void eloop();
    EventLoop();
};
