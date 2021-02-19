/*
 * common_fw.h- A c++ program that houses the firmware that is reasonably hardware
 * agnostic and can be repeated across different boards and flight hardware.
 * Created by Vainavi Viswanath, Aug 21, 2020.
 */

#ifndef COMMON_H_
#define COMMON_H_

#include <SPI.h>
#include <string>
#include <SdFat.h>
#include <TimeLib.h>
#include <Arduino.h>
#include <Wire.h>


struct Queue {

  struct Node {
    struct Node *next;
    char *message;
    int length; //length doesn't include the null terminator
  };

  uint16_t length = 0;

  struct Node *end = 0;
  struct Node *front = 0;

  Queue() {
    length = 0;
    end = nullptr;
    front = nullptr;
  }

  void enqueue(std::string message) {
    struct Node *temp;
    length++;
    temp = (struct Node *)malloc(sizeof(struct Node));

    temp->length = message.length();
    temp->message = (char *)malloc(temp->length + 2);

    strncpy(temp->message, message.c_str(), temp->length + 1);
    temp->message[temp->length] = '\n'; // add \n to string when enqueue
    temp->message[temp->length + 1] = '\0';

    temp->next = nullptr;
    if (!front) {
      front = temp;
    } else {
      end->next = temp;
    }
    end = temp;
  }

  char * dequeue() { // string still needs to be cleared after dequeue; be very careful about this; wherever this is called.
    if(length > 0) {
      length--;
      struct Node *tmp = front;
      char *msg = tmp->message;

      if(length != 0) {
        front = tmp->next;
      } else {
        front = nullptr;
        end = nullptr;
      }

      tmp->message = nullptr;
      tmp->next = nullptr;
      free(tmp);

      return msg;
    }
    return nullptr;
  }
};

/*
 * Data structure to allow the conversion of bytes to floats and vice versa.
 */
union floatArrToBytes {
  char buffer[36];
  float sensorReadings[9];
};

/*
 * Data structure to store all information relevant to a specific sensor type.
 */
struct sensorInfo {
  String name;
  int board_address;
  int id;
  int clock_freq;
};

/*
 * Data structure to store all information relevant to a specific valve.
 */
struct valveInfo {
  String name;
  int id;
  int (*openValve)();
  int (*closeValve)();
  void (*ackFunc)(float *data);
};



String make_packet (int id, bool error);
uint16_t Fletcher16 (uint8_t *data, int count);
void chooseValveById (int id, struct valveInfo *valve, valveInfo valves[], int numValves);
bool write_to_SD(std::string message, const char * file_name);
int decode_received_packet(String packet, valveInfo *valve, valveInfo valves[], int numValves);
void readPacketCounter(float *data);
void incrementPacketCounter();
void take_action(valveInfo *valve, int action);
uint16_t Fletcher16(uint8_t *data, int count);
void debug(String str, int debug);

extern SdFat sd;
extern File file;

extern struct Queue *sdBuffer;

extern union floatArrToBytes farrbconvert;
extern struct sensorInfo *sensors;
extern struct valveInfo *valves;
extern int packetCounter;
#endif // _COMMON_H_



