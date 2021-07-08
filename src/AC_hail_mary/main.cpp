#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

#include <Arduino.h>
#include <Wire.h>

#include <string>

using namespace std;

EthernetUDP Udp;
byte mac[] = {
    0xDE, 0xAD, 0xBE, 0xEF, 0xAC, 0x01};
unsigned int port = 42069; // try to find something that can be the same on gs
const uint8_t numGrounds = 2;
IPAddress groundIP[numGrounds] = {IPAddress(10, 0, 0, 69), IPAddress(10, 0, 0, 70)};

bool setupEthernetComms(byte *mac, IPAddress ip)
{
  Ethernet.begin(mac, ip);

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware)
  {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    Serial.flush();
    exit(1);
  }
  else if (Ethernet.linkStatus() == LinkOFF)
  {
    Serial.println("Ethernet cable is not connected.");
    Serial.flush();
    exit(1);
  }

  Udp.begin(port);
  return true;
}

void sendEthPacket(String packet)
{
  for (uint8_t i = 0; i < numGrounds; i++)
  {
    Udp.beginPacket(groundIP[i], port);
    Udp.write(packet.c_str());
    Udp.endPacket();
  }
}

uint16_t Fletcher16(uint8_t *data, int count)
{

  uint16_t sum1 = 0;
  uint16_t sum2 = 0;

  for (int index = 0; index < count; index++)
  {
    if (data[index] > 0)
    {
      sum1 = (sum1 + data[index]) % 255;
      sum2 = (sum2 + sum1) % 255;
    }
  }
  return (sum2 << 8) | sum1;
}

/*
 * Constructs packet in the following format:
 * {<sensor_ID>,<timestamp>,<data1>,<data2>, ...,<dataN>|checksum}
 */
String make_packet(int id, float *readings)
{
  String packet_content = (String)id;
  packet_content += ",";
  // packet_content += String(millis());
  // packet_content += ",";
  for (int i = 0; i < 4; i++)
  {
    float reading = readings[i];
    packet_content += (String)reading;
    packet_content += ",";
  }

  packet_content.remove(packet_content.length() - 1);
  int count = packet_content.length();
  char const *data = packet_content.c_str();
  uint16_t checksum = Fletcher16((uint8_t *)data, count);
  packet_content += "|";
  String check_ = String(checksum, HEX);
  while (check_.length() < 4)
  {
    check_ = "0" + check_;
  }
  packet_content += check_;
  String packet = "{" + packet_content + "}";
  return packet;
}

int state = 0;
uint32_t trigger = 0;

void setup()
{
  setupEthernetComms(mac, IPAddress(10, 0, 0, 21));
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
}

void loop()
{
  if(millis() > trigger) {
    trigger += 2500;
    if(state == 0) {
      digitalWriteFast(11, HIGH);
      digitalWriteFast(12, LOW);
    } else if(state == 1) {
      digitalWriteFast(11, HIGH);
      digitalWriteFast(12, HIGH);
    } else if(state == 2) {
      digitalWriteFast(11, LOW);
      digitalWriteFast(12, HIGH);
    } else {
      digitalWriteFast(11, HIGH);
      digitalWriteFast(12, HIGH);
      state = -1;
    }
    state++;
  }
  float yeet[7] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
  sendEthPacket(make_packet(3, yeet));
  delay(50);
}
