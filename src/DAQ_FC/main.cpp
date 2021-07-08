#include <Adafruit_MCP9600.h>

#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

#include <Arduino.h>
#include <Wire.h>

#include <string>

using namespace std;

EthernetUDP Udp;
byte mac[] = {
    0xDE, 0xAD, 0xBE, 0xEF, 0xDA, 0x02};
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

Adafruit_MCP9600 cryo_amp_boards[4] = {Adafruit_MCP9600(), Adafruit_MCP9600(), Adafruit_MCP9600(), Adafruit_MCP9600()};

void setup()
{
  setupEthernetComms(mac, IPAddress(10, 0, 0, 12));
  Wire.begin();

#define CLOCK_STRETCH_TIMEOUT 15000
  IMXRT_LPI2C_t *port;
  port = &IMXRT_LPI2C1;
  port->MCCR0 = LPI2C_MCCR0_CLKHI(55) | LPI2C_MCCR0_CLKLO(59) |
                LPI2C_MCCR0_DATAVD(25) | LPI2C_MCCR0_SETHOLD(40);
  port->MCFGR1 = LPI2C_MCFGR1_PRESCALE(2);
  port->MCFGR2 = LPI2C_MCFGR2_FILTSDA(5) | LPI2C_MCFGR2_FILTSCL(5) |
                 LPI2C_MCFGR2_BUSIDLE(3000); // idle timeout 250 us
  port->MCFGR3 = LPI2C_MCFGR3_PINLOW(CLOCK_STRETCH_TIMEOUT * 12 / 256 + 1);
  port->MCCR1 = port->MCCR0;
  port->MCFGR0 = 0;
  port->MFCR = LPI2C_MFCR_RXWATER(1) | LPI2C_MFCR_TXWATER(1);
  port->MCR = LPI2C_MCR_MEN;
  /* END SET I2C CLOCK FREQ */

  for (uint8_t i = 0; i < 4; i++)
  {
    cryo_amp_boards[i].begin(0x60 + i);
    cryo_amp_boards[i].setADCresolution(MCP9600_ADCRESOLUTION_16);
    cryo_amp_boards[i].setThermocoupleType(MCP9600_TYPE_K);
    cryo_amp_boards[i].setFilterCoefficient(0);
    cryo_amp_boards[i].enable(true);
  }
}

void loop()
{
  float data[4];
  for(int i = 0; i < 4; i++) {
    data[i] = cryo_amp_boards[i].readThermocouple();
  }
  sendEthPacket(make_packet(4, data));
  delay(50);
}
