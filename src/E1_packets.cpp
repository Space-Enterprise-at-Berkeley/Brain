#include <Arduino.h>

uint16_t Fletcher16(uint8_t *data, int count) {

  uint16_t sum1 = 0;
  uint16_t sum2 = 0;

  for (int index=0; index<count; index++) {
    if (data[index] > 0) {
      sum1 = (sum1 + data[index]) % 255;
      sum2 = (sum2 + sum1) % 255;
    }
  }
  return (sum2 << 8) | sum1;
}

void send_packet(uint8_t id, float value) {
  uint32_t bits = *(uint32_t*)&value;
  unsigned char packet[9];

  packet[0] = id;

  packet[1] = (bits >> 24) & 0xFF;
  packet[2] = (bits >> 16) & 0xFF;
  packet[3] = (bits >> 8) & 0xFF;
  packet[4] = bits & 0xFF;

  uint16_t checksum = Fletcher16(packet, 5);
  packet[6] = (checksum >> 8) & 0xFF;
  packet[7] = checksum & 0xFF;

  packet[8] = 0xFF;

  Serial6.write(packet, 8);
  Serial.flush();
}

int main(int argc, char** argv) {
  Serial6.begin(57600);
  while(1) {
    send_packet(1, 6859476.00);
    send_packet(2, 6859476.00);
    send_packet(3, 6859476.00);
    send_packet(4, 6859476.00);
    // Serial6.println("{1,6859476.00,6859476.00,6859476.00,6859476.00|e4f7}");
    delay(10);
  }
  return 0;
}
