#include <Arduino.h>

#include <cstring>

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
  unsigned char packet[8];

  packet[0] = id;

  packet[1] = (bits >> 24) & 0xFF;
  packet[2] = (bits >> 16) & 0xFF;
  packet[3] = (bits >> 8) & 0xFF;
  packet[4] = bits & 0xFF;

  uint16_t checksum = Fletcher16(packet, 5);
  packet[5] = (checksum >> 8) & 0xFF;
  packet[6] = checksum & 0xFF;

  packet[7] = 0xFF;

  Serial.write(packet, 8);
  Serial.flush();
}

char* shift_left(char *s, size_t len) {
    std::memmove(s, s + 1, len);
    return s;
}

int main(int argc, char** argv) {
  Serial.begin(57600);
  char *buffer = (char*) malloc(8);
  for(int i = 0;i<8;i++) {
    buffer[i] = 0x00;
  }
  while(1) {
    while(Serial.available() == 1) {
      char holder;
      shift_left(buffer, 8);
      buffer[7] = holder;
      if(holder == (char)0xFF) {
        // verify checksum
        uint16_t real_checksum = Fletcher16(buffer, 5);
        uint16_t sent_checksum = ((uint16_t)buffer[5] << 8) + (uint16_t)buffer[6];
        if(real_checksum == sent_checksum) {
          // extract id and float
          uint8_t id = buffer[0];
          uint32_t float_bits = 0;
          float_bits += (uint32_t)buffer[1] << 24;
          float_bits += (uint32_t)buffer[1] << 16;
          float_bits += (uint32_t)buffer[1] << 8;
          float_bits += (uint32_t)buffer[1];
          float value = *(float*)&float_bits;
          Serial.println(id);
          Serial.println(value);
        }
      }
    }
    // Serial.println(buffer);
    // send_packet(1, 6859476.00);
    // send_packet(2, 6859476.00);
    // send_packet(3, 6859476.00);
    // send_packet(4, 6859476.00);
    // Serial.println("{1,6859476.00,6859476.00,6859476.00,6859476.00|e4f7}");
    delay(100);
  }
  return 0;
}
