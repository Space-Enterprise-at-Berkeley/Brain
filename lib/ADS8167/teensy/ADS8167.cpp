#include "ADS8167.h"
#include "ADS8167_reg.h"

#include <string.h>

#include <Arduino.h>
#include <SPI.h>

#include <math.h>
#include "HardwareSerial.h"
// #define LOG_LOCAL_LEVEL     ESP_LOG_VERBOSE
// #define DBG(msg, ...)      Serial.printf("%s: " msg "\n", __FUNCTION__, ##__VA_ARGS__)

// Helpers
#define CMD_ADDRESS_MASK    ((uint16_t)(1 << 11) - 1)

#define N_CHANNELS          8

#define _SPI_SPEED          14000000

ADS8167::ADS8167(SPIClass *theSPI, uint8_t cs, uint8_t rdy, uint8_t alrt):
  ADC(rdy)
 {
    _cs_pin = cs;
    _rdy_pin = rdy;
    _alrt_pin = alrt;
    _theSPI = theSPI;
}

ADS8167::ADS8167():
  ADC()
 {
   #ifdef DEBUG
     Serial.println("empty constructor ADS8167");
     Serial.flush();
   #endif

 }

bool ADS8167::init(SPIClass *theSPI, uint8_t cs, uint8_t rdy, uint8_t alrt) {
    #ifdef DEBUG
      Serial.print("Starting Init ADS8167 w/ cs pin: ");
      Serial.println(cs);
      Serial.flush();
    #endif
    _cs_pin = cs;
    _rdy_pin = rdy;
    _alrt_pin = alrt;
    _theSPI = theSPI;
    init();
}

bool ADS8167::init() {
    #ifdef DEBUG
      Serial.println("top of init ADC");
      Serial.flush();
    #endif
    _theSPI->begin();

    // not sure if any of these need to be pullup, or that's handled in hardware
    pinMode(_cs_pin, OUTPUT);
    pinMode(_rdy_pin, INPUT);
    pinMode(_alrt_pin, INPUT);

    digitalWrite(_cs_pin, HIGH);

    // enable writing
    write_cmd(ADCCMD_WR_REG, REG_ACCESS, REG_ACCESS_BITS);

    // Powerup all except the internal ref buffer
    write_cmd(ADCCMD_WR_REG, REG_PD_CNTL, PD_CNTL_PD_REF);

    // Data type: ADC value + 4-bit channel id
    // write_cmd(ADCCMD_WR_REG, REG_DATA_CNTL, DATA_CNTL_FORMAT_SAMPLE);

    // Vref = 5V0
    // write_cmd(ADCCMD_WR_REG, REG_OFST_CAL, OFST_CAL_5V0);

    // setSDOMode();

    // Custom channel seq mode
    // write_cmd(ADCCMD_WR_REG, REG_DEVICE_CFG, DEVICE_CFG_SEQMODE_CUSTOM);
    // Manual channel seq moe
    // write_cmd(ADCCMD_WR_REG, REG_DEVICE_CFG, DEVICE_CFG_SEQMODE_MANUAL);
    #ifdef DEBUG
    Serial.println("bottom of init ADC");
      Serial.println("Initialized ADS8167 w/ cs pin: " + String(_cs_pin));
      Serial.flush();
    #endif
    return true;
}

/*
 *
 */
void ADS8167::setSDOMode() {
  write_cmd(ADCCMD_WR_REG, REG_SDO_CNTL1, SDO_CNTL1_UCMODE | SDO_CNTL1_RIGHT_ALIGN);
}

void ADS8167::enableSeqStatus(bool en) {
  uint8_t val = (en) ? SDO_CNTL4_SEQSTS_CFG : 0x00;
  write_cmd(ADCCMD_WR_REG, REG_SDO_CNTL4, val);
}

void ADS8167::enableParityBit(bool en) {
  uint8_t val = (en) ? PARITY_CNTL_PARITY_EN : 0x00;
  write_cmd(ADCCMD_WR_REG, REG_PARITY_CNTL, val);
}

/*
 * sets all inputs to be distinct. As opposed to differential inputs.
 */
void ADS8167::setAllInputsSeparate(){
  write_cmd(ADCCMD_WR_REG, REG_AIN_CFG, 0x00);
  write_cmd(ADCCMD_WR_REG, REG_COM_CFG, 0x00);
}

/*
 * for manual mode.
 */
void ADS8167::setChannel(const uint8_t channelno) {
    #ifdef DEBUG
      Serial.println("set channel " + String(channelno));
      Serial.flush();
    #endif
    write_cmd(ADCCMD_WR_REG, REG_CHANNEL_ID, channelno & 0x07); // 3 lsb 1, rest 0
}

void ADS8167::setManualMode() {
  write_cmd(ADCCMD_WR_REG, REG_DEVICE_CFG, DEVICE_CFG_SEQMODE_MANUAL);
}

void ADS8167::setAutoSequenceMode() {
  write_cmd(ADCCMD_WR_REG, REG_DEVICE_CFG, DEVICE_CFG_SEQMODE_AUTO);
}

void ADS8167::setSequence(const uint8_t length, const uint8_t* channels, const uint8_t repeat, const bool loop) {
  if(length < 1 || length > 16) {
    #ifdef DEBUG
      Serial.println("Sequence max length = 16");
      Serial.flush();
    #endif
    exit(1);
  }

  // Sequence mode enabled
  write_cmd(ADCCMD_WR_REG, REG_DEVICE_CFG, DEVICE_CFG_SEQMODE_CUSTOM);

  // Set channel sequence indexes
  for(uint8_t c=0; c<length; c++)
  {
      write_cmd(ADCCMD_WR_REG, REG_CCS_CHID_IDX(c), channels[c]);
      write_cmd(ADCCMD_WR_REG, REG_CCS_REPEAT_IDX(c), repeat);
  };

  // Repeat sequence, loop sequence
  write_cmd(ADCCMD_WR_REG, REG_CCS_SEQ_LOOP, loop ? CCS_SEQ_LOOP_EN : 0x00);

  // Sequence length, start-stop
  write_cmd(ADCCMD_WR_REG, REG_CCS_START_INDEX, 0);
  write_cmd(ADCCMD_WR_REG, REG_CCS_END_INDEX, length-1);
}

void ADS8167::sequenceStart() {
  write_cmd(ADCCMD_WR_REG, REG_SEQ_START, SEQ_START_START);
}

/**
 * commandto switch to AINy is sent in the Nth cycle and the data corresponding to channel AINy is available in the (N + 2)th cycle.
 * See section 7.4.1.1 (pg 31) : Manual mode of the data sheet.
 */
uint16_t ADS8167::readChannel(uint8_t* channel_out) {
  waitForDataReady();

  _theSPI->beginTransaction(SPISettings(_SPI_SPEED, MSBFIRST, SPI_MODE0));
  digitalWriteFast(_cs_pin, LOW);

  buffer[0] = 0x00;
  buffer[1] = 0x00;
  _theSPI->transfer(buffer, 2);

  digitalWriteFast(_cs_pin, HIGH);
  _theSPI->endTransaction();

  waitForDataReady();

  _theSPI->beginTransaction(SPISettings(_SPI_SPEED, MSBFIRST, SPI_MODE0));
  digitalWriteFast(_cs_pin, LOW);

  buffer[0] = 0x00;
  buffer[1] = 0x00;
  _theSPI->transfer(buffer, 2);

  digitalWriteFast(_cs_pin, HIGH);
  _theSPI->endTransaction();

  // if(channel_out != NULL)
  //   *channel_out = buffer[2] >> 4;

  #ifdef DEBUG
    Serial.println("raw reads (4 bytes)");
    Serial.println(buffer[0], BIN);
    Serial.println(buffer[1], BIN);
    Serial.println(buffer[2], BIN);
    Serial.println(buffer[3], BIN);
  #endif
  uint16_t result = buffer[0] << 8;
  result |= buffer[1];
  return result;
  // return buffer[1] << 8 | buffer[2];
}

long ADS8167::readData(uint8_t channel_no) {
  setChannel(channel_no);
  return readChannel();
}

void ADS8167::enableOTFMode() {
    write_cmd(ADCCMD_WR_REG, REG_DEVICE_CFG, DEVICE_CFG_SEQMODE_ONTHEFLY);
    write_cmd(ADCCMD_WR_REG, REG_ON_THE_FLY_CFG, ON_THE_FLY_CFG_OTF_EN);
}

void ADS8167::waitForDataReady() {
  while(digitalRead(_rdy_pin)==0) {
    #ifdef DEBUG
      Serial.println("adc waiting");
      Serial.flush();
    #endif
  }
}

uint16_t ADS8167::readChannelOTF(const uint8_t otf_next_channel, uint8_t* channel_out) {
    uint8_t cmd = ADCCMD_ONTHEFLY | (otf_next_channel & 0x07);

    // There appears to be a 'feature' in the adc where setting the otf channel repeatedly to
    // same one will return 0xFFFF.
    // static uint8_t last_otf = 0xFF;
    // if(last_otf == otf_next_channel)
    //     cmd = 0x00;
    // last_otf = otf_next_channel;

    waitForDataReady();

    _theSPI->beginTransaction(SPISettings(_SPI_SPEED, MSBFIRST, SPI_MODE0)); // chip uses mode 0 by default
    digitalWrite(_cs_pin, LOW);

    buffer[0] = cmd << 3;
    buffer[1] = 0x00;
    buffer[2] = 0x00;

    _theSPI->transfer(buffer, 3);

    digitalWrite(_cs_pin, HIGH);
    _theSPI->endTransaction();

    if(channel_out != NULL)
        *channel_out = buffer[2] >> 4;

    return buffer[0] << 8 | buffer[1];
}

void ADS8167::write_cmd(const adc_cmd_t cmd, const uint16_t address, const uint8_t data) {
    _theSPI->beginTransaction(SPISettings(_SPI_SPEED, MSBFIRST, SPI_MODE0)); // chip uses mode 0 by default
    digitalWrite(_cs_pin, LOW);
    uint32_t writeData = (cmd << 3) << 16; // Top 3 address bits are discarded, cmd is 5 bits
    writeData  |= address << 8; // Top 5 address bits are discarded, address is 11 bits
    writeData  |= data;
    buffer[0] = (writeData >> 16) & 0xFF;
    buffer[1] = (writeData >> 8) & 0xFF;
    buffer[2] = (writeData) & 0xFF;
    _theSPI->transfer(buffer, 3);

    _theSPI->endTransaction();

    digitalWrite(_cs_pin, HIGH);
}
