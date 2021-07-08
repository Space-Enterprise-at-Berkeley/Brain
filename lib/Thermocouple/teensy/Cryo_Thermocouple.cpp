/*
  Cryo_Thermocouple.cpp - A c++ library to interface with heavy duty cryo/high temp
  thermocouples using the MCP9600 chip.
  Created by Vamshi Balanaga, Feb 5, 2021.
*/



#include <Cryo_Thermocouple.h>

namespace Thermocouple {

    int Cryo::init(uint8_t numSensors, Adafruit_MCP9600 *cryo_boards, uint8_t * addrs, _themotype * types, TwoWire *theWire, float *latestReads) { // assume that numSensors is < max Size of packet. Add some error checking here
      _addrs = addrs;
      _latestReads = latestReads;
      _cryo_amp_boards = cryo_boards;

      _numSensors = numSensors;

      for (int i = 0; i < numSensors; i++) {

        if (!_cryo_amp_boards[i].begin(addrs[i], theWire)) {
          Serial.println("Error initializing cryo board at Addr 0x" + String(addrs[i], HEX));
          return -1;
        }

        _cryo_amp_boards[i].setADCresolution(MCP9600_ADCRESOLUTION_16);
        _cryo_amp_boards[i].setThermocoupleType(types[i]);
        _cryo_amp_boards[i].setFilterCoefficient(0);
        _cryo_amp_boards[i].enable(true);
      }

      for (int i = 0; i < _numSensors; i++) {
        Serial.println(i);
        Serial.println(_latestReads[i]);
        _latestReads[i] = -1;
        Serial.println(_latestReads[i]);
        _latestReads[i] = _cryo_amp_boards[i].readThermocouple();
        Serial.println(_latestReads[i]);
      }

      return 0;
    }
    

    int Cryo::lowerI2CSpeed(TwoWire *theWire) {
      // Need to lower the I2C clock frequency
      /* START SET I2C CLOCK FREQ */
      #define CLOCK_STRETCH_TIMEOUT 15000
      IMXRT_LPI2C_t *port;
      if (theWire == &Wire) {
        port = &IMXRT_LPI2C1;
      } else if (theWire == &Wire1) {
        port = &IMXRT_LPI2C3;
      } else if (theWire == &Wire2) {
        port = &IMXRT_LPI2C4;
      } else {
        Serial.println("Cryo::lowerI2CSpeed - ERROR: UNRECOGNIZED WIRE OBJECT. MUST BE Wire, Wire1, or Wire2");
        return -1;
        _lowerI2CSpeed = false;
      }
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
      _lowerI2CSpeed = true;
      return 0;
    }

    void Cryo::readCryoTemps(float *data) {
      if (!_lowerI2CSpeed) {
        Serial.println("Cryo-Thermocouples - WARNING: Not using lower I2C bus speed. Make sure to call 'Cryo::lowerI2CSpeed' during setup");
      }
      #ifdef DEBUG
      Serial.println("read Cryo temps");
      Serial.flush();
      #endif
      for (int i = 0; i < _numSensors; i++) {
        #ifdef DEBUG
        Serial.print(i);
        Serial.flush();
        #endif
        data[i] = _cryo_amp_boards[i].readThermocouple();
        // data[i] = _cryo_amp_boards[i].readAmbient();
        // data[i] = _cryo_amp_boards[i].readADC();
        #ifdef DEBUG
        Serial.print(data[i]);
        Serial.flush();
        #endif
        _latestReads[i] = data[i];
      }
      data[_numSensors] = -1;
    }

    void Cryo::readSpecificCryoTemp(uint8_t index, float *data) {
      data[0] = _latestReads[index];
      data[1] = -1;
    }

    int Cryo::freeAllResources() {
        free(_latestReads);
        return 0;
    }

  }
