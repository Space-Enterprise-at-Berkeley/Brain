/**
 *
 * HX711 library for Arduino
 *
 * Created by Bruce Deng, Feb 6, 2021.
**/
#ifndef __loadCell__
#define __loadCell__

#include <HX711.h>

namespace LoadCell {

		HX711 _loadcell_1;
		HX711 _loadcell_2;
		int _numSensors;
		HX711 *_loadcells;

		byte  *_dout_pins;
		byte  *_sck_pins;
		float *_calibration_vals;

		int init(HX711 *loadcells, int numSensors, byte  *dout_pins,
									byte * sck_pins, float  *calibration_vals) {

			_loadcells = loadcells;

			_numSensors = numSensors;
			_dout_pins = dout_pins;
			_sck_pins = sck_pins;
			_calibration_vals = calibration_vals;

		  return 0;
		}


		int init(HX711 loadcell1, HX711 loadcell2, int numSensors, byte  *dout_pins,
		 							byte * sck_pins, float  *calibration_vals) {

      _loadcell_1 = loadcell1;
			_loadcell_2 = loadcell2;

			_numSensors = numSensors;
			_dout_pins = dout_pins;
			_sck_pins = sck_pins;
			_calibration_vals = calibration_vals;

//~~~~~~Needs to happen wherever loadcell.h is used~~~~~~~~~
//     for (int i = 0; i < numSensors; i++) {
//     loadcells[i].begin(doutPins[i], sckPins[i]);
//     loadcells[i].set_scale(calVals[i]);
//     loadcells[i].tare();
//
//     long zero_factor = loadcells[i].read_average(); //Get a baseline reading
//     Serial.print("Zero factor: "); //This can be used to remove the need to tare the scale. Useful in permanent scale projects.
//     Serial.println(zero_factor);
//     }
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


		 return 0;
		}

		// void readLoadCells(float *data) {

		// 	for (int i = 0; i < _numSensors; i++) {
		// 		data[i] = _loadcells[i].get_units() * 0.453592;
		// 	}
		// 	data[3] = data[0] + data[1];
		// 	data[_numSensors+1] = -1;
		// }

		void readLoadCells(float *data) {
		
			data[0] = _loadcells[0].get_units() * 0.453592;
			data[1] = _loadcells[1].get_units() * 0.453592;
			data[2] = data[0] + data[1];
			data[3] = _loadcells[2].get_units() * 0.453592;
			data[4] = -1;
		}

}

#endif
