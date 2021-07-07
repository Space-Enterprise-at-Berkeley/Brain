# Main Flight Code

All the different versions of the flight code are defined in this directory with appropriate
environments defined in the `platformio.ini` file. Each different directory has a `.cpp` file
along with a `config.h` file. The config file defines all the sensors, actuators, and commands used by this version of
the code, which is associated with a specific version of the hardware. Since the high level architecture
is all very similar, I will explain a single example here: `E1_coldflow_v2`.

## config.h

Most sensors and input devices are defined in a similar format with a `const uint8_t` defining the number of instances of that sensor/input device followed by a set of arrays defining any associated configuration variables for each instance of the sensor. The following is an example for the thermocouples:
```
// Cryo Thermocouple
const uint8_t numCryoTherms = 4;
uint8_t cryoThermAddrs[numCryoTherms] = {0x60, 0x61, 0x62, 0x63};
_themotype cryoTypes[numCryoTherms] = {MCP9600_TYPE_K, MCP9600_TYPE_K, MCP9600_TYPE_K, MCP9600_TYPE_K};
Adafruit_MCP9600 _cryo_boards[numCryoTherms];
float cryoReadsBackingStore[numCryoTherms];
```

`cryoThermAddrs` defines the I2C addresses for each of the sensors, `cryoTypes` defines a config value for each thermocouple (the `_themotype` data type is defined in the thermocouple library we're using), `_cryo_boards` is the array of instance variables for the thermocouple objects themselves (this will get passed to the thermocouple library to use locally), `cryoReadsBackingStore` is a float array that holds the most recent reads from the thermocouples.

After defining all the sensors and actuators that this code uses, we define the commands that the code listens for. Most of the commands are defined within libraries and the ids are specified in the config file, however, we may define some commands directly in this file. In the following segment we create an array that has references to all the commands we've defined. See `../lib/communications/README.md` for more details about the commands.
```
Command *backingStore[numCommands] = {...};
CommandArray commands(numCommands, backingStore);
```

Within the `setup` function, we instantiate all the objects and call appropriate setup functions for each object.

Following that, we define all the sensors.

## E1_coldflow_v21.cpp


The `setup` function initializes all of our libraries, passing in the relevant configurations defined in `config.h`.
We also populate the `sensor_checks` object: a 2d int array, with shape `[<numSensors>, 2]`, where `sensor_checks[i][0]` stores the cycle_period of the ith sensor, and `sensor_checks[i][1]` stores a counter that gets incremented for each iteration through the `loop`. We'll see this used in the "Reading from Sensors" section below.

The `loop` function can be broken up into 3 functional parts:
1. Receiving and Parsing Commands
2. Conducting Automation Sequences
3. Reading from Sensors

### Receiving and Parsing Commands
We receive packets from both Ethernet and Serial channels. We always check the serial channel, and if this is a ground test, ie. there will be ethernet, we'll also check ethernet for any incoming commands. If we are checking via ethernet, we make sure that the command came from an expected ip address. These are defined in config.
Once we capture a set of data, we pass it to the `processCommand` func in `common`. The process command is explained in more detail in the docs for the common library, but it essentially verifies that the packet is of a valid format, and it completes the action that the command requests (opening a valve, increasing a heater output, etc).

### Automation Sequences

### Sensor Reading

Different sensors are read at different rates defined by the `cycle_period` parameter of the `sensorInfo` struct. The larger `cycle_period`, the less often this sensor is read.
The `sensor_checks` 2d array is used to keep track of when its time to read a sensor.

Within the `loop` function, there's a secondary `for` loop that goes through each sensor. In order to decide if the `i`th sensor will be read in this iteration, `sensor_checks[i][1]` (the counter) is compared with `sensor_checks[i][0]` (`cycle_period`). When the two are equal, the sensor is read from, else we just increment the counter. This results in each sensor being read from only 1/ `cycle_period` iterations. NOTE: It is possible to achieve the same effect w/out this internal loop. TODO: @nlautrette.

Once we decide on reading from a sensor, we read from it (see `sensorReadFunc`), and transmit a packet with the results.
