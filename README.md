# All Firmware

![PlatformIO CI](https://github.com/Space-Enterprise-at-Berkeley/Firmware/workflows/PlatformIO%20CI/badge.svg?branch=master)

Contains all microcontroller code for any and all boards we develop.
See specific dir documentation for more details.
## Setup
We use the PlatformIO build system. To get started, follow the
instructions on this page: https://platformio.org/install. You
can either install the VS Code extension, or the CLI directly.
Once it's installed, you can build all environments by running
`pio run` in your terminal. Works on windows, mac and linux.

## Environments
You can build the code for a specific rocket by running `pio run -e <name>`.
For example, build the E1 code by running `pio run -e E1`. To upload
the code for a specific environment, run `pio run -e <name> -t upload`.
If the specified environment is for Teensy, it should open the teensy loader
GUI.

See `platformio.ini` for a list of current environments.
