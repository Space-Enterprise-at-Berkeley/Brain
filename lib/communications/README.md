# Communications Library

The Packet format that we use includes a checksum based on data, this is all well documented wherever our docs live. (They should move here, but someone else should do that.)

This checksum is not something you can calculate in your head, so a python script exists that takes in a string command and gives you a fully formed packet w/ the checksum. See [here](https://github.com/Space-Enterprise-at-Berkeley/Tests/blob/master/packet_gen.py).

The `command.h` file defines an abstract class Command that has a virtual function `parseCommand` and `confirmation`. These functions must be implemented by every implementation of the Command abstract class. This abstract class creates a common format for dealing w/ all commands by using the two above functions, but it allows each specific type of function to also contain other functions that it needs for extra processing without cluttering a single class with code that is specific to SolenoidCommands and other code that is specific to AutomationSequenceCommands. See `lib/solenoids/solenoids.h` for the SolenoidCommand implementation, or `lib/tempController/tempController.h` for the HeaterCommand.
