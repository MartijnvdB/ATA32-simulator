# ATA32-simulator

This code is a proof of concept for a Fokker F27 Friendship landing gear simulator. It is written for Arduino (1.6.8).

A friend of mine owns some old, inoperative, simulator hardware which he would like to be able to use. The original simulation interfaces and software are not available anymore. Perhaps it is possible to re-implement them with Arduino (or ESP8266, Nodemcu, Raspbery Pi...).

## Installing / Getting started

This software is probably of little use to anyone else because it depends on hardware that I made myself. The hardware implements a rudimentary airplane landing gear interface, including:
* ground/flight switch
* electrical busses switches
* pneumatics switch
* landing gear controls (normal, override, alternate)
* status indication LEDs

However, if you're interested, here is how to install it:

* Install the libraries that this software depends on:
  - SPI
  - arduino-fsm (https://github.com/jonblack/arduino-fsm)
  - Timer (https://github.com/JChristensen/Timer)
* Download the software into your sketches folder
* Open the software in the Arduino GUI and compile it.

## Developing

No outside help is currently needed.

### Building

Build and upload the software using the Arduino GUI.

## Features

This project inmplements a simplified landing gear system using a finite state machine. It interacts with the 'airplane' and the 'pilot' using a couple of electrical swithes, a solenoid and status LEDs.

This part of the 'happy' flow is currently implemented:
* power on/off
* lift off/touch down
* normal gear up
* normal gear down
* alternate gear down

This part of the exception flow is currently implemented in a static way (i.e. absence of power *before* changing states,
not *while* changing states:
* electrical bus failure
* pneumatics

Future implementations should allow for an 'instructor' to set specific failure conditions dynamically and during state transitions. For example:
* pneumatic failures
* mechanical failures

## Licensing

The code in this project is licensed under the GNU General Public License v3.0.

