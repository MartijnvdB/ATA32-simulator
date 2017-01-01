/*
 LandingGearSwitches.h 
 
 Copyright 2016 Martijn van den Burg, martijn@[remove-me-first]palebluedot . nl
 
 This file is part of the LandingGearSwitches library for manipulating the 4051 IC
 with Arduino.
 
 LandingGearSwitches is free software: you can redistribute it and/or modify it under
 the terms of the GNU General Public License as published by the Free
 Software Foundation, either version 3 of the License, or (at your option)
 any later version.
 
 This software is distributed in the hope that it will be useful, but WITHOUT ANY
 WARRANTY; without even the implied warranty of MERCHANTABILITY or
 FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 for more details.
   
   You should have received a copy of the GNU General Public License
   along with LandingGearSwitches.  If not, see <http://www.gnu.org/licenses/>.
   
   */


/* This library is written for the Arduino Duemilanova and Arduino software version 1.6.8.
 * 
 * This library may work with other hardware and/or software. YMMV.
 */

#include "Arduino.h"

#ifndef LandingGearSwitches_h
#define LandingGearSwitches_h

#define VERSION 1.0

#define NUMSWITCHES 7


class LandingGearSwitches {
  public:
    LandingGearSwitches(int, int, int, int);
    ~LandingGearSwitches();

    // public functions, accessors
    void update();
    byte gndflt_state();
    byte maindc_state();
    byte essdc_state();
    byte air_state();
    byte lglever_state();
    byte lgovrd_state();
    byte lg_altndn_state();

  private:
    byte switchesArray[NUMSWITCHES];
    byte control_pin_A;	// used to control 4051
    byte control_pin_B;	// used to control 4051
    byte control_pin_C;	// used to control 4051
    byte data_pin;	// read data from 4051

    void init();
    void set_input_state(byte);
    byte read_output_state();
};


#endif