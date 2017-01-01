/* Name:    LandingGearSwitches
   Purpose: Object for Landing Gear Simulator switches, to be used with
            the 4051 multiplexer/demultiplexer.
   Author:  Martijn van den Burg, martijn@[remove-me-first]palebluedot . nl
   
   Copyright (C) 2016 Martijn van den Burg. All right reserved.

   This program is free software: you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   This program is distributed in the hope that it will be useful, but WITHOUT ANY
   WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
   for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/ 


#include "Arduino.h"
#include "LandingGearSwitches.h"


/* 
 * Constructor.
 * Gets as arguments the numbers of the Arduino digital pins that control
 * the 4051, and the digital pin used to read the selected value from the
 * 4051.
 */
LandingGearSwitches::LandingGearSwitches(int c0, int c1, int c2, int d) {
  control_pin_A = c0;
  control_pin_B = c1;
  control_pin_C = c2;
  data_pin = d;
  
  init();
} 


/*
 * Destructor
 */
LandingGearSwitches::~LandingGearSwitches() { /* nothing to do */ }


/* Initialize Arduino pins. */
void LandingGearSwitches::init() {
  pinMode(control_pin_A, OUTPUT);
  pinMode(control_pin_B, OUTPUT);
  pinMode(control_pin_C, OUTPUT);
  
  pinMode(data_pin, INPUT);
} // init


/* Read all switch states and update the private data. This is cheaper
 * than reading them on demand in the loop().
 */
void LandingGearSwitches::update() {
  // Read switch settings into an array.
  for (int i = 0; i < NUMSWITCHES; i++) {
     set_input_state(i);
     switchesArray[i] = read_output_state();
  }
}

/*
 * Public accessor switch positions.
 */
byte LandingGearSwitches::gndflt_state() {
  return switchesArray[0];
}
byte LandingGearSwitches::maindc_state() {
  return switchesArray[1];
}
byte LandingGearSwitches::essdc_state() {
  return switchesArray[2];
}
byte LandingGearSwitches::air_state() {
  return switchesArray[3];
}
byte LandingGearSwitches::lglever_state() {
  return switchesArray[4];
}
byte LandingGearSwitches::lgovrd_state() {
  return switchesArray[5];
}
byte LandingGearSwitches::lg_altndn_state() {
  return switchesArray[6];
}


/* Control the select inputs of the 4051 by writing a 0 or a 1 to them.
 * Input is the byte value of the switch we want to select.
 * 
 * CBA
 * 000 gnd/flight switch
 * 001 main dc bus switch
 * 010 ess dc bus switch
 * 011 air switch
 * 100 landing gear lever
 * 101 land gear override
 * 110 landing gear altn down
 * 111 - not used -
 */
void LandingGearSwitches::set_input_state(byte input) {
  // Get values of input byte's individual bits,
  // AND the last one with a bit mask to determine if it's 0 or 1.
  byte a = input;
  byte b = input >> 1;
  byte c = input >> 2;

  digitalWrite(control_pin_A, (a &= B00000001));
  digitalWrite(control_pin_B, (b &= B00000001));
  digitalWrite(control_pin_C, (c &= B00000001));
}


/* Return the value of the selected input pin by reading the data pin. */
byte LandingGearSwitches::read_output_state() {
  return digitalRead(data_pin);
}
    
    
/* End */