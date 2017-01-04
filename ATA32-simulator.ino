/*
   Name:    ATA32-simulator
   Purpose: F27 Friendship Landing Gear Simulator

   Author:  MBUR
   Github:  https://github.com/MartijnvdB/ATA32-simulator
   Date:    started Dec 2016

   uses SPI for interfacing with MBI5168 LED driver.
   uses arduino-fsm to implement a finite state machine.
    Based on arduino-fsm 2.2.0 with pull request #5 9https://github.com/jonblack/arduino-fsm/pull/5)

*/


// http://www.humblecoder.com/arduino-finite-state-machine-library/
// https://github.com/jonblack/arduino-fsm
#include <SPI.h>
#include <Fsm.h>
#include "LandingGearSwitches/LandingGearSwitches.h"

// Timer library from https://github.com/JChristensen/Timer.
#include <Event.h>
#include <Timer.h>



/* MBI5168 related pins */
#define OE_PIN 9       // connected to MBI5168 OE, note: is PWM pin
#define LE_PIN 10      // connected to MBI5168 LE
#define MOSI_PIN 11    // SPI Master Out Slave In, connected to MBI5168 SDI
#define SCK_PIN 13     // Clock pin, connected to MBI5168 CLK

/* Digital pins connected to solenoid control and 4051 multiplexer */
#define SOLENOID_PIN 2
#define MPLEX_SELECT_PIN_A 3
#define MPLEX_SELECT_PIN_B 4
#define MPLEX_SELECT_PIN_C 5
#define MPLEX_READ_PIN  6


// define some macros for controlling the MBI5168
#define DISPLAY_ON digitalWrite(OE_PIN, LOW)
#define DISPLAY_OFF digitalWrite(OE_PIN, HIGH)
#define LATCH digitalWrite(LE_PIN, HIGH)
#define LATCH_OFF digitalWrite(LE_PIN, LOW)


#define NUM_LEDS 7 // Used in POST routine
#define LOCK_UNLOCK_TIME_MSEC 750 // estimated time req'd to unlock up/downlock
#define GEAR_EXTEND_TIME_SEC 4  // time for gear extension
#define GEAR_RETRACT_TIME_SEC 2 // time for gear retraction
#define GEAR_ALTNDN_TIME_SEC 5  // time for gear alternate down


/* SPI address - for LEDs and solenoid - is sent to MBI5168.
 * bit 0-2: red LEDs
 * bit 3-6: green LEDs
 * bit 7: solenoid (not controlled by MBI5168 - too much current)
 */
byte address = B00000000;


// State transition events
#define POWER_ON 10
#define POWER_OFF 15
#define LIFT_OFF 20
#define TOUCH_DOWN 30
#define GEAR_UP 40
#define GEAR_DOWN_NORM 50
#define GEAR_DOWN_ALTN 60



/*
   Define the states.
   A state has two callback functions associated with it: on_enter and on_exit, which are called when the state is entered into and exited from, respectively.
   Callbacks can be NULL (at least, the second one)
*/
State state_gnd_powered_off(&on_power_off_enter, &on_power_off_exit);
State state_gnd_powered_on(&on_gnd_powered_on_enter, &on_gnd_powered_on_exit);
State state_dnlocked_flt(&on_dnlocked_flt_enter, &on_dnlocked_flt_exit);
State state_unlock_dnlock(&on_unlock_dnlock_enter, &on_unlock_dnlock_exit);
State state_transit_up(&on_transit_up_enter, &on_transit_up_exit);
State state_up(&on_lock_uplock_enter, &on_lock_uplock_exit);
State state_unlock_uplock(&on_unlock_uplock_enter, &on_unlock_uplock_exit);
State state_transit_dn(&on_transit_dn_enter, &on_transit_dn_enter);
State state_altn_unlock_uplock(&on_altn_unlock_uplock_enter, &on_altn_unlock_uplock_exit);
State state_altn_transit_down(&on_altn_transit_down_enter, &on_altn_transit_down_exit);


/*
   On-enter and on-exit functions.
*/
void on_power_off_enter() {
  address = B00000000;
}
void on_power_off_exit() {
  poweronselftest();
}

void on_gnd_powered_on_enter() {
  //  release_solenoid, lights on
  address = B01111000;
}
void on_gnd_powered_on_exit() {}

void on_dnlocked_flt_enter() {
  // retract solenoid, lights on
  address = B11111000;
}
void on_dnlocked_flt_exit() {}

void on_unlock_dnlock_enter() {}
void on_unlock_dnlock_exit() {
  address = B00000111;
}

void on_transit_up_enter() {}
void on_transit_up_exit() {}

void on_lock_uplock_enter() {
  address = B00000000;
}
void on_lock_uplock_exit() {}

void on_unlock_uplock_enter() {}
void on_unlock_uplock_exit() {
  address = B10000111;
}

void on_transit_dn_enter() {}
void on_transit_dn_exit() {}

void on_altn_unlock_uplock_enter() {}
void on_altn_unlock_uplock_exit() {
  address = B00000111;
}

void on_altn_transit_down_enter() {}
void on_altn_transit_down_exit() {}


/*
   State transition callback functions.
*/
void poweronselftest() {
  byte mask = B00000001;

  DISPLAY_ON;    // enable output
  update_peripherals(B00000000);

  for (int i = 0; i < NUM_LEDS; i++) {
    byte address = B00000000;
    update_peripherals(address |= mask);
    delay(100);
    mask <<= 1;  // shift a bit to the left
  }
  update_peripherals(B01111111); // all LEDs on

  delay(100);

  DISPLAY_OFF;
  update_peripherals(B10000000);
  delay(100);
  update_peripherals(B00000000);
  DISPLAY_ON;
}

// Optional functions that can be executed when changing a state. Not used.
//void on_trans_gnd_powered_off_powered_on() {}
//void on_trans_gnd_powered_on_powered_off() {}
//void on_trans_gnd_powered_on_dnlocked_flt() {}
//void on_trans_dnlocked_flt_powered_on_gnd() {}
//void on_trans_gear_up() {}


Fsm fsm(&state_gnd_powered_off);
LandingGearSwitches LG(MPLEX_SELECT_PIN_A, MPLEX_SELECT_PIN_B, MPLEX_SELECT_PIN_C, MPLEX_READ_PIN);


/* Other functions */

/* Write address to SPI bus, controlling the LEDs and solenoid. */
void update_peripherals(int address) {
  DISPLAY_OFF;
  LATCH;         // enable serial data

  // LED control is influenced by availability of power busses
  if (! LG.essdc_state()) {
    address &= B10111111;
  }
  if (! LG.maindc_state()) {
    address &= B01000000;
  }
  //  send in the address and value via SPI:
  SPI.transfer(address);  // transfer one byte
  LATCH_OFF;  // disable serial data input
  DISPLAY_ON;

  // solenoid control
  digitalWrite(SOLENOID_PIN, (address >> 7) );// MSB of address determines solenoid control

} // update_peripherals


void setup() {
  pinMode(SOLENOID_PIN, OUTPUT);
  pinMode(LE_PIN,  OUTPUT);
  pinMode(OE_PIN, OUTPUT);
  pinMode(MOSI_PIN, OUTPUT);
  pinMode (SCK_PIN, OUTPUT);

  // initialize SPI
  SPI.beginTransaction(SPISettings(25000000, MSBFIRST, SPI_MODE0));

  /* Define state transitions
     void add_transition(State* state_from, State* state_to, int event, void (*on_transition)());
     void add_timed_transition(State* state_from, State* state_to, unsigned long interval, void (*on_transition)());
     --> Last argument may be NULL
  */
  fsm.add_transition(&state_gnd_powered_off, &state_gnd_powered_on, POWER_ON, NULL); //&on_trans_gnd_powered_off_powered_on);
  fsm.add_transition(&state_gnd_powered_on, &state_dnlocked_flt, LIFT_OFF, NULL); // &on_trans_gnd_powered_on_dnlocked_flt);

  // gear up
  fsm.add_transition(&state_dnlocked_flt, &state_unlock_dnlock, GEAR_UP, NULL);
  fsm.add_timed_transition(&state_unlock_dnlock, &state_transit_up, LOCK_UNLOCK_TIME_MSEC, NULL);
  fsm.add_timed_transition(&state_transit_up, &state_up, GEAR_RETRACT_TIME_SEC * 1000, NULL); //&on_trans_gear_up);

  // gear down
  fsm.add_transition(&state_up, &state_unlock_uplock, GEAR_DOWN_NORM, NULL); //&on_trans_gear_up);
  fsm.add_timed_transition(&state_unlock_uplock, &state_transit_dn, LOCK_UNLOCK_TIME_MSEC, NULL);
  fsm.add_timed_transition(&state_transit_dn, &state_dnlocked_flt, GEAR_EXTEND_TIME_SEC * 1000, NULL); //&on_trans_gear_up);

  // gear up while in transit down
  fsm.add_transition(&state_transit_dn, &state_transit_up, GEAR_UP, NULL);

  // gear down while in transit up
  fsm.add_transition(&state_transit_up, &state_transit_dn, GEAR_DOWN_NORM, NULL);

  // touch down
  fsm.add_transition(&state_dnlocked_flt, &state_gnd_powered_on, TOUCH_DOWN, NULL);
  // power off

  fsm.add_transition(&state_gnd_powered_on, &state_gnd_powered_off, POWER_OFF, NULL);

  // alternate down
  fsm.add_transition(&state_up, &state_altn_unlock_uplock, GEAR_DOWN_ALTN, NULL);
  fsm.add_timed_transition(&state_altn_unlock_uplock, &state_altn_transit_down, LOCK_UNLOCK_TIME_MSEC, NULL);
  fsm.add_timed_transition(&state_altn_transit_down, &state_dnlocked_flt, GEAR_EXTEND_TIME_SEC * 1000, NULL);

  address = B00000000;  // all LEDs and solenoid OFF

} // setup


void loop() {
  LG.update();  // gets/stores all switch states
  
  if (LG.maindc_state() && LG.essdc_state() && LG.air_state()) {
    fsm.trigger(POWER_ON);
  }
  if ( (! LG.maindc_state()) && (! LG.essdc_state())  && (! LG.air_state()) ) {
    fsm.trigger(POWER_OFF);
  }
  if (LG.gndflt_state()) {
    fsm.trigger(LIFT_OFF);
  }
  if (! LG.gndflt_state()) {
    fsm.trigger(TOUCH_DOWN);
  }
  // GEAR UP/DN now assumes that an 'alternate gear down' dumps all pneumatics
  if (LG.lglever_state() && LG.air_state() && LG.lg_altndn_state() ) {
    fsm.trigger(GEAR_UP);
  }
  if ( (! LG.lglever_state()) && LG.air_state() && LG.lg_altndn_state()) {
    fsm.trigger(GEAR_DOWN_NORM);
  }
  if (! LG.lg_altndn_state()) {
    fsm.trigger(GEAR_DOWN_ALTN);
  }

  fsm.check_timer();

  // update LED and solenoid 'status'
  update_peripherals(address);

} // loop


/* END */

