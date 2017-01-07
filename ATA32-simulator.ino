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
#define LOCK_UNLOCK_TIME_MSEC 1000 // estimated time req'd to unlock up/downlock
#define GEAR_EXTEND_TIME_MSEC 5000  // time for gear extension
#define GEAR_RETRACT_TIME_MSEC 3000 // time for gear retraction
#define GEAR_ALTNDN_TIME_MSEC 6000  // time for gear alternate down
#define GEAR_UP_TIME_DIFF_FACTOR 0.9
#define GEAR_DN_TIME_DIFF_FACTOR 0.9

/* SPI address - for LEDs and solenoid - is sent to MBI5168.
   bit 0-2: red LEDs, left to right
   bit 3-6: green LEDs, right to left, top
   bit 7: solenoid (not controlled by MBI5168 - too much current)
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
State state_transit_dn(&on_transit_dn_enter, &on_transit_dn_exit);
State state_altn_unlock_uplock(&on_altn_unlock_uplock_enter, &on_altn_unlock_uplock_exit);
State state_altn_transit_down(&on_altn_transit_down_enter, &on_altn_transit_down_exit);


/* Timer objects are used to be able to make each gear go up/down in their own time. */
Timer mlg_left;
Timer mlg_right;
Timer nlg;

int mlg_left_timerid, mlg_right_timerid, nlg_timerid;  // timer event IDs



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
  address = B01111000;  //  release_solenoid, green LEDs on
}
void on_gnd_powered_on_exit() {}

void on_dnlocked_flt_enter() {
  address |= B10000000;   // touch down. retract solenoid
}
void on_dnlocked_flt_exit() {}



/* In the transit up/down states we launch three timers with random times for the LEDs,
    to simulate different up rate/uplock times for the three gears, which should
    make it more realistic.

    This sort of decouples the illumination of the LEDs from the state of the FSM.
    Perhaps not in the spirit of a FSM, but the alternative would be to make separate
    states for each gear.

    Note: the /transit time/ and the LED indicator timer use the same time definition
    (e.g. GEAR_RETRACT_TIME_MSEC) but they occur one after the other in loop(). This could
    theoretically cause a race condition (LED state != FSM state, for some microseconds
    inside the loop()).
*/

void on_unlock_dnlock_enter() {
  mlg_left_timerid = mlg_left.after(random(GEAR_UP_TIME_DIFF_FACTOR * LOCK_UNLOCK_TIME_MSEC, LOCK_UNLOCK_TIME_MSEC), show_mlg_left_is_unlocked );  // last arg. is callback to execute
  mlg_right_timerid = mlg_right.after(random(GEAR_UP_TIME_DIFF_FACTOR * LOCK_UNLOCK_TIME_MSEC, LOCK_UNLOCK_TIME_MSEC), show_mlg_right_is_unlocked );
  nlg_timerid = nlg.after(random(GEAR_UP_TIME_DIFF_FACTOR * LOCK_UNLOCK_TIME_MSEC, LOCK_UNLOCK_TIME_MSEC), show_nlg_is_unlocked );
}
void on_unlock_dnlock_exit() {
  // stop the timers in case LG is selected down before it got up
  mlg_left.stop(mlg_left_timerid);
  mlg_right.stop(mlg_right_timerid);
  nlg.stop(nlg_timerid);
}

void on_transit_up_enter() {
  mlg_left_timerid = mlg_left.after(random(GEAR_UP_TIME_DIFF_FACTOR * GEAR_RETRACT_TIME_MSEC, GEAR_RETRACT_TIME_MSEC), show_mlg_left_is_up );  // last arg. is callback to execute
  mlg_right_timerid = mlg_right.after(random(GEAR_UP_TIME_DIFF_FACTOR * GEAR_RETRACT_TIME_MSEC, GEAR_RETRACT_TIME_MSEC), show_mlg_right_is_up );
  nlg_timerid = nlg.after(random(GEAR_UP_TIME_DIFF_FACTOR * GEAR_RETRACT_TIME_MSEC, GEAR_RETRACT_TIME_MSEC), show_nlg_is_up );
}
void on_transit_up_exit() {
  // stop the timers in case LG is selected down before it got up
  mlg_left.stop(mlg_left_timerid);
  mlg_right.stop(mlg_right_timerid);
  nlg.stop(nlg_timerid);
}

void on_lock_uplock_enter() {
  address &= B01111111; // retract solenoid
}
void on_lock_uplock_exit() {}

void on_unlock_uplock_enter() {
  mlg_left_timerid = mlg_left.after(random(GEAR_DN_TIME_DIFF_FACTOR * LOCK_UNLOCK_TIME_MSEC, LOCK_UNLOCK_TIME_MSEC), show_mlg_left_is_unlocked );  // last arg. is callback to execute
  mlg_right_timerid = mlg_right.after(random(GEAR_DN_TIME_DIFF_FACTOR * LOCK_UNLOCK_TIME_MSEC, LOCK_UNLOCK_TIME_MSEC), show_mlg_right_is_unlocked );
  nlg_timerid = nlg.after(random(GEAR_DN_TIME_DIFF_FACTOR * LOCK_UNLOCK_TIME_MSEC, LOCK_UNLOCK_TIME_MSEC), show_nlg_is_unlocked );
}
void on_unlock_uplock_exit() {
  // stop the timers in case LG is selected up before it got down
  mlg_left.stop(mlg_left_timerid);
  mlg_right.stop(mlg_right_timerid);
  nlg.stop(nlg_timerid);
  address |= B10000000;  // retract solenoid
}

void on_transit_dn_enter() {
  mlg_left.after(random(GEAR_DN_TIME_DIFF_FACTOR * GEAR_EXTEND_TIME_MSEC, GEAR_EXTEND_TIME_MSEC), show_mlg_left_is_dn );  // last arg. is callback to execute
  mlg_right.after(random(GEAR_DN_TIME_DIFF_FACTOR * GEAR_EXTEND_TIME_MSEC, GEAR_EXTEND_TIME_MSEC), show_mlg_right_is_dn );
  nlg.after(random(GEAR_DN_TIME_DIFF_FACTOR * GEAR_EXTEND_TIME_MSEC, GEAR_EXTEND_TIME_MSEC), show_nlg_is_dn );
}
void on_transit_dn_exit() {
  // stop the timers in case LG is selected up before it got down
  mlg_left.stop(mlg_left_timerid);
  mlg_right.stop(mlg_right_timerid);
  nlg.stop(nlg_timerid);
}

void on_altn_unlock_uplock_enter() {
  mlg_left_timerid = mlg_left.after(random(GEAR_DN_TIME_DIFF_FACTOR * LOCK_UNLOCK_TIME_MSEC, LOCK_UNLOCK_TIME_MSEC), show_mlg_left_is_unlocked );  // last arg. is callback to execute
  mlg_right_timerid = mlg_right.after(random(GEAR_DN_TIME_DIFF_FACTOR * LOCK_UNLOCK_TIME_MSEC, LOCK_UNLOCK_TIME_MSEC), show_mlg_right_is_unlocked );
  nlg_timerid = nlg.after(random(GEAR_DN_TIME_DIFF_FACTOR * LOCK_UNLOCK_TIME_MSEC, LOCK_UNLOCK_TIME_MSEC), show_nlg_is_unlocked );
}
void on_altn_unlock_uplock_exit() {}

void on_altn_transit_down_enter() {
  mlg_left.after(random(GEAR_DN_TIME_DIFF_FACTOR * GEAR_ALTNDN_TIME_MSEC, GEAR_ALTNDN_TIME_MSEC), show_mlg_left_is_dn );  // last arg. is callback to execute
  mlg_right.after(random(GEAR_DN_TIME_DIFF_FACTOR * GEAR_ALTNDN_TIME_MSEC, GEAR_ALTNDN_TIME_MSEC), show_mlg_right_is_dn );
  nlg.after(random(GEAR_DN_TIME_DIFF_FACTOR * GEAR_ALTNDN_TIME_MSEC, GEAR_ALTNDN_TIME_MSEC), show_nlg_is_dn );
}
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



/* Functions that set the right SPI address for controlling the LEDs.
    More readable than having them in the code.
*/
void show_mlg_left_is_up() {
  address &= B01011110;  // release solenoid
}
void show_mlg_left_is_unlocked() {
  address &= B11011111; // green off
  address |= B10000001; // red on, retract solenoid
}
void show_mlg_left_is_dn() {
  address |= B00100000; // green on
  address &= B11111110; // red off
}
void show_mlg_right_is_up() {
  address &= B11110011;
}
void show_mlg_right_is_unlocked() {
  address &= B11110111; // green off
  address |= B00000100; // red on
}
void show_mlg_right_is_dn() {
  address |= B00001000; // green on
  address &= B11111011; // red off
}
void show_nlg_is_up() {
  address &= B10101101;
}
void show_nlg_is_unlocked() {
  address &= B10101111; // green off
  address |= B00000010; // red on
}
void show_nlg_is_dn() {
  address |= B01010000; // green on
  address &= B11111101; // red off
}





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
  fsm.add_timed_transition(&state_transit_up, &state_up, GEAR_RETRACT_TIME_MSEC, NULL); //&on_trans_gear_up);

  // gear down
  fsm.add_transition(&state_up, &state_unlock_uplock, GEAR_DOWN_NORM, NULL); //&on_trans_gear_up);
  fsm.add_timed_transition(&state_unlock_uplock, &state_transit_dn, LOCK_UNLOCK_TIME_MSEC, NULL);
  fsm.add_timed_transition(&state_transit_dn, &state_dnlocked_flt, GEAR_EXTEND_TIME_MSEC, NULL); //&on_trans_gear_up);

  // gear up while in transit down

  // VVVVVVV
  fsm.add_transition(&state_unlock_uplock, &state_up, GEAR_UP, NULL); // while unlocking
  fsm.add_transition(&state_transit_dn, &state_transit_up, GEAR_UP, NULL);

  // gear down while in transit up
  fsm.add_transition(&state_unlock_dnlock, &state_dnlocked_flt, GEAR_DOWN_NORM, NULL); // while unlocking
  fsm.add_transition(&state_transit_up, &state_transit_dn, GEAR_DOWN_NORM, NULL); // while moving

  // touch down
  fsm.add_transition(&state_dnlocked_flt, &state_gnd_powered_on, TOUCH_DOWN, NULL);
  // power off

  fsm.add_transition(&state_gnd_powered_on, &state_gnd_powered_off, POWER_OFF, NULL);

  // alternate down
  fsm.add_transition(&state_up, &state_altn_unlock_uplock, GEAR_DOWN_ALTN, NULL);
  fsm.add_timed_transition(&state_altn_unlock_uplock, &state_altn_transit_down, LOCK_UNLOCK_TIME_MSEC, NULL);
  fsm.add_timed_transition(&state_altn_transit_down, &state_dnlocked_flt, GEAR_EXTEND_TIME_MSEC * 1000, NULL);

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
  /* GEAR UP/DN now assumes that an 'alternate gear down' dumps all pneumatics. */
  if (LG.lglever_state() && LG.air_state() && LG.lg_altndn_state() ) {
    fsm.trigger(GEAR_UP);
  }
  if ( (! LG.lglever_state()) && LG.air_state() && LG.lg_altndn_state()) {
    fsm.trigger(GEAR_DOWN_NORM);
  }
  if (! LG.lg_altndn_state()) {
    fsm.trigger(GEAR_DOWN_ALTN);
  }

  /* Update FSM timer-based transitions */
  fsm.check_timer();

  /* The global variable 'address' is manipulated in the various states.
     Here in loop() it controls the LEDs and solenoid.
  */
  update_peripherals(address);

  /* Update landing gear up/down timers. */
  mlg_left.update();
  mlg_right.update();
  nlg.update();

} // loop


/* END */

