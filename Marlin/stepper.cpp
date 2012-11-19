/*
 stepper.c - stepper motor driver: executes motion plans using stepper motors
 Part of Grbl

 Copyright (c) 2009-2011 Simen Svale Skogsrud

 Grbl is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Grbl is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
 */

/* The timer calculations of this module informed by the 'RepRap cartesian firmware' by Zack Smith
 and Philipp Tiefenbacher. */

#include "Marlin.h"
#include "stepper.h"
#include "planner.h"
#include "temperature.h"
#include "ultralcd.h"
#include "language.h"
#include "speed_lookuptable.h"
#include "L6470.h"

//===========================================================================
//=============================public variables  ============================
//===========================================================================
volatile block_t *current_block; // A pointer to the block currently being traced

//===========================================================================
//=============================private variables ============================
//===========================================================================
//static makes it inpossible to be called from outside of this file by extern.!

// Variables used by The Stepper Driver Interrupt

// true when starting a new block. Goes false after the first timer event completes  
static volatile boolean block_initial_step;
static volatile unsigned char L6470_x_dir, L6470_y_dir, L6470_z_dir,
		L6470_e_dir;
static volatile boolean L6470_complete = true; // Flag to stop the step/dir counters going while waiting on the L6470's
static volatile boolean L6470_load_move = false; // Flag to stop the step/dir counters going while waiting on the L6470's

static volatile boolean x_busy = false, y_busy = false, z_busy = false, e_busy = false;

static volatile boolean steppers_enabled = false;

static unsigned char out_bits; // The next stepping-bits to be output
static long counter_x, // Counter variables for the bresenham line tracer
		counter_y, counter_z, counter_e;
volatile static unsigned long step_events_completed; // The number of step events executed in the current block
#ifdef ADVANCE
static long advance_rate, advance, final_advance = 0;
static long old_advance = 0;
#endif
static long e_steps[3];
static long acceleration_time, deceleration_time;
//static unsigned long accelerate_until, decelerate_after, acceleration_rate, initial_rate, final_rate, nominal_rate;
static unsigned short acc_step_rate; // needed for deccelaration start point
static char step_loops;
#ifdef TIMER_BASED_STEPPER
static unsigned short OCR1A_nominal;
#endif

volatile long endstops_trigsteps[3] = { 0, 0, 0 };
volatile long endstops_stepsTotal, endstops_stepsDone;
static volatile bool endstop_x_hit = false;
static volatile bool endstop_y_hit = false;
static volatile bool endstop_z_hit = false;

static bool old_x_min_endstop = false;
static bool old_x_max_endstop = false;
static bool old_y_min_endstop = false;
static bool old_y_max_endstop = false;
static bool old_z_min_endstop = false;
static bool old_z_max_endstop = false;

static volatile bool check_endstops = true;

volatile long count_position[NUM_AXIS] = { 0, 0, 0, 0 };
volatile char count_direction[NUM_AXIS] = { 1, 1, 1, 1 };

//===========================================================================
//=============================functions         ============================
//===========================================================================

#define CHECK_ENDSTOPS  if(check_endstops)
#define DISABLED_CHECK_ENDSTOPS  if(false)

#ifdef TIMER_BASED_STEPPER

// intRes = intIn1 * intIn2 >> 16
// uses:
// r26 to store 0
// r27 to store the byte 1 of the 24 bit result
#define MultiU16X8toH16(intRes, charIn1, intIn2) \
asm volatile ( \
"clr r26 \n\t" \
"mul %A1, %B2 \n\t" \
"movw %A0, r0 \n\t" \
"mul %A1, %A2 \n\t" \
"add %A0, r1 \n\t" \
"adc %B0, r26 \n\t" \
"lsr r0 \n\t" \
"adc %A0, r26 \n\t" \
"adc %B0, r26 \n\t" \
"clr r1 \n\t" \
: \
"=&r" (intRes) \
: \
"d" (charIn1), \
"d" (intIn2) \
: \
"r26" \
)

// intRes = longIn1 * longIn2 >> 24
// uses:
// r26 to store 0
// r27 to store the byte 1 of the 48bit result
#define MultiU24X24toH16(intRes, longIn1, longIn2) \
asm volatile ( \
"clr r26 \n\t" \
"mul %A1, %B2 \n\t" \
"mov r27, r1 \n\t" \
"mul %B1, %C2 \n\t" \
"movw %A0, r0 \n\t" \
"mul %C1, %C2 \n\t" \
"add %B0, r0 \n\t" \
"mul %C1, %B2 \n\t" \
"add %A0, r0 \n\t" \
"adc %B0, r1 \n\t" \
"mul %A1, %C2 \n\t" \
"add r27, r0 \n\t" \
"adc %A0, r1 \n\t" \
"adc %B0, r26 \n\t" \
"mul %B1, %B2 \n\t" \
"add r27, r0 \n\t" \
"adc %A0, r1 \n\t" \
"adc %B0, r26 \n\t" \
"mul %C1, %A2 \n\t" \
"add r27, r0 \n\t" \
"adc %A0, r1 \n\t" \
"adc %B0, r26 \n\t" \
"mul %B1, %A2 \n\t" \
"add r27, r1 \n\t" \
"adc %A0, r26 \n\t" \
"adc %B0, r26 \n\t" \
"lsr r27 \n\t" \
"adc %A0, r26 \n\t" \
"adc %B0, r26 \n\t" \
"clr r1 \n\t" \
: \
"=&r" (intRes) \
: \
"d" (longIn1), \
"d" (longIn2) \
: \
"r26" , "r27" \
)
#endif

// Some useful constants

//#define ENABLE_STEPPER_DRIVER_INTERRUPT()  TIMSK1 |= (1<<OCIE1A)
//#define DISABLE_STEPPER_DRIVER_INTERRUPT() TIMSK1 &= ~(1<<OCIE1A)
#define ENABLE_STEPPER_DRIVER_INTERRUPT()  steppers_enabled = true;
#define DISABLE_STEPPER_DRIVER_INTERRUPT() {steppers_enabled = false; L6470_load_move = false; L6470_complete = true; x_busy = false; y_busy = false; z_busy = false; e_busy = false; }

void checkHitEndstops() {
	if (endstop_x_hit || endstop_y_hit || endstop_z_hit) {
		SERIAL_ECHO_START;
		SERIAL_ECHOPGM(MSG_ENDSTOPS_HIT);
		if (endstop_x_hit) {
			SERIAL_ECHOPAIR(" X:",
					(float)endstops_trigsteps[X_AXIS]/axis_steps_per_unit[X_AXIS]);
		}
		if (endstop_y_hit) {
			SERIAL_ECHOPAIR(" Y:",
					(float)endstops_trigsteps[Y_AXIS]/axis_steps_per_unit[Y_AXIS]);
		}
		if (endstop_z_hit) {
			SERIAL_ECHOPAIR(" Z:",
					(float)endstops_trigsteps[Z_AXIS]/axis_steps_per_unit[Z_AXIS]);
		}
	}
	SERIAL_ECHOLN("");
	endstop_x_hit = false;
	endstop_y_hit = false;
	endstop_z_hit = false;
}

void endstops_hit_on_purpose() {
	endstop_x_hit = false;
	endstop_y_hit = false;
	endstop_z_hit = false;
}

void enable_endstops(bool check) {
	check_endstops = check;
}

//         __________________________
//        /|                        |\     _________________         ^
//       / |                        | \   /|               |\        |
//      /  |                        |  \ / |               | \       s
//     /   |                        |   |  |               |  \      p
//    /    |                        |   |  |               |   \     e
//   +-----+------------------------+---+--+---------------+----+    e
//   |               BLOCK 1            |      BLOCK 2          |    d
//
//                           time ----->
// 
//  The trapezoid is the shape the speed curve over time. It starts at block->initial_rate, accelerates 
//  first block->accelerate_until step_events_completed, then keeps going at constant speed until 
//  step_events_completed reaches block->decelerate_after after which it decelerates until the trapezoid generator is reset.
//  The slope of acceleration is calculated with the leib ramp alghorithm.

void st_wake_up() {
	//  TCNT1 = 0;
	ENABLE_STEPPER_DRIVER_INTERRUPT();
}

void step_wait() {
	for (int8_t i = 0; i < 6; i++) {
	}
}

#ifdef TIMER_BASED_STEPPER

FORCE_INLINE unsigned short calc_timer(unsigned short step_rate) {
	unsigned short timer;
	if(step_rate > MAX_STEP_FREQUENCY) step_rate = MAX_STEP_FREQUENCY;

	if(step_rate > 20000) { // If steprate > 20kHz >> step 4 times
		step_rate = (step_rate >> 2)&0x3fff;
		step_loops = 4;
	}
	else if(step_rate > 10000) { // If steprate > 10kHz >> step 2 times
		step_rate = (step_rate >> 1)&0x7fff;
		step_loops = 2;
	}
	else {
		step_loops = 1;
	}

	if(step_rate < (F_CPU/500000)) step_rate = (F_CPU/500000);
	step_rate -= (F_CPU/500000); // Correct for minimal speed
	if(step_rate >= (8*256)) { // higher step rate
		unsigned short table_address = (unsigned short)&speed_lookuptable_fast[(unsigned char)(step_rate>>8)][0];
		unsigned char tmp_step_rate = (step_rate & 0x00ff);
		unsigned short gain = (unsigned short)pgm_read_word_near(table_address+2);
		MultiU16X8toH16(timer, tmp_step_rate, gain);
		timer = (unsigned short)pgm_read_word_near(table_address) - timer;
	}
	else { // lower step rates
		unsigned short table_address = (unsigned short)&speed_lookuptable_slow[0][0];
		table_address += ((step_rate)>>1) & 0xfffc;
		timer = (unsigned short)pgm_read_word_near(table_address);
		timer -= (((unsigned short)pgm_read_word_near(table_address+2) * (unsigned char)(step_rate & 0x0007))>>3);
	}
	if(timer < 100) {timer = 100; MYSERIAL.print(MSG_STEPPER_TO_HIGH); MYSERIAL.println(step_rate);} //(20kHz this should never happen)
	return timer;
}
#endif

#ifdef TIMER_BASED_STEPPER

// Initializes the trapezoid generator from the current block. Called whenever a new 
// block begins.
FORCE_INLINE void trapezoid_generator_reset() {
#ifdef ADVANCE
	advance = current_block->initial_advance;
	final_advance = current_block->final_advance;
	// Do E steps + advance steps
	e_steps[current_block->active_extruder] += ((advance >>8) - old_advance);
	old_advance = advance >>8;
#endif
	deceleration_time = 0;
	// step_rate to timer interval
	OCR1A_nominal = calc_timer(current_block->nominal_rate);
	acc_step_rate = current_block->initial_rate;
	acceleration_time = calc_timer(acc_step_rate);
	OCR1A = acceleration_time;

#ifdef STEPPER_DEBUG  
//    SERIAL_ECHO_START;
//    SERIAL_ECHOPGM("advance :");
//    SERIAL_ECHO(current_block->advance/256.0);
//    SERIAL_ECHOPGM("advance rate :");
//    SERIAL_ECHO(current_block->advance_rate/256.0);
//    SERIAL_ECHOPGM("initial advance :");
//  SERIAL_ECHO(current_block->initial_advance/256.0);
//    SERIAL_ECHOPGM("final advance :");
//    SERIAL_ECHOLN(current_block->final_advance/256.0);
#endif

}

#endif 

#ifdef TIMER_BASED_STEPPER

// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse.  
// It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately. 
ISR(TIMER1_COMPA_vect)
{
	block_initial_step = false;

	// If there is no current block, attempt to pop one from the buffer
	if (current_block == NULL) {
		// Anything in the buffer?
		current_block = plan_get_current_block();
		if (current_block != NULL) {
			block_initial_step = true;
			current_block->busy = true;
			trapezoid_generator_reset();
			counter_x = -(current_block->step_event_count >> 1);
			counter_y = counter_x;
			counter_z = counter_x;
			counter_e = counter_x;
			step_events_completed = 0;

#ifdef Z_LATE_ENABLE
			if(current_block->steps_z > 0) {
				enable_z();
				OCR1A = 2000; //1ms wait
				return;
			}
#endif

//      #ifdef ADVANCE
//      e_steps[current_block->active_extruder] = 0;
//      #endif
		}
		else {
			OCR1A=2000; // 1kHz.
		}
	}

	if (current_block != NULL) {
		// Set directions TO DO This should be done once during init of trapezoid. Endstops -> interrupt
		out_bits = current_block->direction_bits;

		// Set direction en check limit switches
		if ((out_bits & (1<<X_AXIS)) != 0) { // stepping along -X axis
#if !defined COREXY  //NOT COREXY
			WRITE_L6470(MOTOR_X, X_DIR_PIN, INVERT_X_DIR);
			L6470_x_dir = INVERT_X_DIR;
#endif
			count_direction[X_AXIS]=-1;
			DISABLED_CHECK_ENDSTOPS
			{
#if X_MIN_PIN > -1
				bool x_min_endstop=(READ(X_MIN_PIN) != X_ENDSTOPS_INVERTING);
				if(x_min_endstop && old_x_min_endstop && (current_block->steps_x > 0)) {
					endstops_trigsteps[X_AXIS] = count_position[X_AXIS];
					endstop_x_hit=true;
					step_events_completed = current_block->step_event_count;

					dSPIN_HardStop(MOTOR_X);
					dSPIN_ResetPos(MOTOR_X);
				}
				old_x_min_endstop = x_min_endstop;
#endif
			}
		}
		else { // +direction
#if !defined COREXY  //NOT COREXY
			WRITE_L6470(MOTOR_X, X_DIR_PIN,!INVERT_X_DIR);
			L6470_x_dir = !INVERT_X_DIR;
#endif

			count_direction[X_AXIS]=1;
			DISABLED_CHECK_ENDSTOPS
			{
#if X_MAX_PIN > -1
				bool x_max_endstop=(READ(X_MAX_PIN) != X_ENDSTOPS_INVERTING);
				if(x_max_endstop && old_x_max_endstop && (current_block->steps_x > 0)) {
					endstops_trigsteps[X_AXIS] = count_position[X_AXIS];
					endstop_x_hit=true;
					step_events_completed = current_block->step_event_count;

					dSPIN_HardStop(MOTOR_X);
				}
				old_x_max_endstop = x_max_endstop;
#endif
			}
		}

		if ((out_bits & (1<<Y_AXIS)) != 0) { // -direction
#if !defined COREXY  //NOT COREXY
			WRITE_L6470(MOTOR_Y, Y_DIR_PIN,INVERT_Y_DIR);
			L6470_y_dir = INVERT_Y_DIR;
#endif
			count_direction[Y_AXIS]=-1;
			DISABLED_CHECK_ENDSTOPS
			{
#if Y_MIN_PIN > -1
				bool y_min_endstop=(READ(Y_MIN_PIN) != Y_ENDSTOPS_INVERTING);
				if(y_min_endstop && old_y_min_endstop && (current_block->steps_y > 0)) {
					endstops_trigsteps[Y_AXIS] = count_position[Y_AXIS];
					endstop_y_hit=true;
					step_events_completed = current_block->step_event_count;

					dSPIN_HardStop(MOTOR_Y);
					dSPIN_ResetPos(MOTOR_Y);
				}
				old_y_min_endstop = y_min_endstop;
#endif
			}
		}
		else { // +direction
#if !defined COREXY  //NOT COREXY
			WRITE_L6470(MOTOR_Y, Y_DIR_PIN,!INVERT_Y_DIR);
			L6470_y_dir = !INVERT_Y_DIR;
#endif
			count_direction[Y_AXIS]=1;
			DISABLED_CHECK_ENDSTOPS
			{
#if Y_MAX_PIN > -1
				bool y_max_endstop=(READ(Y_MAX_PIN) != Y_ENDSTOPS_INVERTING);
				if(y_max_endstop && old_y_max_endstop && (current_block->steps_y > 0)) {
					endstops_trigsteps[Y_AXIS] = count_position[Y_AXIS];
					endstop_y_hit=true;

					dSPIN_HardStop(MOTOR_Y);

					step_events_completed = current_block->step_event_count;
				}
				old_y_max_endstop = y_max_endstop;
#endif
			}
		}

		/*
		 #ifdef COREXY  //coreXY kinematics defined
		 if((current_block->steps_x >= current_block->steps_y)&&((out_bits & (1<<X_AXIS)) == 0)){  //+X is major axis
		 WRITE_L6470(MOTOR_X, X_DIR_PIN, !INVERT_X_DIR);
		 WRITE_L6470(MOTOR_Y, Y_DIR_PIN, !INVERT_Y_DIR);
		 }
		 if((current_block->steps_x >= current_block->steps_y)&&((out_bits & (1<<X_AXIS)) != 0)){  //-X is major axis
		 WRITE_L6470(MOTOR_X, X_DIR_PIN, INVERT_X_DIR);
		 WRITE_L6470(MOTOR_Y, Y_DIR_PIN, INVERT_Y_DIR);
		 }
		 if((current_block->steps_y > current_block->steps_x)&&((out_bits & (1<<Y_AXIS)) == 0)){  //+Y is major axis
		 WRITE_L6470(MOTOR_X, X_DIR_PIN, !INVERT_X_DIR);
		 WRITE_L6470(MOTOR_Y, Y_DIR_PIN, INVERT_Y_DIR);
		 }
		 if((current_block->steps_y > current_block->steps_x)&&((out_bits & (1<<Y_AXIS)) != 0)){  //-Y is major axis
		 WRITE_L6470(MOTOR_X, X_DIR_PIN, INVERT_X_DIR);
		 WRITE_L6470(MOTOR_Y, Y_DIR_PIN, !INVERT_Y_DIR);
		 }
		 #endif //coreXY
		 */

		if ((out_bits & (1<<Z_AXIS)) != 0) { // -direction
			WRITE_L6470(MOTOR_Z, Z_DIR_PIN,INVERT_Z_DIR);
			L6470_z_dir = INVERT_Z_DIR;

#ifdef Z_DUAL_STEPPER_DRIVERS
			WRITE_L6470(MOTOR_Z2, Z2_DIR_PIN,INVERT_Z_DIR);
			L6470_z2_dir = INVERT_Z2_DIR;
#endif

			count_direction[Z_AXIS]=-1;
			DISABLED_CHECK_ENDSTOPS
			{
#if Z_MIN_PIN > -1
				bool z_min_endstop=(READ(Z_MIN_PIN) != Z_ENDSTOPS_INVERTING);
				if(z_min_endstop && old_z_min_endstop && (current_block->steps_z > 0)) {
					endstops_trigsteps[Z_AXIS] = count_position[Z_AXIS];
					endstop_z_hit=true;
					step_events_completed = current_block->step_event_count;

					dSPIN_HardStop(MOTOR_Z);
					dSPIN_ResetPos(MOTOR_Z);
				}
				old_z_min_endstop = z_min_endstop;
#endif
			}
		}
		else { // +direction
			WRITE_L6470(MOTOR_Z, Z_DIR_PIN,!INVERT_Z_DIR);
			L6470_z_dir = !INVERT_Z_DIR;

#ifdef Z_DUAL_STEPPER_DRIVERS
			WRITE_L6470(MOTOR_Z2, Z2_DIR_PIN,!INVERT_Z_DIR);
			L6470_z2_dir = !INVERT_Z2_DIR;
#endif

			count_direction[Z_AXIS]=1;
			DISABLED_CHECK_ENDSTOPS
			{
#if Z_MAX_PIN > -1
				bool z_max_endstop=(READ(Z_MAX_PIN) != Z_ENDSTOPS_INVERTING);
				if(z_max_endstop && old_z_max_endstop && (current_block->steps_z > 0)) {
					endstops_trigsteps[Z_AXIS] = count_position[Z_AXIS];
					endstop_z_hit=true;
					step_events_completed = current_block->step_event_count;

					dSPIN_HardStop(MOTOR_Z);
				}
				old_z_max_endstop = z_max_endstop;
#endif
			}
		}

#ifndef ADVANCE
		if ((out_bits & (1<<E_AXIS)) != 0) { // -direction
			REV_E_DIR();
			/*
			 if(current_block->active_extruder == 1) {
			 WRITE(E1_DIR_PIN, INVERT_E1_DIR);
			 } else {
			 WRITE(E0_DIR_PIN, INVERT_E0_DIR);
			 }*/
			count_direction[E_AXIS]=-1;
			L6470_e_dir = INVERT_E0_DIR;
		}
		else { // +direction
			NORM_E_DIR();
			count_direction[E_AXIS]=1;
			L6470_e_dir = !INVERT_E0_DIR;
		}
#endif //!ADVANCE

		if(block_initial_step) { // Move L6470 once per block
			// Request the L6470 manager to load the initial move.
			L6470_load_move = true;
			L6470_complete = false;

			/*x_busy = true;
			 y_busy = true;
			 z_busy = true;
			 e_busy = true;*/

		}

		for(int8_t i=0; i < step_loops; i++) { // Take multiple steps per interrupt (For high speed moves)
#if MOTHERBOARD != 8 // !teensylu
			MSerial.checkRx();// Check for serial chars.
#endif

#ifdef ADVANCE
			counter_e += current_block->steps_e;
			if (counter_e > 0) {
				counter_e -= current_block->step_event_count;
				if ((out_bits & (1<<E_AXIS)) != 0) { // - direction
					e_steps[current_block->active_extruder]--;
				}
				else {
					e_steps[current_block->active_extruder]++;
				}
			}
#endif //ADVANCE
#if !defined COREXY
			counter_x += current_block->steps_x;
			if (counter_x > 0) {
				WRITE_L6470_STEP(MOTOR_X, X_STEP_PIN, !INVERT_X_STEP_PIN);
				counter_x -= current_block->step_event_count;
				count_position[X_AXIS]+=count_direction[X_AXIS];
				WRITE_L6470_STEP(MOTOR_X, X_STEP_PIN, INVERT_X_STEP_PIN);
			}

			counter_y += current_block->steps_y;
			if (counter_y > 0) {
				WRITE_L6470_STEP(MOTOR_Y, Y_STEP_PIN, !INVERT_Y_STEP_PIN);
				counter_y -= current_block->step_event_count;
				count_position[Y_AXIS]+=count_direction[Y_AXIS];
				WRITE_L6470_STEP(MOTOR_Y, Y_STEP_PIN, INVERT_Y_STEP_PIN);
			}
#endif

			/*
			 #ifdef COREXY
			 counter_x += current_block->steps_x;
			 counter_y += current_block->steps_y;

			 if ((counter_x > 0)&&!(counter_y>0)){  //X step only
			 WRITE_L6470_STEP(MOTOR_X, X_STEP_PIN, !INVERT_X_STEP_PIN);
			 WRITE_L6470_STEP(MOTOR_Y, Y_STEP_PIN, !INVERT_Y_STEP_PIN);
			 counter_x -= current_block->step_event_count;
			 count_position[X_AXIS]+=count_direction[X_AXIS];
			 WRITE_L6470_STEP(MOTOR_X, X_STEP_PIN, INVERT_X_STEP_PIN);
			 WRITE_L6470_STEP(MOTOR_Y, Y_STEP_PIN, INVERT_Y_STEP_PIN);
			 }

			 if (!(counter_x > 0)&&(counter_y>0)){  //Y step only
			 WRITE_L6470_STEP(MOTOR_X, X_STEP_PIN, !INVERT_X_STEP_PIN);
			 WRITE_L6470_STEP(MOTOR_Y, Y_STEP_PIN, !INVERT_Y_STEP_PIN);
			 counter_y -= current_block->step_event_count;
			 count_position[Y_AXIS]+=count_direction[Y_AXIS];
			 WRITE_L6470_STEP(MOTOR_X, X_STEP_PIN, INVERT_X_STEP_PIN);
			 WRITE_L6470_STEP(MOTOR_Y, Y_STEP_PIN, INVERT_Y_STEP_PIN);
			 }

			 if ((counter_x > 0)&&(counter_y>0)){  //step in both axes
			 if (((out_bits & (1<<X_AXIS)) == 0)^((out_bits & (1<<Y_AXIS)) == 0)){  //X and Y in different directions
			 WRITE_L6470_STEP(MOTOR_Y, Y_STEP_PIN, !INVERT_Y_STEP_PIN);
			 counter_x -= current_block->step_event_count;
			 WRITE_L6470_STEP(MOTOR_Y, Y_STEP_PIN, INVERT_Y_STEP_PIN);
			 step_wait();
			 count_position[X_AXIS]+=count_direction[X_AXIS];
			 count_position[Y_AXIS]+=count_direction[Y_AXIS];
			 WRITE_L6470_STEP(MOTOR_Y, Y_STEP_PIN, !INVERT_Y_STEP_PIN);
			 counter_y -= current_block->step_event_count;
			 WRITE_L6470_STEP(MOTOR_Y, Y_STEP_PIN, INVERT_Y_STEP_PIN);
			 }
			 else{  //X and Y in same direction
			 WRITE_L6470_STEP(MOTOR_X, X_STEP_PIN, !INVERT_X_STEP_PIN);
			 counter_x -= current_block->step_event_count;
			 WRITE_L6470_STEP(MOTOR_X, X_STEP_PIN, INVERT_X_STEP_PIN) ;
			 step_wait();
			 count_position[X_AXIS]+=count_direction[X_AXIS];
			 count_position[Y_AXIS]+=count_direction[Y_AXIS];
			 WRITE_L6470_STEP(MOTOR_X, X_STEP_PIN, !INVERT_X_STEP_PIN);
			 counter_y -= current_block->step_event_count;
			 WRITE_L6470_STEP(MOTOR_X, X_STEP_PIN, INVERT_X_STEP_PIN);
			 }
			 }
			 #endif //corexy
			 */

			counter_z += current_block->steps_z;
			if (counter_z > 0) {
				WRITE_L6470_STEP(MOTOR_Z, Z_STEP_PIN, !INVERT_Z_STEP_PIN);

#ifdef Z_DUAL_STEPPER_DRIVERS
				WRITE_L6470_STEP(MOTOR_Z2, Z2_STEP_PIN, !INVERT_Z_STEP_PIN);
#endif

				counter_z -= current_block->step_event_count;
				count_position[Z_AXIS]+=count_direction[Z_AXIS];
				WRITE_L6470_STEP(MOTOR_Z, Z_STEP_PIN, INVERT_Z_STEP_PIN);

#ifdef Z_DUAL_STEPPER_DRIVERS
				WRITE_L6470_STEP(MOTOR_Z2, Z2_STEP_PIN, INVERT_Z_STEP_PIN);
#endif
			}

#ifndef ADVANCE
			counter_e += current_block->steps_e;
			if (counter_e > 0) {
				//WRITE_E_STEP(!INVERT_E_STEP_PIN);
				WRITE_L6470_STEP(MOTOR_E, E0_STEP_PIN, !INVERT_E_STEP_PIN);
				counter_e -= current_block->step_event_count;
				count_position[E_AXIS]+=count_direction[E_AXIS];
				//WRITE_E_STEP(INVERT_E_STEP_PIN);
				WRITE_L6470_STEP(MOTOR_E, E0_STEP_PIN, INVERT_E_STEP_PIN);
			}
#endif //!ADVANCE

			step_events_completed += 1;
			if(step_events_completed >= current_block->step_event_count) break;
		}

		if(L6470_complete) {
			step_events_completed = current_block->step_event_count;
		}

		// Calculare new timer value
		unsigned short timer;
		unsigned short step_rate;
		if (step_events_completed <= (unsigned long int)current_block->accelerate_until) {

			MultiU24X24toH16(acc_step_rate, acceleration_time, current_block->acceleration_rate);
			acc_step_rate += current_block->initial_rate;

			// upper limit
			if(acc_step_rate > current_block->nominal_rate)
			acc_step_rate = current_block->nominal_rate;

			// step_rate to timer interval
			timer = calc_timer(acc_step_rate);
			OCR1A = timer;
			acceleration_time += timer;
#ifdef ADVANCE
			for(int8_t i=0; i < step_loops; i++) {
				advance += advance_rate;
			}
			//if(advance > current_block->advance) advance = current_block->advance;
			// Do E steps + advance steps
			e_steps[current_block->active_extruder] += ((advance >>8) - old_advance);
			old_advance = advance >>8;

#endif
		}
		else if (step_events_completed > (unsigned long int)current_block->decelerate_after) {
			MultiU24X24toH16(step_rate, deceleration_time, current_block->acceleration_rate);

			if(step_rate > acc_step_rate) { // Check step_rate stays positive
				step_rate = current_block->final_rate;
			}
			else {
				step_rate = acc_step_rate - step_rate; // Decelerate from aceleration end point.
			}

			// lower limit
			if(step_rate < current_block->final_rate)
			step_rate = current_block->final_rate;

			// step_rate to timer interval
			timer = calc_timer(step_rate);
			OCR1A = timer;
			deceleration_time += timer;
#ifdef ADVANCE
			for(int8_t i=0; i < step_loops; i++) {
				advance -= advance_rate;
			}
			if(advance < final_advance) advance = final_advance;
			// Do E steps + advance steps
			e_steps[current_block->active_extruder] += ((advance >>8) - old_advance);
			old_advance = advance >>8;
#endif //ADVANCE
		}
		else {
			OCR1A = OCR1A_nominal;
		}

		if ((step_events_completed >= current_block->step_event_count) && L6470_complete) {
			current_block = NULL;
			plan_discard_current_block();
		}

	}

}
#endif

#ifdef ADVANCE
unsigned char old_OCR0A;
// Timer interrupt for E. e_steps is set in the main routine;
// Timer 0 is shared with millies
ISR(TIMER0_COMPA_vect)
{
	old_OCR0A += 52; // ~10kHz interrupt (250000 / 26 = 9615kHz)
	OCR0A = old_OCR0A;
	// Set E direction (Depends on E direction + advance)
	for(unsigned char i=0; i<4;i++) {
		if (e_steps[0] != 0) {
			WRITE(E0_STEP_PIN, INVERT_E_STEP_PIN);
			if (e_steps[0] < 0) {
				WRITE(E0_DIR_PIN, INVERT_E0_DIR);
				e_steps[0]++;
				WRITE(E0_STEP_PIN, !INVERT_E_STEP_PIN);
			}
			else if (e_steps[0] > 0) {
				WRITE(E0_DIR_PIN, !INVERT_E0_DIR);
				e_steps[0]--;
				WRITE(E0_STEP_PIN, !INVERT_E_STEP_PIN);
			}
		}
#if EXTRUDERS > 1
		if (e_steps[1] != 0) {
			WRITE(E1_STEP_PIN, INVERT_E_STEP_PIN);
			if (e_steps[1] < 0) {
				WRITE(E1_DIR_PIN, INVERT_E1_DIR);
				e_steps[1]++;
				WRITE(E1_STEP_PIN, !INVERT_E_STEP_PIN);
			}
			else if (e_steps[1] > 0) {
				WRITE(E1_DIR_PIN, !INVERT_E1_DIR);
				e_steps[1]--;
				WRITE(E1_STEP_PIN, !INVERT_E_STEP_PIN);
			}
		}
#endif
#if EXTRUDERS > 2
		if (e_steps[2] != 0) {
			WRITE(E2_STEP_PIN, INVERT_E_STEP_PIN);
			if (e_steps[2] < 0) {
				WRITE(E2_DIR_PIN, INVERT_E2_DIR);
				e_steps[2]++;
				WRITE(E2_STEP_PIN, !INVERT_E_STEP_PIN);
			}
			else if (e_steps[2] > 0) {
				WRITE(E2_DIR_PIN, !INVERT_E2_DIR);
				e_steps[2]--;
				WRITE(E2_STEP_PIN, !INVERT_E_STEP_PIN);
			}
		}
#endif
	}
}
#endif // ADVANCE
void st_init() {
	//Initialize Dir Pins

#ifdef TIMER_BASED_STEPPER

#if X_DIR_PIN > -1
	SET_OUTPUT(X_DIR_PIN);
#endif
#if Y_DIR_PIN > -1
	SET_OUTPUT(Y_DIR_PIN);
#endif
#if Z_DIR_PIN > -1
	SET_OUTPUT(Z_DIR_PIN);

#if defined(Z_DUAL_STEPPER_DRIVERS) && (Z2_DIR_PIN > -1)
	SET_OUTPUT(Z2_DIR_PIN);
#endif
#endif
#if E0_DIR_PIN > -1
	SET_OUTPUT(E0_DIR_PIN);
#endif
#if defined(E1_DIR_PIN) && (E1_DIR_PIN > -1)
	SET_OUTPUT(E1_DIR_PIN);
#endif
#if defined(E2_DIR_PIN) && (E2_DIR_PIN > -1)
	SET_OUTPUT(E2_DIR_PIN);
#endif

#endif

	//Initialize Enable Pins - steppers default to disabled.

#if (X_ENABLE_PIN > -1)
	SET_OUTPUT(X_ENABLE_PIN);
	if (!X_ENABLE_ON)
		WRITE(X_ENABLE_PIN, HIGH);

	//if(!X_ENABLE_ON) { WRITE(X_ENABLE_PIN, HIGH); delay(1000); WRITE(X_ENABLE_PIN, LOW); delay(1000); WRITE(X_ENABLE_PIN, HIGH);}
	//else {WRITE(X_ENABLE_PIN, LOW); delay(1); WRITE(X_ENABLE_PIN, HIGH); delay(1); WRITE(X_ENABLE_PIN, LOW);}
#endif
#if (Y_ENABLE_PIN > -1)
	SET_OUTPUT(Y_ENABLE_PIN);
	if (!Y_ENABLE_ON)
		WRITE(Y_ENABLE_PIN, HIGH);
#endif
#if (Z_ENABLE_PIN > -1)
	SET_OUTPUT(Z_ENABLE_PIN);
	if (!Z_ENABLE_ON)
		WRITE(Z_ENABLE_PIN, HIGH);

#if defined(Z_DUAL_STEPPER_DRIVERS) && (Z2_ENABLE_PIN > -1)
	SET_OUTPUT(Z2_ENABLE_PIN);
	if(!Z_ENABLE_ON) WRITE(Z2_ENABLE_PIN,HIGH);
#endif
#endif
#if (E0_ENABLE_PIN > -1)
	SET_OUTPUT(E0_ENABLE_PIN);
	if (!E_ENABLE_ON)
		WRITE(E0_ENABLE_PIN, HIGH);
#endif
#if defined(E1_ENABLE_PIN) && (E1_ENABLE_PIN > -1)
	SET_OUTPUT(E1_ENABLE_PIN);
	if(!E_ENABLE_ON) WRITE(E1_ENABLE_PIN,HIGH);
#endif
#if defined(E2_ENABLE_PIN) && (E2_ENABLE_PIN > -1)
	SET_OUTPUT(E2_ENABLE_PIN);
	if(!E_ENABLE_ON) WRITE(E2_ENABLE_PIN,HIGH);
#endif

	//endstops and pullups

#if X_MIN_PIN > -1
	//SET_INPUT(X_MIN_PIN);  Disabled as doing detection in L6470. Set as an output now so we can drive the existing code.
	SET_OUTPUT(X_MIN_PIN);
#ifdef ENDSTOPPULLUP_XMIN
	WRITE(X_MIN_PIN, HIGH);
#endif
#endif

#if Y_MIN_PIN > -1
	//SET_INPUT(Y_MIN_PIN); Disabled as doing detection in L6470. Set as an output now so we can drive the existing code.
	SET_OUTPUT(Y_MIN_PIN);
	//Added  as doing detection in L6470. Set as an output now so we can drive the existing code.

#ifdef ENDSTOPPULLUP_YMIN
	WRITE(Y_MIN_PIN, HIGH);
#endif
#endif

#if Z_MIN_PIN > -1
	//SET_INPUT(Z_MIN_PIN); Disabled as doing detection in L6470. Set as an output now so we can drive the existing code.
	SET_OUTPUT(Z_MIN_PIN);
#ifdef ENDSTOPPULLUP_ZMIN
	WRITE(Z_MIN_PIN, HIGH);
#endif
#endif

#if X_MAX_PIN > -1
	SET_INPUT(X_MAX_PIN);
#ifdef ENDSTOPPULLUP_XMAX
	WRITE(X_MAX_PIN,HIGH);
#endif
#endif

#if Y_MAX_PIN > -1
	SET_INPUT(Y_MAX_PIN);
#ifdef ENDSTOPPULLUP_YMAX
	WRITE(Y_MAX_PIN,HIGH);
#endif
#endif

#if Z_MAX_PIN > -1
	SET_INPUT(Z_MAX_PIN);
#ifdef ENDSTOPPULLUP_ZMAX
	WRITE(Z_MAX_PIN,HIGH);
#endif
#endif

#ifdef TIMER_BASED_STEPPER

	//Initialize Step Pins
#if (X_STEP_PIN > -1)
	SET_OUTPUT(X_STEP_PIN);
	WRITE(X_STEP_PIN,INVERT_X_STEP_PIN);
	if(!X_ENABLE_ON) WRITE(X_ENABLE_PIN,HIGH);
#endif
#if (Y_STEP_PIN > -1)
	SET_OUTPUT(Y_STEP_PIN);
	WRITE(Y_STEP_PIN,INVERT_Y_STEP_PIN);
	if(!Y_ENABLE_ON) WRITE(Y_ENABLE_PIN,HIGH);
#endif
#if (Z_STEP_PIN > -1)
	SET_OUTPUT(Z_STEP_PIN);
	WRITE(Z_STEP_PIN,INVERT_Z_STEP_PIN);
	if(!Z_ENABLE_ON) WRITE(Z_ENABLE_PIN,HIGH);

#if defined(Z_DUAL_STEPPER_DRIVERS) && (Z2_STEP_PIN > -1)
	SET_OUTPUT(Z2_STEP_PIN);
	WRITE(Z2_STEP_PIN,INVERT_Z_STEP_PIN);
	if(!Z_ENABLE_ON) WRITE(Z2_ENABLE_PIN,HIGH);
#endif
#endif
#if (E0_STEP_PIN > -1)
	SET_OUTPUT(E0_STEP_PIN);
	WRITE(E0_STEP_PIN,INVERT_E_STEP_PIN);
	if(!E_ENABLE_ON) WRITE(E0_ENABLE_PIN,HIGH);
#endif
#if defined(E1_STEP_PIN) && (E1_STEP_PIN > -1)
	SET_OUTPUT(E1_STEP_PIN);
	WRITE(E1_STEP_PIN,INVERT_E_STEP_PIN);
	if(!E_ENABLE_ON) WRITE(E1_ENABLE_PIN,HIGH);
#endif
#if defined(E2_STEP_PIN) && (E2_STEP_PIN > -1)
	SET_OUTPUT(E2_STEP_PIN);
	WRITE(E2_STEP_PIN,INVERT_E_STEP_PIN);
	if(!E_ENABLE_ON) WRITE(E2_ENABLE_PIN,HIGH);
#endif
#endif

#ifdef CONTROLLERFAN_PIN
	SET_OUTPUT(CONTROLLERFAN_PIN); //Set pin used for driver cooling fan
#endif

#ifdef TIMER_BASED_STEPPER

	// waveform generation = 0100 = CTC
	TCCR1B &= ~(1<<WGM13);
	TCCR1B |= (1<<WGM12);
	TCCR1A &= ~(1<<WGM11);
	TCCR1A &= ~(1<<WGM10);

	// output mode = 00 (disconnected)
	TCCR1A &= ~(3<<COM1A0);
	TCCR1A &= ~(3<<COM1B0);

	// Set the timer pre-scaler
	// Generally we use a divider of 8, resulting in a 2MHz timer
	// frequency on a 16MHz MCU. If you are going to change this, be
	// sure to regenerate speed_lookuptable.h with
	// create_speed_lookuptable.py
	TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (2<<CS10);

	OCR1A = 0x4000;
	TCNT1 = 0;
#endif  
	ENABLE_STEPPER_DRIVER_INTERRUPT();

#ifdef ADVANCE
#if defined(TCCR0A) && defined(WGM01)
	TCCR0A &= ~(1<<WGM01);
	TCCR0A &= ~(1<<WGM00);
#endif
	e_steps[0] = 0;
	e_steps[1] = 0;
	e_steps[2] = 0;
	TIMSK0 |= (1<<OCIE0A);
#endif //ADVANCE
	enable_endstops(true); // Start with endstops active. After homing they can be disabled
#ifdef TIMER_BASED_STEPPER  
			sei();
#endif  
}

// Block until all buffered steps are executed
void st_synchronize() {
	while (blocks_queued()) {
		manage_heater();
		manage_L6470();
		manage_inactivity();
		LCD_STATUS;
	}
}

void st_set_position(const long &x, const long &y, const long &z,
		const long &e) {
	CRITICAL_SECTION_START;
	count_position[X_AXIS] = x;
	count_position[Y_AXIS] = y;
	count_position[Z_AXIS] = z;
	count_position[E_AXIS] = e;
	CRITICAL_SECTION_END;
}

void st_set_e_position(const long &e) {
	CRITICAL_SECTION_START;
	count_position[E_AXIS] = e;
	CRITICAL_SECTION_END;
}

long st_get_position(uint8_t axis) // Get position in steps ( micro steps) needs to be converted back into mm
		{
#ifdef TIMER_BASED_STEPPER  
	long count_pos;
	CRITICAL_SECTION_START;
	count_pos = count_position[axis];
	CRITICAL_SECTION_END;
	return count_pos;
#endif
	// The abs pos is a 22 bit number in 2's complement format. This isn't being handled correctly here.
	unsigned long current_pos = dSPIN_GetParam(axis, dSPIN_ABS_POS);
	if (current_pos & 0x200000) { // Need to convert 22 bit 2s complement into a signed long. If this bit is set then the number is -ve and so set the higher bits to 1
		current_pos |= 0xFC0000;
	}
	return (long) current_pos + 0x100000;
}

void finishAndDisableSteppers() {
	st_synchronize();
	LCD_MESSAGEPGM(MSG_STEPPER_RELEASED);
	disable_x();
	disable_y();
	disable_z();
	disable_e0(); disable_e1(); disable_e2();
}

void quickStop() {
	DISABLE_STEPPER_DRIVER_INTERRUPT();
	for (int i = 0; i < MOTOR_COUNT; i++) {
		dSPIN_HardHiZ(i);
	}
	while (blocks_queued())
		plan_discard_current_block();
	current_block = NULL;
	ENABLE_STEPPER_DRIVER_INTERRUPT();
}

static unsigned int L6470_status[MOTOR_COUNT];
static unsigned int L6470_oldstatus[MOTOR_COUNT];
static unsigned long step_loss[MOTOR_COUNT];
static unsigned long step_loss_counter[MOTOR_COUNT];
static unsigned long thermal_warning_counter[MOTOR_COUNT];
static unsigned long thermal_shutdown_counter[MOTOR_COUNT];

/*
 Check the status of the L6470 devices via SPI and take appropriate actions
 */
void dSPIN_monitorStatus() {
	dSPIN_GetStatus_All(L6470_status); //    <-- Get the status for all of the motors in one call
	for (int device = 0; device < MOTOR_COUNT; device++) {

		if (L6470_status[device] != L6470_oldstatus[device]) { // Has status changed for this device? If nothing has changed then do nothing.
			L6470_oldstatus[device] = L6470_status[device];
			// Some status bits need action to be taken. These go here. Others are just for info and go in the below debug section

			// *** Endstop compatibility actions **** //
			// Force the normal endstop pins to match the L6470 endstops. These pins need to be set as outputs instead of inputs for this to work.
			// We do this to maintain compatibility with the existing code. The downside is that pins are wasted as nothing is actually connected to them
			if (device == MOTOR_X) {
				if (L6470_status[device] & dSPIN_STATUS_SW_F) { // Make the existing endstop code mirror the switch bit in the L6470 X axis
					WRITE(X_MIN_PIN, !X_ENDSTOPS_INVERTING);
					// Drive the existing code low, this PIN has been set to be an output now.
				} else {
					WRITE(X_MIN_PIN, X_ENDSTOPS_INVERTING);
				}
			}

			if (device == MOTOR_Y) {
				if (L6470_status[device] & dSPIN_STATUS_SW_F) { // Make the existing endstop code mirror the switch bit in the L6470 Y axis
					WRITE(Y_MIN_PIN, !Y_ENDSTOPS_INVERTING);
					// Drive the existing code low, this PIN has been set to be an output now.
				} else {
					WRITE(Y_MIN_PIN, Y_ENDSTOPS_INVERTING);
				}
			}

			if (device == MOTOR_Z) {
				if (L6470_status[device] & dSPIN_STATUS_SW_F) { // Make the existing endstop code mirror the switch bit in the L6470 Z axis
					WRITE(Z_MIN_PIN, !Z_ENDSTOPS_INVERTING);
					// Drive the existing code low, this PIN has been set to be an output now.
				} else {
					WRITE(Z_MIN_PIN, Z_ENDSTOPS_INVERTING);
				}
			}

			// *** Endstop actions **** //
			if (L6470_status[device] & dSPIN_STATUS_SW_EVN) { // A Switch event has occurred. Device position will be reset automatically.
				step_events_completed = current_block->step_event_count;

				if (device == MOTOR_X) {
					endstops_trigsteps[X_AXIS] = count_position[X_AXIS];
					endstop_x_hit = true;
				}

				if (device == MOTOR_Y) {
					endstops_trigsteps[Y_AXIS] = count_position[Y_AXIS];
					endstop_y_hit = true;
				}

				if (device == MOTOR_Z) {
					endstops_trigsteps[Z_AXIS] = count_position[Z_AXIS];
					endstop_z_hit = true;
				}

#ifdef STEPPER_DEBUG
				SERIAL_ECHO_START;
				SERIAL_ECHOPAIR("Device no: ", (unsigned long)device);
				SERIAL_ECHOLN("dSPIN_STATUS_SW_EVN ");
#endif
			} else { // If there is no event, then check to see if we are already in home and moving deeper
				if (L6470_status[device] & dSPIN_STATUS_SW_F) { // Endstop is on
#ifdef STEPPER_DEBUG
					SERIAL_ECHO_START;
					SERIAL_ECHOPAIR("Device no: ", (unsigned long)device);
					SERIAL_ECHOLN("dSPIN_STATUS_SW_F is active");
#endif

					if (!(L6470_status[device] & dSPIN_STATUS_DIR)) { //  and we are trying to Move in reverse deeper into the endstop (note that this currently assumes axis are not reversed!!)

						dSPIN_HardStop(device);
						dSPIN_ResetPos(device);

						SERIAL_ECHO_START;
						SERIAL_ECHOPAIR("Device no: ", (unsigned long)device);
						SERIAL_ECHOLN(
								"Reverse Movement stopped because dSPIN_STATUS_SW_F is active");
					}
				}
			}

			// *** Busy Status updates *** //
				if (device == MOTOR_X) {
					boolean x_busy_temp = dSPIN_IsBusy_Status(
							L6470_status[device]);
					x_busy = x_busy_temp;
#ifdef STEPPER_DEBUG_BUSY
					SERIAL_ECHOPAIR("X Busy: ", (unsigned long)x_busy);
#endif
				}

				if (device == MOTOR_Y) {
					boolean y_busy_temp = dSPIN_IsBusy_Status(
							L6470_status[device]);
					y_busy = y_busy_temp;
#ifdef STEPPER_DEBUG_BUSY
					SERIAL_ECHOPAIR("Y Busy: ", (unsigned long)y_busy);
#endif
				}

				if (device == MOTOR_Z) {
					boolean z_busy_temp = dSPIN_IsBusy_Status(
							L6470_status[device]);
					z_busy = z_busy_temp;
#ifdef STEPPER_DEBUG_BUSY
					SERIAL_ECHOPAIR("Z Busy: ", (unsigned long)z_busy);
#endif
				}

				if (device == MOTOR_E) {
					boolean e_busy_temp = dSPIN_IsBusy_Status(
							L6470_status[device]);
					e_busy = e_busy_temp;
#ifdef STEPPER_DEBUG_BUSY
					SERIAL_ECHOPAIR("E Busy: ", (unsigned long)e_busy);
#endif
				}

			// *** Thermal shutdown action **** //
			if (!L6470_status[device] & dSPIN_STATUS_TH_SD) {
				quickStop();

				SERIAL_ECHO_START;
				SERIAL_ECHOPAIR("Device no: ", (unsigned long)device);
				SERIAL_ECHOPAIR("STATUS_TH_SD: ",
						++thermal_shutdown_counter[device]);
				SERIAL_ECHOLN("");
			}

			// *** Overcurrent action **** //
			if (!L6470_status[device] & dSPIN_STATUS_OCD) {
				quickStop();

				SERIAL_ECHO_START;
				SERIAL_ECHOPAIR("Device no: ", (unsigned long)device);
				SERIAL_ECHOLN("dSPIN_STATUS_OCD ");
			}

			// *** Undervoltage action **** //
			if (!L6470_status[device] & dSPIN_STATUS_UVLO) {
				quickStop();

				SERIAL_ECHO_START;
				SERIAL_ECHOPAIR("Device no: ", (unsigned long)device);
				SERIAL_ECHOLN("dSPIN_STATUS_UVLO ");
			}

			// *** Notification only actions *** //
			if (!(L6470_status[device] & dSPIN_STATUS_TH_WRN)) {
				SERIAL_ECHOPAIR("STATUS_TH_WRN: ",
						++thermal_warning_counter[device]);
				SERIAL_ECHOLN("");
			}

			if (!(L6470_status[device] & dSPIN_STATUS_STEP_LOSS_A)
					|| !(L6470_status[device] & dSPIN_STATUS_STEP_LOSS_B)) {
#ifdef STEPPER_DEBUG_BUSY
				SERIAL_ECHO_START;
  			        SERIAL_ECHOPAIR("Device no: ", (unsigned long)device);
				SERIAL_ECHOPAIR("STEP_LOST ", ++step_loss_counter[device]);
				SERIAL_ECHOLN("");
#endif
			}

#ifdef STEPPER_DEBUG
			SERIAL_ECHO_START;
			SERIAL_ECHOPAIR("Device no: ", (unsigned long)device);
			SERIAL_ECHOLN("");
#endif
			if (L6470_status[device] & dSPIN_STATUS_HIZ) {
				SERIAL_ECHO_START;
				SERIAL_ECHOLN("STATUS_HIZ ");
                        }
			//if(!(L6470_status[device] & dSPIN_STATUS_BUSY) ) SERIAL_ECHOLN("STATUS_BUSY ");
			//if((L6470_status[device] & dSPIN_STATUS_BUSY) ) SERIAL_ECHOLN("STATUS_NOT BUSY ");
			//if(L6470_status[device] & dSPIN_STATUS_SW_EVN ) SERIAL_ECHOLN("STATUS_SW_EVN ");
			if (L6470_status[device] & dSPIN_STATUS_NOTPERF_CMD) {
				SERIAL_ECHO_START;
				SERIAL_ECHOLN("STATUS_NOTPERF_CMD ");
                        }
			if (L6470_status[device] & dSPIN_STATUS_WRONG_CMD) {
				SERIAL_ECHO_START;
				SERIAL_ECHOLN("STATUS_WRONG_CMD ");
                        }
			//if(!(L6470_status[device] & dSPIN_STATUS_UVLO )) SERIAL_ECHOLN("STATUS_UVLO ");
			//if(!(L6470_status[device] & dSPIN_STATUS_TH_WRN )) { SERIAL_ECHOPAIR("STATUS_TH_WRN: ", ++thermal_warning_counter[device]); SERIAL_ECHOLN("");}
			//if(!(L6470_status[device] & dSPIN_STATUS_TH_SD )) { SERIAL_ECHOPAIR("STATUS_TH_SD: ", ++thermal_shutdown_counter[device]); SERIAL_ECHOLN("");}
			//if(!(L6470_status[device] & dSPIN_STATUS_OCD )) SERIAL_ECHOLN("STATUS_OCD ");
			//if(!(L6470_status[device] & dSPIN_STATUS_STEP_LOSS_A ) || !(L6470_status[device] & dSPIN_STATUS_STEP_LOSS_B )) { SERIAL_ECHOPAIR("STEP_LOST ", ++step_loss_counter[device]); SERIAL_ECHOLN("");}
			if (L6470_status[device] & dSPIN_STATUS_SCK_MOD) {
				SERIAL_ECHO_START; 
				SERIAL_ECHOLN("STATUS_SCK_MOD ");
                        }
		}
	}
}

byte dir[MOTOR_COUNT];
unsigned long steps[MOTOR_COUNT];
unsigned long min_speed[MOTOR_COUNT];
unsigned long max_speed[MOTOR_COUNT];
unsigned long max_speed_run[MOTOR_COUNT];
unsigned long accel[MOTOR_COUNT];
unsigned long decel[MOTOR_COUNT];
unsigned long param_result[MOTOR_COUNT];

void manage_L6470() {
  int loop_counter = 1;
  do {
	if (steppers_enabled) {
		block_initial_step = false;

		// If there is no current block, attempt to pop one from the buffer
		if (current_block == NULL) {
			// Anything in the buffer?
			current_block = plan_get_current_block();
			if (current_block != NULL) {
				block_initial_step = true;
				current_block->busy = true;
			} 
		}

		if (current_block != NULL) {
			// Set directions TO DO This should be done once during init of trapezoid. Endstops -> interrupt
			out_bits = current_block->direction_bits;

			// Set direction en check limit switches
			if ((out_bits & (1 << X_AXIS)) != 0) { // stepping along -X axis
				WRITE_L6470(MOTOR_X, X_DIR_PIN, INVERT_X_DIR);
				L6470_x_dir = INVERT_X_DIR;
			} else { // +direction
				WRITE_L6470(MOTOR_X, X_DIR_PIN, !INVERT_X_DIR);
				L6470_x_dir = !INVERT_X_DIR;
			}

			if ((out_bits & (1 << Y_AXIS)) != 0) { // -direction
				WRITE_L6470(MOTOR_Y, Y_DIR_PIN, INVERT_Y_DIR);
				L6470_y_dir = INVERT_Y_DIR;
			} else { // +direction
				WRITE_L6470(MOTOR_Y, Y_DIR_PIN, !INVERT_Y_DIR);
				L6470_y_dir = !INVERT_Y_DIR;
			}
			if ((out_bits & (1 << Z_AXIS)) != 0) { // -direction
				WRITE_L6470(MOTOR_Z, Z_DIR_PIN, INVERT_Z_DIR);
				L6470_z_dir = INVERT_Z_DIR;

#ifdef Z_DUAL_STEPPER_DRIVERS
				WRITE_L6470(MOTOR_Z2, Z2_DIR_PIN,INVERT_Z_DIR);
				L6470_z2_dir = INVERT_Z2_DIR;
#endif

			} else { // +direction
				WRITE_L6470(MOTOR_Z, Z_DIR_PIN, !INVERT_Z_DIR);
				L6470_z_dir = !INVERT_Z_DIR;

#ifdef Z_DUAL_STEPPER_DRIVERS
				WRITE_L6470(MOTOR_Z2, Z2_DIR_PIN,!INVERT_Z_DIR);
				L6470_z2_dir = !INVERT_Z2_DIR;
#endif
			}

			if ((out_bits & (1 << E_AXIS)) != 0) { // -direction
				L6470_e_dir = INVERT_E0_DIR;
			} else { // +direction
				L6470_e_dir = !INVERT_E0_DIR;
			}

			if (block_initial_step) { // Move L6470 once per block
				// Request the L6470 manager to load the initial move.
				L6470_load_move = true;
				L6470_complete = false;
			}

			// *** Load the move *** //
			if (L6470_load_move) {
				for (int i = 0; i < MOTOR_COUNT; i++) {
					if (i == MOTOR_X) {
						steps[i] = current_block->steps_x;
						dir[i] = L6470_x_dir ? 0 : 1;
						min_speed[i] = MinSpdCalc(
								min(current_block->initial_rate_x, current_block->final_rate_x)/60);
                                                max_speed[i] = MaxSpdCalc(current_block->rate_x);                                                
                                                //max_speed_run[i] = MaxRunSpdCalc(current_block->rate_x);

						accel[i] = AccCalc(current_block->accel_x);
						decel[i] = DecCalc(current_block->accel_x);
					}
					if (i == MOTOR_Y) {
						steps[i] = current_block->steps_y;
						dir[i] = L6470_y_dir ? 0 : 1;
						min_speed[i] = MinSpdCalc(
								min(current_block->initial_rate_y, current_block->final_rate_y)/60);

                                                max_speed[i] = MaxSpdCalc(current_block->rate_y);
                                                //max_speed_run[i] = MaxRunSpdCalc(current_block->rate_y);

						accel[i] = AccCalc(current_block->accel_y);
						decel[i] = DecCalc(current_block->accel_y);
					}
					if (i == MOTOR_Z) {
						steps[i] = current_block->steps_z;
						dir[i] = L6470_z_dir ? 0 : 1;
						min_speed[i] = MinSpdCalc(
								min(current_block->initial_rate_z, current_block->final_rate_z)/60);
						max_speed[i] = MaxSpdCalc(current_block->rate_z);
						accel[i] = AccCalc(current_block->accel_z);
						decel[i] = DecCalc(current_block->accel_z);
					}
					if (i == MOTOR_E) {
                                                //float xyspeed_factor = sqrt(current_block->rate_x^2 + current_block->rate_y^2) / 10;
						steps[i] = current_block->steps_e;
						dir[i] = L6470_e_dir ? 0 : 1;
						min_speed[i] = MinSpdCalc(
								min(current_block->initial_rate_e, current_block->final_rate_e)/60);
      						max_speed[i] = MaxSpdCalc(current_block->rate_e);// Allow the extrude to run a bit faster if the move is long enough to hit top speed

						accel[i] = AccCalc(current_block->accel_e); // <-- Accel slightly faster to bias the extrusion towards the start of the movement. This helps avoid the extruder running too long which seems to occur at slower feed rates.
						decel[i] = DecCalc(current_block->accel_e); // <-- Decel slightly slower to bias the extrusion towards the start of the movement. 
					}
				}


				dSPIN_SetParam_All(dSPIN_MIN_SPEED, min_speed, param_result); //<-- This doesn't work atm
				dSPIN_SetParam_All(dSPIN_ACC, accel, param_result);
				dSPIN_SetParam_All(dSPIN_DEC, decel, param_result);
				dSPIN_SetParam_All(dSPIN_MAX_SPEED, max_speed, param_result);
				dSPIN_Move_All(dir, steps); // <-- This is what makes the motors move

				L6470_load_move = false;
				L6470_complete = false;

#ifdef EXTRUDER_DEBUG
				SERIAL_ECHO_START;
				SERIAL_ECHOLN("Extruder motion requested");
				SERIAL_ECHOPAIR("Speed: ", current_block->rate_e);
				SERIAL_ECHOLN("");
				SERIAL_ECHOPAIR("MinSpeed: ", min(current_block->initial_rate_y, current_block->final_rate_y));
				SERIAL_ECHOLN("");
				SERIAL_ECHOPAIR("Steps: ", (unsigned long)current_block->steps_e);
				SERIAL_ECHOLN("");
				SERIAL_ECHOPAIR("Accel: ", current_block->accel_e);
				SERIAL_ECHOLN("");
				//SERIAL_ECHOPAIR("Decel: ", current_block->de_e);
				//SERIAL_ECHOLN("");

				SERIAL_ECHOLN("Extruder motion calculated");
				SERIAL_ECHOPAIR("Speed: ", (current_block->rate_e * 110)/100);
				SERIAL_ECHOLN("");
				SERIAL_ECHOPAIR("Steps: ", (unsigned long)current_block->steps_e);
				SERIAL_ECHOLN("");
				SERIAL_ECHOPAIR("Accel: ", (current_block->accel_e * 125)/100 );
				SERIAL_ECHOLN("");
				SERIAL_ECHOPAIR("Decel: ", (current_block->accel_e * 95)/100 );
				SERIAL_ECHOLN("");

 // time = 

//                                SERIAL_ECHOPAIR("Time: ", (current_block->accel_e * 100)/95 );
//				SERIAL_ECHOLN("");


#endif

#ifdef STEPPER_DEBUG
				SERIAL_ECHO_START;
				SERIAL_ECHOLN("Writing to L6470s");
				SERIAL_ECHOPAIR(" Speed X: ", current_block->rate_x);
				SERIAL_ECHOLN("");
				SERIAL_ECHOPAIR(" MinSpeed X: ", (unsigned long)min_speed[MOTOR_X]);
				SERIAL_ECHOLN("");
				SERIAL_ECHOPAIR(" Y: ", current_block->rate_y);
				SERIAL_ECHOLN("");
				SERIAL_ECHOPAIR(" MinSpeed Y: ", min_speed[MOTOR_Y]);
				SERIAL_ECHOLN("");
				SERIAL_ECHOPAIR(" Z: ", current_block->rate_z);
				SERIAL_ECHOLN("");
				SERIAL_ECHOPAIR(" E: ", current_block->rate_e);
				SERIAL_ECHOLN("");
				SERIAL_ECHOPAIR(" MinSpeed E: ", min_speed[MOTOR_E]);
				SERIAL_ECHOLN("");
				SERIAL_ECHOPAIR(" Accel X: ", current_block->accel_x);
				SERIAL_ECHOLN("");
				SERIAL_ECHOPAIR(" Y: ", current_block->accel_y);
				SERIAL_ECHOLN("");
				SERIAL_ECHOPAIR(" Z: ", current_block->accel_z);
				SERIAL_ECHOLN("");
				SERIAL_ECHOPAIR(" E: ", current_block->accel_e);
				SERIAL_ECHOLN("");
				SERIAL_ECHOPAIR(" Steps X: ",
						(unsigned long) current_block->steps_x);
				SERIAL_ECHOLN("");
				SERIAL_ECHOPAIR(" Y: ", (unsigned long)current_block->steps_y);
				SERIAL_ECHOLN("");
				SERIAL_ECHOPAIR(" Z: ", (unsigned long)current_block->steps_z);
				SERIAL_ECHOLN("");
				SERIAL_ECHOPAIR(" E: ", (unsigned long)current_block->steps_e);
				SERIAL_ECHOLN("");
#endif
			} // if load_move

			if (!L6470_complete) { // Move not finished, so check to see if the motors are still running and update all status from the motors
				dSPIN_monitorStatus(); // Get status from devices and take action if needed - i.e. reset busy flags, endstops hit (if wired to L6470 instead of arduino), undervoltage overcurrent or thermal shutdown

				// For some reason the extruder runs on after everything else has finished. This just causes blobs, so make it stop and consider the move complete anyway.
				// We could potentially add a dynamic adjustment here - or find and fix the root cause. It seems to be speed dependant - slower feed rates makes the extruder overrun worse, faster and the extruder underruns.
				if (!x_busy && !y_busy && !z_busy) {
					if (e_busy
							&& ((current_block->steps_x > 0)
									|| (current_block->steps_y > 0)
									|| (current_block->steps_z > 0))) {
						dSPIN_HardStop(MOTOR_E);
#ifdef STEPPER_DEBUG
						SERIAL_ECHO_START;
						SERIAL_ECHOLN(
								"L6470 Extruder still busy after movement complete.");
#endif
					}
				}

				if (x_busy || y_busy || z_busy || e_busy) {
					// Do nothing while the movement motors are busy. Extruder is handled specially.
				} else { // Move is finished, reset the complete flag so another one can be loaded.
					L6470_complete = true;

#ifdef STEPPER_DEBUG
					SERIAL_ECHO_START;
					if (L6470_complete)
						SERIAL_ECHOLN("L6470 movement complete flag set")
					else
						SERIAL_ECHOLN("L6470 movement complete flag not set");
#endif
				} // if not busy
			} // if !complete

			// *** Are we finished *** //
			if (L6470_complete) {
				current_block = NULL;
				plan_discard_current_block();
			}
		} // if current_block != NULL
	} // if steppers_enabled
  } while(L6470_complete && (current_block == NULL) && (loop_counter-- >= 0)); // If the move has completed then immediately load a new one. Don't wait if there is nothing ready.
}
