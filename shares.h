//*************************************************************************************
/** @file shares.h
 *    This file contains extern declarations for queues and other inter-task data
 *    communication objects used in a ME405/507/FreeRTOS project. 
 *
 *  Revisions:
 *    @li 09-30-2012 JRR Original file was a one-file demonstration with two tasks
 *    @li 10-05-2012 JRR Split into multiple files, one for each task plus a main one
 *    @li 10-29-2012 JRR Reorganized with global queue and shared data references
 *    @li 01-04-2014 JRR Re-reorganized, allocating shares with new now
 * 	  @li 02-09-2015 JH, TT, CL Added shared variables to be used in multiple tasks 
 * 		  and ISRs
 *
 *  License:
 *		This file is copyright 2015 by JR Ridgely and released under the Lesser GNU 
 *		Public License, version 2. It intended for educational use only, but its use
 *		is not limited thereto. */
/*		THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 *		AND	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * 		IMPLIED 	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * 		ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 * 		LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUEN-
 * 		TIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
 * 		OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 * 		CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
 * 		OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 * 		OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
//*************************************************************************************

// This define prevents this .h file from being included multiple times in a .cpp file
#ifndef _SHARES_H_
#define _SHARES_H_

//-------------------------------------------------------------------------------------
// Externs:  In this section, we declare variables and functions that are used in all
// (or at least two) of the files in the data acquisition project. Each of these items
// will also be declared exactly once, without the keyword 'extern', in one .cpp file
// as well as being declared extern here. 

// This queue allows tasks to send characters to the user interface task for display.
extern TextQueue* p_print_ser_queue;

// This shared data item allows a value for motor 1 speed and direction to be shared between
// the task_user and task_motor. Task_user is the source of the data item, and task_motor
// is the sink that utilizes the data item.
extern TaskShare<int16_t>* p_share_1;

// This shared data item allows a value for motor 2 speed and direction to be shared between
// the task_user and task_motor. Task_user is the source of the data item, and task_motor
// is the sink that utilizes the data item.
extern TaskShare<int16_t>* p_share_2;

// This shared data item is used to set the motor mode: power, brake, freewheel for motors
// 1 and 2. Motor 1 uses integers 0 through 2 and motor 2 uses integers 3 through 5.
extern TaskShare<uint8_t>* p_mode;

// This shared data item is used to read the state of the encoder tick used for comparison 
// in the ISR.
extern TaskShare<uint8_t>* p_state;

// This shared data item is used to hold the state of the last encoder tick of encoder 1 used for 
// comparison in the ISR.
extern TaskShare<uint8_t>* p_state_old_1;

// This shared data item is used to hold the state of the last encoder tick of encoder 2 used for 
// comparison in the ISR.
extern TaskShare<uint8_t>* p_state_old_2;

// This shared data item is used to hold the count for current number of errors. This number 
// is incremented in the ISR when applicable and read using error_count method.
extern TaskShare<uint32_t>* p_error_cntr;

// This shared data item is used to hold the count for current position of encoder 1. This number 
// is incremented or decremented based on comparsion of current and old states in the ISR and 
// read using view_count method.
extern TaskShare<uint32_t>* p_encoder_cntr_1;

// This shared data item is used to hold the count for current position of encoder 2. This number 
// is incremented or decremented based on comparsion of current and old states in the ISR and 
// read using view_count method.
extern TaskShare<uint32_t>* p_encoder_cntr_2;

// This shared data is used to hold the location of channel A external interrupt pin.
extern TaskShare<uint8_t>* p_ext_pin_A;

// This shared data is used to hold the location of channel B external interrupt pin.
extern TaskShare<uint8_t>* p_ext_pin_B;

// This shared data is used to hold the location of channel C external interrupt pin.
extern TaskShare<uint8_t>* p_ext_pin_C;

// This shared data is used to hold the location of channel D external interrupt pin.
extern TaskShare<uint8_t>* p_ext_pin_D;

// This shared data item is used to hold the position sent to the control loop for motor 1
extern TaskShare<int16_t>* p_position_1;

// This shared data item is used to hold the position sent to the control loop for motor 2
extern TaskShare<int16_t>* p_position_2;

// This shared data item is used to signal task_trigger to pull the gun's trigger
extern TaskShare<bool>* fire_at_will;

// These are shared data items to read the IR sensors
extern TaskShare<uint16_t>* p_high_left;
extern TaskShare<uint16_t>* p_high_right;
extern TaskShare<uint16_t>* p_center;
extern TaskShare<uint16_t>* p_low_left;
extern TaskShare<uint16_t>* p_low_right;

// These shared data items are position done flags for the control loop
extern TaskShare<bool>* p_pos_done_1;
extern TaskShare<bool>* p_pos_done_2;
		

#endif // _SHARES_H_
