//*************************************************************************************
/** \file main.cpp
 *    This file contains the main() code for a program which runs the ME405 board for
 *    ME405 Final Project. This program uses an A/D converter to convert an analog signal
 *	  into an LED brightness via pulse width modulation. It also uses a motor driver to
 * 	  independently control separate motors connected to the VNH3P30 motor drivers
 * 	  of the ME 405 board.
 *    
 *  Revisions:
 *    \li 09-30-2012 JRR Original file was a one-file demonstration with two tasks
 *    \li 10-05-2012 JRR Split into multiple files, one for each task plus a main one
 *    \li 10-30-2012 JRR A hopefully somewhat stable version with global queue 
 *                       pointers and the new operator used for most memory allocation
 *    \li 11-04-2012 JRR FreeRTOS Swoop demo program changed to a sweet test suite
 *    \li 01-05-2012 JRR Program reconfigured as ME405 Lab 1 starting point
 *    \li 03-28-2014 JRR Pointers to shared variables and queues changed to references
 *    @li 01-04-2015 JRR Names of share & queue classes changed; allocated with new now
 * 	  @li 01-26-2015 CAL, TT, JH Updated comments and lab name
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


#include <stdlib.h>                         // Prototype declarations for I/O functions
#include <avr/io.h>                         // Port I/O for SFR's
#include <avr/wdt.h>                        // Watchdog timer header
#include <string.h>                         // Functions for C string handling

#include "FreeRTOS.h"                       // Primary header for FreeRTOS
#include "task.h"                           // Header for FreeRTOS task functions
#include "queue.h"                          // FreeRTOS inter-task communication queues
#include "croutine.h"                       // Header for co-routines and such

#include "rs232int.h"                       // ME405/507 library for serial comm.
#include "time_stamp.h"                     // Class to implement a microsecond timer
#include "taskbase.h"                       // Header of wrapper for FreeRTOS tasks
#include "textqueue.h"                      // Wrapper for FreeRTOS character queues
#include "taskqueue.h"                      // Header of wrapper for FreeRTOS queues
#include "taskshare.h"                      // Header for thread-safe shared data
#include "shares.h"                         // Global ('extern') queue declarations
#include "task_motor.h"       		        // Header for the data acquisition task
#include "task_user.h"                      // Header for user interface task
#include "encoder_driver.h"                 // Header for the encoder driver
#include "task_encoder.h"					// Header for the encoder task
#include "task_control.h"					// Header for the controller task
#include "task_sensor.h"						// Header for the scan task
#include "task_trigger.h"					// Header for the trigger task
#include "task_position.h"					// Header for the position task

// Declare the queues which are used by tasks to communicate with each other here. 
// Each queue must also be declared 'extern' in a header file which will be read 
// by every task that needs to use that queue. The format for all queues except 
// the serial text printing queue is 'frt_queue<type> name (size)', where 'type' 
// is the type of data in the queue and 'size' is the number of items (not neces-
// sarily bytes) which the queue can hold

/** This is a print queue, descended from \c emstream so that things can be printed 
 *  into the queue using the "<<" operator and they'll come out the other end as a 
 *  stream of characters. It's used by tasks that send things to the user interface 
 *  task to be printed. 
 */
TextQueue* p_print_ser_queue;

// This shared data item allows a value for motor speed and direction to be shared between
// the task_user and task_motor for each motor 1 and 2. Task_user is the source of the data item, and task_motor
// is the sink that utilizes the data item.
TaskShare<int16_t>* p_share_1;
TaskShare<int16_t>* p_share_2;

// This shared data item is used to set the motor mode: power, brake, freewheel for motors
// 1 and 2. Motor 1 uses integers 0 through 2 and motor 2 uses integers 3 through 5.
TaskShare<uint8_t>* p_mode;

// This shared data item is used to read the state of the encoder tick used for comparison 
// in the ISR.
TaskShare<uint8_t>* p_state;

// This shared data item is used to hold the state of the last encoder tick from encoder 1 used for 
// comparison in the ISR.
TaskShare<uint8_t>* p_state_old_1;

// This shared data item is used to hold the state of the last encoder tick from encoder 2 used for 
// comparison in the ISR.
TaskShare<uint8_t>* p_state_old_2;

// This shared data item is used to hold the count for current number of errors. This number 
// is incremented in the ISR when applicable and read using error_count method.
TaskShare<uint32_t>* p_error_cntr;

// This shared data item is used to hold the count for current position of encoder 1. This number 
// is incremented or decremented based on comparsion of current and old states in the ISR and 
// read using view_count method.
TaskShare<uint32_t>* p_encoder_cntr_1;

// This shared data item is used to hold the count for current position of encoder 2. This number 
// is incremented or decremented based on comparsion of current and old states in the ISR and 
// read using view_count method.
TaskShare<uint32_t>* p_encoder_cntr_2;

// This shared data is used to hold the location of channel A external interrupt pin.
TaskShare<uint8_t>* p_ext_pin_A;

// This shared data is used to hold the location of channel B external interrupt pin.
TaskShare<uint8_t>* p_ext_pin_B;

// This shared data is used to hold the location of channel C external interrupt pin.
TaskShare<uint8_t>* p_ext_pin_C;

// This shared data is used to hold the location of channel D external interrupt pin.
TaskShare<uint8_t>* p_ext_pin_D;

// This shared data item is used to hold the position of motor 1 sent to the control loop
TaskShare<int16_t>* p_position_1;

// This shared data item is used to hold the position of motor 2 sent to the control loop
TaskShare<int16_t>* p_position_2;

// This shared data item is used to signal task_trigger to pull the gun's trigger
TaskShare<bool>* fire_at_will;

// These are shared data items to read the IR sensors
TaskShare<uint16_t>* p_high_left;
TaskShare<uint16_t>* p_high_right;
TaskShare<uint16_t>* p_center;
TaskShare<uint16_t>* p_low_left;
TaskShare<uint16_t>* p_low_right;

// These are shared data items that signal when a particular motor's control loop has 
// reached the desired value
TaskShare <bool>* p_pos_done_1;
TaskShare <bool>* p_pos_done_2;
//=====================================================================================
/** The main function sets up the RTOS.  Some test tasks are created. Then the 
 *  scheduler is started up; the scheduler runs until power is turned off or there's a 
 *  reset.
 *  @return This is a real-time microcontroller program which doesn't return. Ever.
 */

int main (void)
{
	// Disable the watchdog timer unless it's needed later. This is important because
	// sometimes the watchdog timer may have been left on...and it tends to stay on
	MCUSR = 0;
	wdt_disable ();

	// Configure a serial port which can be used by a task to print debugging infor-
	// mation, or to allow user interaction, or for whatever use is appropriate.  The
	// serial port will be used by the user interface task after setup is complete and
	// the task scheduler has been started by the function vTaskStartScheduler()
	rs232* p_ser_port = new rs232 (9600, 1);
	*p_ser_port << clrscr << PMS ("ME405 Term Project Tasks") << endl;

	// Create the queues and other shared data items here
	p_print_ser_queue = new TextQueue (32, "Print", p_ser_port, 10);
	
 	// Create shared variables for motor control
	p_share_1 = new TaskShare<int16_t> ("Speed_1");
	p_share_2 = new TaskShare<int16_t> ("Speed_2");
	p_mode = new TaskShare<uint8_t> ("Mode");
	p_state= new TaskShare<uint8_t> ("State");
	
	// Create shared variables for encoder states and positions
	p_state_old_1= new TaskShare<uint8_t> ("StateOld_1");
	p_state_old_2= new TaskShare<uint8_t> ("StateOld_2");
	p_position_1= new TaskShare<int16_t> ("Pos_1");
	p_position_2= new TaskShare<int16_t> ("Pos_2");
	
	// Create shared variables for encoder counters
	p_encoder_cntr_1= new TaskShare<uint32_t> ("EncoderCntr_1");
	p_encoder_cntr_2= new TaskShare<uint32_t> ("EncoderCntr_2");
	p_error_cntr= new TaskShare<uint32_t> ("ErrorCntr");
	
	// Create shared variables for the encoder external interrupt pins
	p_ext_pin_A= new TaskShare<uint8_t> ("ExtPinA");
	p_ext_pin_B= new TaskShare<uint8_t> ("ExtPinB");
	p_ext_pin_C= new TaskShare<uint8_t> ("ExtPinC");
	p_ext_pin_D= new TaskShare<uint8_t> ("ExtPinD");
	
	// Create shared variable for the trigger
	fire_at_will = new TaskShare<bool> ("Shoot_em_up");
	
//  Create shared variable for the phototransistor sensor readings
	p_high_left= new TaskShare<uint16_t> ("P_high_L"); 
	p_high_right= new TaskShare<uint16_t> ("P_high_R"); 
	p_center= new TaskShare<uint16_t> ("P_center"); 
	p_low_left= new TaskShare<uint16_t> ("P_low_L"); 
	p_low_right= new TaskShare<uint16_t> ("P_low_R"); 

	// Create shared variables for signaling when a desired position has been reached
	p_pos_done_1 = new TaskShare <bool> ("Pos_done_1");
	p_pos_done_2 = new TaskShare <bool> ("Pos_done_2");
	
	// The user interface is at low priority; it is only used to print debugging messages
	// and restart the microcontroller in this application
	new task_user ("UserInt", task_priority (0), 260, p_ser_port);

	// Create a task which outputs to a motor driver
	new task_motor ("Motor", task_priority (2), 280, p_ser_port);
	
	// Create a task which monitors the activity of the optical encoders
	new task_encoder ("Encoder", task_priority (1), 280, p_ser_port);
	
	// Create a task which utilized the motor and encoder tasks to provide closed-loop
	// control of a motor
	new task_control ("Controller", task_priority (3), 280, p_ser_port);
	
	// Create a task to scan a bank of phototransistors using an A/D converter
	new task_sensor ("Sensor", task_priority (1), 280, p_ser_port);
	
	// Create a task to pull the trigger
	new task_trigger ("Trigger", task_priority (0), 280, p_ser_port);
	
	// Create a task to determine the current motors' positions
	new task_position ("Position", task_priority (4), 280, p_ser_port);
	
	// Here's where the RTOS scheduler is started up. It should never exit as long as
	// power is on and the microcontroller isn't rebooted
	vTaskStartScheduler ();
}

