//**************************************************************************************
/** @file task_motor.cpp
 *    This file contains the code for a task class which controls the operation of a
 *    DC motor using PWM inputs for speed set by the control loop. The motor's mode of 
 *    operation is also set by the control loop.
 *
 *  Revisions:
 *    @li 09-30-2012 JRR Original file was a one-file demonstration with two tasks
 *    @li 10-05-2012 JRR Split into multiple files, one for each task
 *    @li 10-25-2012 JRR Changed to a more fully C++ version with class task_sender
 *    @li 10-27-2012 JRR Altered from data sending task into LED blinking class
 *    @li 11-04-2012 JRR Altered again into the multi-task monstrosity
 *    @li 12-13-2012 JRR Yet again transmogrified; now it controls LED brightness
 *    @li 01-26-2015 CAL, TT, JH Original file was a LED brightness control created
 * 					  by JRR
 *
 *  License:
 *    This file is copyright 2012 by JR Ridgely and released under the Lesser GNU 
 *    Public License, version 2. It intended for educational use only, but its use
 *    is not limited thereto. */
/*    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 *    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 *    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 *    ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 *    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUEN-
 *    TIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
 *    OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 *    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
 *    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
//**************************************************************************************

#include "textqueue.h"                      // Header for text queue class
#include "task_motor.h"                		// Header for this task
#include "shares.h"                         // Shared inter-task communications

#define brake_1 0                           // These defines help make the code more
#define free_1  1							// readable. Enumeration data types were 
#define power_1 2							// not implemented for the shared data item
#define brake_2 3							// p_mode, thus defining the integers in this 
#define free_2  4							// way makes the code easier to read and 
#define power_2 5							// understand.

//-------------------------------------------------------------------------------------
/** This constructor creates a task which controls the running mode and speed of a DC 
 *  motor using input from @c task_user. The main job of this constructor is to call the
 *  constructor of parent class (\c frt_task ).
 *  @param a_name A character string which will be the name of this task
 *  @param a_priority The priority at which this task will initially run (default: 0)
 *  @param a_stack_size The size of this task's stack in bytes 
 *                      (default: configMINIMAL_STACK_SIZE)
 *  @param p_ser_dev Pointer to a serial device (port, radio, SD card, etc.) which can
 *                   be used by this task to communicate (default: NULL)
 */

task_motor::task_motor (const char* a_name, unsigned portBASE_TYPE a_priority, 
						size_t a_stack_size, emstream* p_ser_dev)
						: TaskBase (a_name, a_priority, a_stack_size, p_ser_dev)
{
	// Nothing is done in the body of this constructor. All the work is done in the
	// call to the frt_task constructor on the line just above this one
}


//-------------------------------------------------------------------------------------
/** This method is called once by the RTOS scheduler. It constructs the motor driver
 *  to run using fast PWM for two separate instances. Each time around the for (;;)
 *  loop, the motor driver is updated with the latest shared variables from task_user.
 */

void task_motor::run (void)
{
	// Make a variable which will hold times to use for precise task scheduling
	TickType_t previousTicks = xTaskGetTickCount ();
	
	// The following lines of code construct the motor driver for two separate instances
	// of the motor_driver. Each instance controls a different motor. p_motor_1 controls
	// the gun angle
	Motor* p_motor_1 = new Motor (p_serial, &PORTC, &DDRC, 0, &PORTC, &DDRC, 1, &PORTC,
								  &DDRC, 2, &PORTB, &DDRB, 6, &OCR1B);
	
	// p_motor_2 drives the base rotations
	Motor* p_motor_2 = new Motor (p_serial, &PORTD, &DDRD, 5, &PORTD, &DDRD, 6, &PORTD,
								  &DDRD, 7, &PORTB, &DDRB, 5, &OCR1A);
	
	
	// These lines configure fast 8-bit fast PWM for motor 2
	TCCR1A |= (1 << WGM10)
			 | (1 << COM1A1) | (1 << COM1B1);
	TCCR1A &= ~(1 << COM1A0) & ~(1 << COM1B0);

	// The CS11 and CS10 bits set the prescaler for this timer/counter to run the
	// timer at F_CPU / 64
	TCCR1B |= (1 << WGM12)
			 | (1 << CS11)  | (1 << CS10);

	// To set 8-bit fast PWM mode we must set bits WGM30 and WGM32, which are in two
	// different registers (ugh). We use COM3B1 and Com3B0 to set up the PWM so that
	// the pin output will have inverted sense, that is, a 0 is on and a 1 is off; 
	// this is needed because the LED connects from Vcc to the pin. 
	TCCR3A |= (1 << WGM30)
			 | (1 << COM3B1) | (1 << COM3B0);

	// The CS31 and CS30 bits set the prescaler for this timer/counter to run the
	// timer at F_CPU / 64
	TCCR3B |= (1 << WGM32)
			 | (1 << CS31)  | (1 << CS30);

	// This is the task loop for the motor control task. This loop runs until the
	// power is turned off or something equally dramatic occurs
	for (;;)
	{
		// The shared variables are stored locally for use as inputs to the motor driver
		mode = p_mode->get();
		speed_1 = p_share_1->get();
		speed_2 = p_share_2->get();
		
		// Motor 1 is controlled using modes 0 through 2, which correspond to brake_1,
		// free_1, and power_1 respectively. Speed is only used as an input to the method
		// set_power.
		if(mode < 3)
		{
			switch(mode)
			{
				case(brake_1):
					p_motor_1->brake();
					break;
				
				case(free_1):
					p_motor_1->freewheel();
					break;
				
				case(power_1):
					p_motor_1->set_power(speed_1);
					break;
				
				default:
					DBG (p_serial, "ERROR...ERROR... Abandon hope" << endl);
					break;
			}
		}
		// Motor 2 is controlled using modes 3 through 5, which correspond to brake_2,
		// free_2, and power_2 respectively. Speed is only used as an input to the method
		// set_power.
		else
		{
			switch(mode)
			{
				case(brake_2):
					p_motor_2->brake();
					break;
				
				case(free_2):
					p_motor_2->freewheel();
					break;
				
				case(power_2):
					p_motor_2->set_power(speed_2);
					break;
				
				default:
					DBG (p_serial, "ERROR...ERROR... Abandon hope" << endl);
					break;
			}
		}
				
		// This enables motor driver to print debug messages
		*p_serial << (*p_serial, *p_motor_1);
		*p_serial << (*p_serial, *p_motor_2);
		
		// This is a method we use to cause a task to make one run through its task
		// loop every N milliseconds and let other tasks run at other times
		delay_from_for_ms (previousTicks, 100);		
	}
}