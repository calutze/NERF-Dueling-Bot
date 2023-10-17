//**************************************************************************************
/** @file task_control.cpp
 *    This file contains the code for a task class which controls motor position
 *	  of an electric motor.
 *
 *  Revisions:
 *    @li 02-17-2015 CAL, TT, JH Original file was a LED brightness control created
 * 					  by JRR. This file has been modified to contain motor control
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
#include "task_control.h"                	// Header for this task
#include "shares.h"                         // Shared inter-task communications
#include <math.h>                           // Includes math library

#define brake_1 0                           // These defines help make the code more
#define free_1  1							// readable. Enumeration data types were 
#define power_1 2							// not implemented for the shared data item
#define brake_2 3							// p_mode, thus defining the integers in this 
#define free_2  4							// way makes the code easier to read and 
#define power_2 5							// understand.

//-------------------------------------------------------------------------------------
/** This constructor creates a task which reads input from an encoder and controls the 
 *  encoder using input from @c task_user. The main job of this constructor is to call the
 *  constructor of parent class (\c frt_task ).
 *  @param a_name A character string which will be the name of this task
 *  @param a_priority The priority at which this task will initially run (default: 0)
 *  @param a_stack_size The size of this task's stack in bytes 
 *                      (default: configMINIMAL_STACK_SIZE)
 *  @param p_ser_dev Pointer to a serial device (port, radio, SD card, etc.) which can
 *                   be used by this task to communicate (default: NULL)
 */

task_control::task_control (const char* a_name, unsigned portBASE_TYPE a_priority, 
							size_t a_stack_size, emstream* p_ser_dev)
							: TaskBase (a_name, a_priority, a_stack_size, p_ser_dev)
{
	
	// Nothing is done in the body of this constructor. All the work is done in the
	// call to the frt_task constructor on the line just above this one
}


//-------------------------------------------------------------------------------------
/** This method is called once by the RTOS scheduler. It constructs the encoder 
 *  to run using external interrupts. Each time around the for (;;) loop, the encoder 
 *  is updated with the latest shared variables from the motor and/or task_user.
 */

void task_control::run (void)
{
	// Make a variable which will hold times to use for precise task scheduling
	TickType_t previousTicks = xTaskGetTickCount ();

	dead_zone = 20;
	hinge_limit = 1100;
	
	// This is the task loop for the control task. This loop runs until the
	// power is turned off or something equally dramatic occurs
	for (;;)
	{
		ref_pos_1 = p_position_1->get();
		ref_pos_2 = p_position_2->get();
						
		current_pos_1 = p_encoder_cntr_1->get();
		current_pos_2 = p_encoder_cntr_2->get();
		
		error_1 = ref_pos_1 - current_pos_1;
		error_2 = ref_pos_2 - current_pos_2;

		KP_out_1 = error_1*KP_1;
		KP_out_2 = error_2*KP_2;
		
		KI_out_1 = ((error_old_1 + error_1) * KI_1);
		KI_out_2 = ((error_old_2 + error_2) * KI_2);
			
		speed_out_1 = (KP_out_1 + KI_out_1 + KD_out_1);
		speed_out_2 = (KP_out_2 + KI_out_2 + KD_out_2);
			
// 		MOTOR 1:	
		// Brakes the motor if close to final position
		if ((error_1<=10 && error_1>=-10) || (speed_out_1==0))
		{
			p_mode -> put(brake_1);
			p_pos_done_1 -> put(true);
		}
			
 		// Brakes the motor if close to hinge limit
		else if ((speed_out_1 > 1) && (current_pos_1 >= hinge_limit))
		{				
			p_mode -> put(brake_1);
		}
		// Brakes the motor if going past zero
		else if ((speed_out_1 < -1) && (current_pos_1 <= 0))
		{
			p_mode -> put(brake_1);
		}
		
		// Positive speed cap
		else if (speed_out_1 > 300)
		{
			speed_out_1 =300;
			p_mode -> put(power_1);
		}
		
		// Motor will not spin unless power is greater than that defined by
		// the dead_zone variable
		else if (speed_out_1 < dead_zone && speed_out_1>0)
		{
			speed_out_1=dead_zone;
			p_mode -> put(power_1);
		}
		
		else if (speed_out_1 > -dead_zone && speed_out_1<0)
		{
			speed_out_1=-dead_zone;
			p_mode -> put(power_1); 
		}
		
		// Negative speed cap
		else if(speed_out_1 < -300)
		{
			speed_out_1 = -300;
			p_mode -> put(power_1);
		}
				
		else
		{
			p_mode -> put(power_1);
		}
		
		error_old_1 = error_1;
		p_share_1 -> put(speed_out_1);
		p_position_1 -> put(ref_pos_1);
		
		//Delay control loop between motor 1 and motor 2
		delay_from_for_ms (previousTicks, 30);	
		
// 		MOTOR 2:
		// Brakes the motor if close to final position
		if((error_2<=30 && error_2>=-30) /*|| (speed_out_2==0)*/)
		{
			p_mode -> put(brake_2);
			p_pos_done_2 -> put(true);
		}
		
		// Positive speed cap
		else if(speed_out_2 > 300)
		{
			speed_out_2 =300;
			p_mode -> put(power_2);
		}
		
		// Motor will not spin unless power is greater than that defined by
		// the dead_zone variable
		else if ((speed_out_2 < dead_zone) && (speed_out_2>0))
		{
			speed_out_2=dead_zone;
			p_mode -> put(power_2);
		}
		
		else if ((speed_out_2 > -dead_zone) && (speed_out_2<0))
		{
			speed_out_2=-dead_zone;
			p_mode -> put(power_2);
		}
		
		// Negative speed cap
		else if(speed_out_2 < -300)
		{
			speed_out_2 = -300;
			p_mode -> put(power_2);
		}
				
		else
		{
			p_mode -> put(power_2);
		}
			
		error_old_2 = error_2;
		p_share_2 -> put(speed_out_2);
		p_position_2 -> put(ref_pos_2);
		
		// Outputs position to serial port for debugging.
		// Note: motors will not run without ARS being printed to the serial port for some reason
		*p_print_ser_queue  << "A: "<< current_pos_2 << endl;
		*p_print_ser_queue  << "R: "<< ref_pos_2 << endl;
		*p_print_ser_queue  << "S: "<< speed_out_2 << endl;
		
		// This is a method we use to cause a task to make one run through its task
		// loop every N milliseconds and let other tasks run at other times
		delay_from_for_ms (previousTicks, 30);		
	}
}