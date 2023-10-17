//**************************************************************************************
/** @file task_position.cpp
 *    This file contains the header for a task class that uses an array of phototransistors 
 * 	  to scan for IR light and determine the desired position for the turret based on that
 *    data.
 *
 *  Revisions:
 *    @li 03-07-2015 CAL, TT, JH Original file was a LED brightness control created
 * 					  by JRR. This file has been modified to contain position
 * 					  determination and reading of an optical sensor
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
// This define prevents this .h file from being included multiple times in a .cpp file

// #include "textqueue.h"                      // Header for text queue class
#include "task_position.h"                		// Header for this task
#include "shares.h"                         // Shared inter-task communications
#include <math.h>                           // Includes math library

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

task_position::task_position (const char* a_name, unsigned portBASE_TYPE a_priority, 
					  size_t a_stack_size, emstream* p_ser_dev)
					  : TaskBase (a_name, a_priority, a_stack_size, p_ser_dev)
{
	
	// Nothing is done in the body of this constructor. All the work is done in the
	// call to the frt_task constructor on the line just above this one
	state = 0;
}

//-------------------------------------------------------------------------------------
/** This method is called once by the RTOS scheduler. It constructs the encoder 
 *  to run using external interrupts. Each time around the for (;;) loop, the encoder 
 *  is updated with the latest shared variables from the motor and/or task_user.
 */

void task_position::run (void)
{
	// Make a variable which will hold times to use for precise task scheduling
	TickType_t previousTicks = xTaskGetTickCount ();
	
	state = 0;
	hinge_limit = 500;
	threshold = 10;
	base_r_limit = 600;
	base_l_limit = 1000;
	runs = 0;
	tol = 50;
	p_pos_done_1 -> put(false);
	p_pos_done_2 -> put(false);
	
	// This is the task loop for the position task. This loop runs until the
	// power is turned off or something equally dramatic occurs
	for (;;)
	{		
		switch (state)
		{
			// Spin 160 degrees from start position to face the target area
			case (0):
				p_position_1 -> put(0);
				p_position_2 -> put(700);
				done_1 = p_pos_done_1 -> get();
				done_2 = p_pos_done_2 -> get();
				if ((done_1 == true) && (done_2 == true))
				{
					transition_to (1);
				}
				break;
			
			// Search for light source in target area
			case (1):
				// Gather shared variables into local variables
				high_left = p_high_left -> get();
				high_right = p_high_right-> get();
				center = p_center-> get();
				low_left = p_low_left-> get();
				low_right = p_low_right-> get();
				
				pos_1 = p_position_1 -> get();
				pos_2 = p_position_2 -> get();
 				
				done_1 = p_pos_done_1 -> get();
				done_2 = p_pos_done_2 -> get();
				
				// If any sensor reads above the threshold transition to state 2
				if ((high_left >= threshold) || (high_right >= threshold) || 
					(center >= threshold) || (low_left >= threshold) ||
					(low_right >= threshold))
				{
					transition_to (2);
				}
				
				if (done_1==true && done_2==true)
				{
					p_pos_done_1-> put(false);
					p_pos_done_2-> put(false);
					
					// If above hinge limit, go down
					if (pos_1 >= hinge_limit)
					{
						pos_1 -= 10;
						p_position_1 -> put(pos_1);
					}
					
					// search pattern
					else
					{
						pos_1 += 50;
						if (runs == 0)
						{
							pos_2 += 100;
							if (pos_2 >= base_l_limit)
							{
								runs = 1;
							}
						}
						else
						{
							pos_2 -= 100;
							if (pos_2 <= base_r_limit)
							{
								runs = 0;
							}
						}
					}
										
					p_position_1 -> put(pos_1);
					p_position_2 -> put(pos_2);
				}

				break;
		
			// Lock on to target once the light source is found
			case (2):
				// Gather shared variables into local variables
				high_left = p_high_left -> get();
				high_right = p_high_right-> get();
				center = p_center-> get();
				low_left = p_low_left-> get();
				low_right = p_low_right-> get();
				
				pos_1 = p_position_1 -> get();
				pos_2 = p_position_2 -> get();
				
				done_1 = p_pos_done_1 -> get();
				done_2 = p_pos_done_2 -> get();
				
				// Wait until task_control sets done flags, indicating the reference
				// position has been reached
				if (done_1==true && done_2==true)
				{
					p_pos_done_1-> put(false);
					p_pos_done_2-> put(false);
					
					// Pull trigger if in final position
					if ((center > (low_right - tol)) && (center > (high_right - tol)) 
						&& (center > (low_left - tol)) && (center > (low_right - tol))
						&& (center < (low_right + tol)) && (center < (high_right + tol)) 
						&& (center < (low_left - tol)) && (center < (low_right + tol)))
					{
						fire_at_will->put(true);
					}
									
					else if((high_left> center) || (low_left> center) )
					{
						pos_2+=50;
					}
					
					else if((high_right> center) || (low_right> center) )
					{
						pos_2-=50;
					}
					
					else if((high_right> center) || (high_left> center) )
					{
						pos_1+=10;
					}
					
					else if((low_right> center) || (low_left> center) )
					{
						pos_1-=10;
					}
					else
					{
						*p_print_ser_queue << "Borked" << endl;
					}
					p_position_1 -> put(pos_1);
					p_position_2 -> put(pos_2);
				}
				break;
					
			default:
				*p_print_ser_queue  << "Borked" << endl;
				break;
		}
		
		// This is a method we use to cause a task to make one run through its task
		// loop every N milliseconds and let other tasks run at other times
		delay_from_for_ms (previousTicks, 50);	
	}
}