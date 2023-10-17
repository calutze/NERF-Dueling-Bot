//**************************************************************************************
/** @file task_trigger.cpp
 *    This file contains the code for a task class which controls the trigger of Jankbot
 *
 *  Revisions:
 *    @li 09-30-2012 JRR Original file was a one-file demonstration with two tasks
 *    @li 10-05-2012 JRR Split into multiple files, one for each task
 *    @li 10-25-2012 JRR Changed to a more fully C++ version with class task_sender
 *    @li 10-27-2012 JRR Altered from data sending task into LED blinking class
 *    @li 11-04-2012 JRR Altered again into the multi-task monstrosity
 *    @li 12-13-2012 JRR Yet again transmogrified; now it controls LED brightness
 *    @li 03-08-2015 CL, JH, & TT change purpose of code; now it controls trigger pull
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
#include "task_trigger.h"                   // Header for this task
#include "shares.h"                         // Shared inter-task communications


//-------------------------------------------------------------------------------------
/** This constructor creates a task which controls the trigger pull of Jankbot using
 *  input from task_control. The main job of this constructor is to call the
 *  constructor of parent class (\c frt_task ); the parent's constructor the work.
 *  @param a_name A character string which will be the name of this task
 *  @param a_priority The priority at which this task will initially run (default: 0)
 *  @param a_stack_size The size of this task's stack in bytes 
 *                      (default: configMINIMAL_STACK_SIZE)
 *  @param p_ser_dev Pointer to a serial device (port, radio, SD card, etc.) which can
 *                   be used by this task to communicate (default: NULL)
 */

task_trigger::task_trigger (const char* a_name, 
								 unsigned portBASE_TYPE a_priority, 
								 size_t a_stack_size,
								 emstream* p_ser_dev
								)
	: TaskBase (a_name, a_priority, a_stack_size, p_ser_dev)
{
	// Nothing is done in the body of this constructor. All the work is done in the
	// call to the frt_task constructor on the line just above this one
}


//-------------------------------------------------------------------------------------
/** This method is called once by the RTOS scheduler. Each time around the for (;;)
 *  loop, it checks the trigger flag to set PWM for the servo motor to pull the trigger. 
 */

void task_trigger::run (void)
{
	// Make a variable which will hold times to use for precise task scheduling
	TickType_t previousTicks = xTaskGetTickCount ();

	// Configure counter/timer 1 as a PWM for servo motor. 
	DDRB = (1 << 7);
	
	// The PWM is set to fast mode with ICR1 acting as the upper limit for output compare
	TCCR1A |= (1 << WGM11) | (1 << COM1C1) | (1 << COM1C0);
			 
	// The CS11 and CS10 bits set the prescaler for this timer/counter to run the
	// timer at F_CPU / 64
	TCCR1B |= (1 << WGM12) | (1 << WGM13) | (1 << CS11) | (1 << CS10);
	
	// This upper limit was calculated to achieve a desired output frequency of 30 Hz
	ICR1 = 7499;
	
	// Start run timer at 0
	runs = 0;

	// This is the task loop for the servo controlled trigger pull. This loop runs until the
	// power is turned off or something equally dramatic occurs.
	for (;;)
	{
	
		//Pull trigger is fire_at_will is true and keep it there for 100 runs
		if((ready = fire_at_will -> get()) && (runs < 50))
		{
			OCR1C = 1799;
			runs++;
		}
		
		// Release trigger and clear runs and fire_at_will
		else if (runs < 100)
		{
			OCR1C = 0;
			runs++;
			fire_at_will -> put(false);
		}
		
		else
		{
			runs = 0;
		}
		
		// This is a method we use to cause a task to make one run through its task
		// loop every N milliseconds and let other tasks run at other times
		delay_from_for_ms (previousTicks, 50);
		
	}
}

