//**************************************************************************************
/** @file task_encoder.cpp
 *    This file contains the code for a task class which controls an encoder
 *	  on an electric motor. It tests the encoder by calling methods from encoder_driver.cpp.
 *
 *  Revisions:
 *    @li 02-09-2015 CAL, TT, JH Original file was a LED brightness control created
 * 					  by JRR. This file has been modified to con
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
#include "task_encoder.h"                	// Header for this task
#include "shares.h"                         // Shared inter-task communications

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

task_encoder::task_encoder (const char* a_name, unsigned portBASE_TYPE a_priority, 
							size_t a_stack_size, emstream* p_ser_dev)
							: TaskBase (a_name, a_priority, a_stack_size, p_ser_dev)
{
	// Nothing is done in the body of this constructor. All the work is done in the
	// call to the frt_task constructor on the line just above this one
}


//-------------------------------------------------------------------------------------
/** This method is called once by the RTOS scheduler. It constructs the encoder 
 *  to run using external interrupts. Each time around the for (;;) loop, the encoder 
 *  is updated with the latest shared variables from the motor, control loop, and/or task_user.
 */

void task_encoder::run (void)
{
	// Make a variable which will hold times to use for precise task scheduling
	TickType_t previousTicks = xTaskGetTickCount ();
	
	// Constructer call for the encoder driver, p_encoder_1 measures the gun angle
	Encoder* p_encoder_1 = new Encoder (p_serial, &PORTE, &DDRE, &PINE, ISC40, ISC41, 4, ISC50, 
									  ISC51, 5);
	
	// p_encoder_2 measures the base rotation
	Encoder* p_encoder_2 = new Encoder (p_serial, &PORTE, &DDRE, &PINE, ISC60, ISC61, 6, ISC70, 
									  ISC71, 7);
	
	// This is the task loop for the encoder task. This loop runs until the
	// power is turned off or something equally dramatic occurs
	for (;;)
	{		
		// This is a method we use to cause a task to make one run through its task
		// loop every N milliseconds and let other tasks run at other times
		delay (100);		
	}
}