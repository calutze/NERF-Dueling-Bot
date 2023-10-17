//**************************************************************************************
/** @file task_sensor.cpp
 *    This file contains the header for a task class that uses an array of phototransistors 
 * 	  to scan for IR light.
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

#include "textqueue.h"                      // Header for text queue class
#include "task_sensor.h"                		// Header for this task
#include "shares.h"                         // Shared inter-task communications
#include <math.h>                           // Includes math library

//-------------------------------------------------------------------------------------
/** This constructor creates a task which reads input from a phototransistor array  
 *  The main job of this constructor is to call the constructor of parent class 
 *  (\c frt_task ).
 *  @param a_name A character string which will be the name of this task
 *  @param a_priority The priority at which this task will initially run (default: 0)
 *  @param a_stack_size The size of this task's stack in bytes 
 *                      (default: configMINIMAL_STACK_SIZE)
 *  @param p_ser_dev Pointer to a serial device (port, radio, SD card, etc.) which can
 *                   be used by this task to communicate (default: NULL)
 */

task_sensor::task_sensor (const char* a_name, unsigned portBASE_TYPE a_priority, 
					  size_t a_stack_size, emstream* p_ser_dev)
					  : TaskBase (a_name, a_priority, a_stack_size, p_ser_dev)
{
	
	// Nothing is done in the body of this constructor. All the work is done in the
	// call to the frt_task constructor on the line just above this one
}

//-------------------------------------------------------------------------------------
/** This method is called once by the RTOS scheduler. The A/D converters associated with each 
 *  phototransistor are oversampled on each pass. The readings are stored in shared variables for use
 *  in other tasks.
 */

void task_sensor::run (void)
{
	// Make a variable which will hold times to use for precise task scheduling
	TickType_t previousTicks = xTaskGetTickCount ();
	
	// Create an analog to digital converter driver object and a variable in which to
	// store its output.
	adc* p_adc = new adc (p_serial);
	
	// This is the task loop for the sensor task. This loop runs until the
	// power is turned off or something equally dramatic occurs
	for (;;)
	{
		// The A/D converter for each sensor is oversampled in these lines
		center = p_adc -> read_oversampled(0,4);
		high_left = p_adc -> read_oversampled(4,4);
		high_right = p_adc -> read_oversampled(3,4);
		low_left = p_adc -> read_oversampled(2,4);
		low_right = p_adc -> read_oversampled(1,4);
		
		// The A/D readings are stored in shared variables for use in the control loop
		p_high_left-> put(high_left);
		p_high_right-> put(high_right);
		p_center-> put(center);
		p_low_left-> put(low_left);
		p_low_right-> put(low_right);

		// This is a method we use to cause a task to make one run through its task
		// loop every N milliseconds and let other tasks run at other times
		delay_from_for_ms (previousTicks, 100);		
	}
}

