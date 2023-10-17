//**************************************************************************************
/** @file task_motor.h
 *    This file contains the header for a task class that controls the operation of a
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
 *    @li 01-26-2015 CAL, TT, JH motor driver class calls
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
#ifndef _TASK_MOTOR_H_
#define _TASK_MOTOR_H_

#include <stdlib.h>                         // Prototype declarations for I/O functions
#include <avr/io.h>                         // Header for special function registers

#include "FreeRTOS.h"                       // Primary header for FreeRTOS
#include "task.h"                           // Header for FreeRTOS task functions
#include "queue.h"                          // FreeRTOS inter-task communication queues

#include "taskbase.h"                       // ME405/507 base task class
#include "time_stamp.h"                     // Class to implement a microsecond timer
#include "taskqueue.h"                      // Header of wrapper for FreeRTOS queues
#include "taskshare.h"                      // Header for thread-safe shared data

#include "rs232int.h"                       // ME405/507 library for serial comm.
#include "motor_driver.h"					// Header for Motor driver class
#include "encoder_driver.h"                 // Header for Encoder driver class

#include "emstream.h"                       // Header for serial ports and devices

//-------------------------------------------------------------------------------------
/** @brief   This task controls two separate motors using a motor driver and shared 
 *           variables set in task_user
 *  @details The motor controller is run using a driver in files @c motor_driver.h and 
 *           @c motor_driver.cpp. Code in this task sets up a timer/counter in PWM mode 
 *           and controls the motors' modes and run speeds. 
 */

class task_motor : public TaskBase
{
	private:
		// No private variables or methods for this class

	protected:
		uint8_t mode;
		int16_t speed_1;
		int16_t speed_2;

	public:
		// This constructor creates a generic task of which many copies can be made
		task_motor (const char*, unsigned portBASE_TYPE, size_t, emstream*);

		// This method is called by the RTOS once to run the task loop for ever and ever.
		void run (void);
};

// This operator prints the A/D converter (see file adc.cpp for details). It's not 
// a part of class adc, but it operates on objects of class adc
// emstream& operator << (emstream&, adc&);
emstream& operator << (emstream&, Motor&);

#endif // _TASK_MOTOR_H_
