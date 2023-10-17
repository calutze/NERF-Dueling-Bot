//**************************************************************************************
/** @file task_control.h
 *    This file contains the header for a task class that reads and controls an encoder 
 * 	  of an electric motor.
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

// This define prevents this .h file from being included multiple times in a .cpp file
#ifndef _TASK_CONTROL_H_
#define _TASK_CONTROL_H_

//Include relevent files
#include <stdlib.h>                         // Prototype declarations for I/O functions
#include <avr/io.h>                         // Header for special function registers

#include "FreeRTOS.h"                       // Primary header for FreeRTOS
#include "task.h"                           // Header for FreeRTOS task functions
#include "queue.h"                          // FreeRTOS inter-task communication queues

#include "taskbase.h"                       // ME405/507 base task class
#include "time_stamp.h"                     // Class to implement a microsecond timer
#include "taskqueue.h"                      // Header of wrapper for FreeRTOS queues
#include "textqueue.h"                      // Header for a "<<" queue class
#include "taskshare.h"                      // Header for thread-safe shared data

#include "rs232int.h"                       // ME405/507 library for serial comm.
#include "motor_driver.h"					// Header for Motor driver class
#include "encoder_driver.h"                 // Header for Encoder driver class

#include "emstream.h"                       // Header for serial ports and devices

//-------------------------------------------------------------------------------------
/** @brief   This task controls and reads a motor with an encoder
 *  @details The encoder is read and controller is run using a driver in files @c encoder_driver.h and 
 *           @c encoder_driver.cpp.
 */

class task_control : public TaskBase
{
	private:
		// No private variables or methods for this class

	protected:
		int32_t current_pos_1;
		int32_t current_pos_2;
		int32_t ref_pos_1;
		int32_t ref_pos_2;
		const double KP_1 = .5;
		const double KI_1 = .05;
		const double KP_2 = 1;
		const double KI_2 = .01;
		int8_t KD;
		int32_t error_old_1;
		int32_t error_old_2;
		int32_t error_1;
		int32_t error_2;
		double KP_out_1;
		double KP_out_2;
		double KI_out_1;
		double KI_out_2;
		double KD_out_1;
		double KD_out_2;
		double speed_out_1;
		double speed_out_2;
		int8_t dead_zone;
		uint16_t hinge_limit;
		uint8_t count;

	public:
		// This constructor creates a generic task of which many copies can be made
		task_control (const char*, unsigned portBASE_TYPE, size_t, emstream*);

		// This method is called by the RTOS once to run the task loop for ever and ever.
		void run (void);
};

// This operator prints the A/D converter (see file adc.cpp for details). It's not 
// a part of class adc, but it operates on objects of class adc
// emstream& operator << (emstream&, adc&);
emstream& operator << (emstream&, task_control&);

#endif // _TASK_CONTROL_H_
