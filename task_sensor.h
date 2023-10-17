//**************************************************************************************
/** @file task_sensor.h
 *    This file contains the header for a task class that uses an array of phototransistors 
 * 	  to scan for IR light.
 *
 *  Revisions:
 *    @li 02-17-2015 CAL, TT, JH Original file was a LED brightness control created
 * 					  by JRR. This file has been modified to contain a scanning system
 * 					  for IR light.
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
#ifndef _TASK_SENSOR_H_
#define _TASK_SENSOR_H_

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

#include "emstream.h"                       // Header for serial ports and devices
#include "adc.h"							// Header for A/D converter class

//-------------------------------------------------------------------------------------
/** @brief   This task reads input from the phototransistor sensor array
 *  @details The A/D converters for each phototransistor are read and stored in shared
 *           data space for use in other tasks.
 */

class task_sensor : public TaskBase
{
	private:
		// No private variables or methods for this class

	protected:
		// Each phototransistor in the array is labeled based on physical location
		// in the array.
		uint16_t high_left;
		uint16_t high_right;
		uint16_t center;
		uint16_t low_left;
		uint16_t low_right;

	public:
		// This constructor creates a generic task of which many copies can be made
		task_sensor (const char*, unsigned portBASE_TYPE, size_t, emstream*);
		
		// This method is called by the RTOS once to run the task loop for ever and ever.
		void run (void);
};

// This operator prints the task_sensor (see file task_sensor.cpp for details). It's not 
// a part of class task_sensor, but it operates on objects of class task_sensor
emstream& operator << (emstream&, task_sensor&);

#endif // _TASK_SENSOR_H_
