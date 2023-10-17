//**************************************************************************************
/** @file task_position.h
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
#ifndef _TASK_POSITION_H_
#define _TASK_POSITION_H_

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
/** @brief   This task controls and reads an encoder
 *  @details The encoder readed and controler is run using a driver in files 
 */

class task_position : public TaskBase
{
	private:
		// No private variables or methods for this class

	protected:		
		// Storage for reference position variable
		int16_t pos_1;
		int16_t pos_2;
		
		// Maximum position for hinge before rack falls out
		uint16_t hinge_limit;
		
		// Threshold for light detection
		uint16_t threshold;
		
		// Limits for directions
		uint8_t base_r_limit;
		uint8_t base_l_limit;
		
		// Run count for scanner
		uint8_t runs;
		
		// Done flags
		bool done_1;
		bool done_2;
		
		// Center tolerance
		uint8_t tol;
		
		// Each phototransistor in the array labeled as Row_Column
		uint16_t high_left;
		uint16_t high_right;
		uint16_t center;
		uint16_t low_left;
		uint16_t low_right;
	public:
		// This constructor creates a generic task of which many copies can be made
		task_position (const char*, unsigned portBASE_TYPE, size_t, emstream*);
		
		// This method is called by the RTOS once to run the task loop for ever and ever.
		void run (void);
};

// This operator prints the A/D converter (see file adc.cpp for details). It's not 
// a part of class adc, but it operates on objects of class adc
// emstream& operator << (emstream&, adc&);
emstream& operator << (emstream&, task_position&);

#endif // _TASK_POSITION_H_
