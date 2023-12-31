//**************************************************************************************
/** @file task_user.h
 *    This file contains header stuff for a user interface task for a ME405/FreeRTOS
 *    test suite. 
 *
 *  Revisions:
 *    @li 09-30-2012 JRR Original file was a one-file demonstration with two tasks
 *    @li 10-05-2012 JRR Split into multiple files, one for each task
 *    @li 10-25-2012 JRR Changed to a more fully C++ version with class task_user
 *    @li 11-04-2012 JRR Modified from the data acquisition example to the test suite
 *    @li 01-04-2014 JRR Changed base class names to TaskBase, TaskShare, etc.
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
#ifndef _TASK_USER_H_
#define _TASK_USER_H_

#include <stdlib.h>                         // Prototype declarations for I/O functions

#include "FreeRTOS.h"                       // Primary header for FreeRTOS
#include "task.h"                           // Header for FreeRTOS task functions
#include "queue.h"                          // FreeRTOS inter-task communication queues

#include "rs232int.h"                       // ME405/507 library for serial comm.
#include "time_stamp.h"                     // Class to implement a microsecond timer
#include "taskbase.h"                       // Header for ME405/507 base task class
#include "taskqueue.h"                      // Header of wrapper for FreeRTOS queues
#include "textqueue.h"                      // Header for a "<<" queue class
#include "taskshare.h"                      // Header for thread-safe shared data

#include "shares.h"                         // Global ('extern') queue declarations
#include "encoder_driver.h"					// Header for encoder_driver file


/// This macro defines a string that identifies the name and version of this program. 
#define PROGRAM_VERSION		PMS ("ME405 Lab 1 Unmodified Program V0.01 ")


//-------------------------------------------------------------------------------------
/** @brief   This task interacts with the user to tell him/her how to interface 
 *           with the system. 
 *  @details This task sends information to the shared variables that dictate the motors'
 *           mode of operation and velocity (when in a running mode only).
 */

class task_user : public TaskBase
{
	private:
		// No private variables or methods for this class

	protected:
		// This method displays a simple help message telling the user what to do. It's
		// protected so that only methods of this class or possibly descendents can use it
		void print_help_message (void);
		
		// This method displays a simple help massage telling the user how to select an
		// encoder operating mode. It's protected so that only methods of this class or
		// possible descendents can use it
		void print_encoder_message (void);
		
		// This method displays a simple help message telling the user how to select a motor 
		// operating mode. It's protected so that only methods of this class or possibly 
		// descendents can use it
		void print_motochoice_message (void);
		
		// This method displays a simple help message telling the user how to select a motor 
		// rotation direction. It's protected so that only methods of this class or possibly 
		// descendents can use it
		void print_direction_message(void);

		// This method displays information about the status of the system
		void show_status (void);

	public:
		// This constructor creates a user interface task object
		task_user (const char*, unsigned portBASE_TYPE, size_t, emstream*);

		/** This method is called by the RTOS once to run the task loop for ever and ever.
		*/
		void run (void);
};

#endif // _TASK_USER_H_
