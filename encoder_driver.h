//======================================================================================
/** @file encoder_driver.h
 *    This file contains a driver for an incremental optical encoder.This driver tracks
 *	  the encoder count and the number of errors in reading during operation. The driver
 *	  is hopefully thread safe in FreeRTOS due to the use of a mutex to prevent its use
 *	  by multiple tasks at the same time. There is no protection from priority inversion,
 *	  however, except for the priority elevation in the mutex.
 *
 *  Revisions:
 *    @li 02-09-2015 CAL, TT, JH encoder driver class implemented. Original file was
 *	  a LED brightness control created by JRR
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
//======================================================================================

// This define prevents this .H file from being included multiple times in a .CPP file
#ifndef ENCODER_DRIVER
#define ENCODER_DRIVER

#include "emstream.h"                       // Header for serial ports and devices
#include "FreeRTOS.h"                       // Header for the FreeRTOS RTOS
#include "task.h"                           // Header for FreeRTOS task functions
#include "queue.h"                          // Header for FreeRTOS queues
#include "semphr.h"                         // Header for FreeRTOS semaphores
#include "taskshare.h"                      // Header for thread-safe shared data
#include "textqueue.h"                      // Header of wrapper for FreeRTOS queues
#include "shares.h"							// Shared inter-task communications

//-------------------------------------------------------------------------------------
/** @brief   This class will read an encoder connected to the AVR processor by any of
 * 			 the external input pins 4 through 7.
 *  @details The class contains a protected pointer to the serial port for outputs.
 * 			 It also contains pointers for encoder PORTs, DDRs, and pin addresses.
 * 			 Public function error_count displays the number of errors accumulated.
 * 			 Public function clear_count zeros the encoder count. Public function
 * 			 view_count displays the current encoder count.  Public function set_count
 * 			 sets the encoder count according to a user input.
 */

class Encoder
{
	protected:
		/// The Encoder class uses this pointer to the serial port to say hello
		emstream* ptr_to_serial;
		
		// Pointers to encoder PORTs, DDRs, and pin addresses
		volatile uint8_t* INTERRUPT_PORT;
		volatile uint8_t* INTERRUPT_DDR;
		uint8_t   INTERRUPT_PIN_0_A;
		uint8_t   INTERRUPT_PIN_1_A;
		uint8_t   INTERRUPT_PIN_0_B;
		uint8_t   INTERRUPT_PIN_1_B;
		uint8_t   EXT_PIN_NUMBER_A;
		uint8_t   EXT_PIN_NUMBER_B;
		
		// Storage variables for encoder class
		uint32_t  ENCODER_COUNT;
		uint32_t  ERROR_COUNT;
		uint8_t   STATE;
		uint8_t   STATE_OLD;

    public:
		// The constructor sets up the Encoder driver for use. The "= NULL" part is a
		// default parameter, meaning that if that parameter isn't given on the line
		// where this constructor is called, the compiler will just fill in "NULL".
		// In this case that has the effect of turning off diagnostic printouts
		Encoder (emstream* = NULL, volatile uint8_t* i_port = NULL,
				 volatile uint8_t* i_ddr = NULL, volatile uint8_t* i_port_in = NULL,
				 uint8_t i_pin_0_a = 0, uint8_t i_pin_1_a = 0, uint8_t e_pin_a = 0,
				 uint8_t i_pin_0_b = 0, uint8_t i_pin_1_b = 0, uint8_t e_pin_b = 0);
		
		// Method that counts errors
		uint16_t error_count(uint8_t enc_num);
		
		// Method that clears the encoder count
		void clear_count(uint8_t enc_num);
		
		// Method that allows for encoder count viewing
		void view_count(uint8_t enc_num);
		
		// Method that sets encoder count
		void set_count(uint8_t enc_num, uint32_t NEW_COUNT);

}; // end of class Encoder

// This operator prints the Encoder diagnostics (see file Encoder.cpp for details). It's not 
// a part of class Encoder, but it operates on objects of class Encoder
emstream& operator << (emstream&, Encoder&);

#endif // ENCODER_DRIVER