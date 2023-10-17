//======================================================================================
/** @file motor_driver.h
 *    This file contains a DC motor driver for the VNH3SP30 motor driver attached to the
 *	  ME 405 board. This driver can control separate motors, causing them to either 
 *    rotate clockwise, counterclockwise, brake, or freely rotate. The driver is 
 * 	  hopefully thread safe in FreeRTOS due to the use of a mutex to prevent its use by
 *	  multiple tasks at the same time. There is no protection from priority inversion,
 *	  however, except for the priority elevation in the mutex.
 *
 *  Revisions:
 *    @li 01-15-2008 JRR Original (somewhat useful) file
 *    @li 10-11-2012 JRR Less original, more useful file with FreeRTOS mutex added
 *    @li 10-12-2012 JRR There was a bug in the mutex code, and it has been fixed
 *    @li 01-26-2015 CAL, TT, JH motor driver class implemented
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
#ifndef MOTOR_DRIVER
#define MOTOR_DRIVER

#include "emstream.h"                       // Header for serial ports and devices
#include "FreeRTOS.h"                       // Header for the FreeRTOS RTOS
#include "task.h"                           // Header for FreeRTOS task functions
#include "queue.h"                          // Header for FreeRTOS queues
#include "semphr.h"                         // Header for FreeRTOS semaphores


//-------------------------------------------------------------------------------------
/** @brief   This class will operate the VNH3SP30 motor driver on an AVR processor.
 *  @details The class contains a protected pointer to the serial port for outputs.
 * 			 It also contains pointers for motor driver PORTs, DDRs, and pin addresses.
 * 			 Public function set_power controls the motor speed based on a signed 16-bit
 * 			 input. Public function brake grounds both the motor leads so it will not
 *			 spin. Public function freewheel sets the motor speed to zero, allowing it to
 * 			 rotate freely.
 */

class Motor
{
	protected:
		/// The Motor class uses this pointer to the serial port to say hello
		emstream* ptr_to_serial;
		
		// Pointers to motor driver PORTs, DDRs, and pin addresses
		volatile uint8_t* INa_PORT;
		volatile uint8_t* INa_DDR;
		uint8_t   INa_PIN;
		volatile uint8_t* INb_PORT;
		volatile uint8_t* INb_DDR;
		uint8_t   INb_PIN;
		volatile uint8_t* DIAG_PORT;
		volatile uint8_t* DIAG_DDR;
		uint8_t   DIAG_PIN;
		volatile uint8_t* PWM_PORT;
		volatile uint8_t* PWM_DDR;
		uint8_t   PWM_PIN;
		volatile uint16_t* PWM_OCR;

    public:
		// The constructor sets up the Motor driver for use. The "= NULL" part is a
		// default parameter, meaning that if that parameter isn't given on the line
		// where this constructor is called, the compiler will just fill in "NULL".
		// In this case that has the effect of turning off diagnostic printouts
		Motor (emstream* = NULL, volatile uint8_t* a_port = NULL, volatile uint8_t* a_d = NULL, 
			  uint8_t a_pin = 0, volatile uint8_t* b_port = NULL, volatile uint8_t* b_d = NULL, 
			  uint8_t b_pin = 0, volatile uint8_t* diag_po = NULL, volatile uint8_t* diag_d = NULL, 
			  uint8_t diag_pi = 0, volatile uint8_t* pwm_po = NULL, volatile uint8_t* pwm_d = NULL, 
			  uint8_t pwm_pi = 0, volatile uint16_t* pwm_oc = NULL);
		
		// The set_power function 
		void set_power(int16_t speed);
		
		// The freewheel function
		void freewheel(void);
		
		// The brake function
		void brake(void);

}; // end of class Motor


// This operator prints the Motor diagnostics (see file Motor.cpp for details). It's not 
// a part of class Motor, but it operates on objects of class Motor
emstream& operator << (emstream&, Motor&);

#endif // MOTOR_DRIVER
