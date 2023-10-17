//*************************************************************************************
/** @file motor_driver.cpp
 *    This file contains a DC motor driver for the VNH3SP30 motor driver attached to the
 *	  ME 405 board. This driver can control separate motors, causing them to either 
 *    rotate clockwise, counterclockwise, brake, or freely rotate.
 *
 *  Revisions:
 *    @li 01-15-2008 JRR Original (somewhat useful) file
 *    @li 10-11-2012 JRR Less original, more useful file with FreeRTOS mutex added
 *    @li 10-12-2012 JRR There was a bug in the mutex code, and it has been fixed
 *    @li 01-26-2015 CAL, TT, JH motor driver class implemented
 *
 *  License:
 *    This file is copyright 2015 by JR Ridgely and released under the Lesser GNU 
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
//*************************************************************************************

#include <stdlib.h>                         // Include standard library header files
#include <avr/io.h>
#include <math.h>

#include "rs232int.h"                       // Include header for serial port class
#include "motor_driver.h"                   // Include header for the motor driver class

//-------------------------------------------------------------------------------------
/** \brief This constructor sets up a motor driver to work with the VNH3SP30 motor 
 * 	driver. 
 *  \details The motor driver is made ready so that when a method such as
 * 	@c set_power() is called, the corresponding registers on the VNH3SP30 can be
 * 	correctly set. The constructor contains a protected pointer to the serial port for
 *	outputs. It also contains pointers for motor driver PORTs, DDRs, and pin addresses.
 * 	Public function set_power controls the motor speed based on a signed 16-bit
 * 	input. Public function brake grounds both the motor leads so it will not
 *	spin. Public function freewheel sets the motor speed to zero, allowing it to
 * 	rotate freely.
 *  @param p_serial_port A pointer to the serial port which writes debugging info. 
 *  @param a_port A pointer to the port corresponding to VNH3SP30 pin INa
 *  @param a_d A pointer to the Data Direction Register for VNH3SP30 pin INa
 *  @param a_pin An integer for the pin/bit location corresponding to VNH3SP30 pin INa
 *  @param b_port A pointer to the port corresponding to VNH3SP30 pin INb
 *  @param b_d A pointer to the Data Direction Register for VNH3SP30 pin INb
 *  @param b_pin An integer for the pin/bit location corresponding to VNH3SP30 pin INb
 *  @param diag_po A pointer to the port corresponding to VNH3SP30 diagnostic pin: 
 * 					DIAGA/B 
 *  @param diag_d A pointer to the Data Direction Register for VNH3SP30 diagnostic pin:
 *					 DIAGA/B
 *  @param diag_pi An integer for the pin/bit location corresponding to VNH3SP30 
 * 					diagnostic pin: DIAGA/B
 *  @param pwm_po A pointer to the port corresponding to VNH3SP30 PWM pin
 *  @param pwm_d A pointer to the Data Direction Register for VNH3SP30 pin INb
 *  @param pwm_pi An integer for the pin/bit location corresponding to VNH3SP30 pin INa
 *  @param pwm_oc A pointer to the Output Compare Register for the PWM
 */

Motor::	Motor (emstream* p_serial_port, volatile uint8_t* a_port, volatile uint8_t* a_d, 
			  uint8_t a_pin, volatile uint8_t* b_port, volatile uint8_t* b_d, 
			  uint8_t b_pin, volatile uint8_t* diag_po, volatile uint8_t* diag_d, 
			  uint8_t diag_pi, volatile uint8_t* pwm_po, volatile uint8_t* pwm_d, 
			  uint8_t pwm_pi, volatile uint16_t* pwm_oc)
{	
		ptr_to_serial = p_serial_port;
			
		// Take constructor arguments and save them in corresponding protected variables
		// for the motor object
		INa_PORT = a_port;
		INa_DDR = a_d;
		INa_PIN = a_pin;
		INb_PORT = b_port;
		INb_DDR = b_d;
		INb_PIN = b_pin;
		DIAG_PORT = diag_po;
		DIAG_DDR = diag_d;
		DIAG_PIN = diag_pi;
		PWM_PORT = pwm_po;
		PWM_DDR = pwm_d;
		PWM_PIN = pwm_pi;
		PWM_OCR = pwm_oc;
		
		// Initialize port for PWM as an output and as high
 		*PWM_PORT |= (1 << PWM_PIN);
		*PWM_DDR |= (1 << PWM_PIN);
	
		// Initialize port for the VNH3SP30 motor driver diagnostic pins, the port is
		// an input to the AVR and the pullup resistor is enabled
		*DIAG_PORT |= (1 << DIAG_PIN);
		*DIAG_DDR &= ~(1 << DIAG_PIN);
		
		// Initialize the Data Direction Registers for INa and INb pins as outputs
		*INa_DDR |= (1 << INa_PIN);
		*INb_DDR |= (1 << INb_PIN);
		
		// Motor debugging message
		DBG (ptr_to_serial, "Motor constructor OK" << endl);
}

//-------------------------------------------------------------------------------------
/** @brief   Takes a 16 bit signed input and sets the motor torque
 *  @details A positive input speed will spin the motor clockwise by setting INa to
 * 			 high and set the PWM Output Compare Register to the input speed. A negative
 *			 input speed will spin the motor counterclockwise by setting INb to high and
 *			 set the PWM Output Compare Register to the input speed. 
 *  @param   speed A value used to set the PWM duty cycle
 */

void Motor::set_power(int16_t speed)
{
	//set motor to spin clockwise, by setting INa to high and set PWM period to input speed
	if (speed > 0)
	{
		*INa_PORT |= (1 << INa_PIN);
		*INb_PORT &= ~(1 <<INb_PIN);

		*PWM_OCR = abs(speed);
	}
	//set motor to spin counterclockwise, by setting INb to high and set PWM period to input speed
	else
	{
		*INb_PORT |= (1 << INb_PIN);
		*INa_PORT &= ~(1 << INa_PIN);

		*PWM_OCR = abs(speed);
	}
}	

//-------------------------------------------------------------------------------------
/** @brief   This method toggles motor freewheeling
 * 	@details By setting the PWM Output Compare Register to zero, the motor will spin
 * 			 freely.
 */

void Motor::freewheel(void)
{
	*PWM_OCR = 0;
}

//-------------------------------------------------------------------------------------
/** @brief   Sets the motor to a braked state
 *  @details Grounds both the INa and INb pins so that the motor is braked. Sets the PWM 
 *           for maximum braking power.
 */

void Motor::brake(void)
{
	*INa_PORT &= ~(1 << INa_PIN);
	*INb_PORT &= ~(1 << INb_PIN);
	
	*PWM_OCR = 255;
}

//-------------------------------------------------------------------------------------
/** \brief   This overloaded operator "prints the motor driver"
 *  \details This prints the motor driver control registers PORTC, PORTD, PORTB, DDRC
 * 			 DDRD, DDRB, OCR1A, and OCR1B. This is useful for debugging purposes.
 *  @param   serpt Reference to a serial port to which the printout will be printed
 *  @param   vroom Reference to the motor driver which is being printed
 *  @return  A reference to the same serial device on which we write information.
 *           This is used to string together things to write with @c << operators
 */

emstream& operator << (emstream& serpt, Motor& vroom)
{
//  These messages were used for debugging purposes, and are not printed during normal
//  operation.
	
// 	serpt << PMS ("PORTC: ") <<bin << PORTC <<endl;
// 	serpt << PMS ("DDRC:  ") <<bin << DDRC <<endl;
// 	serpt << PMS ("PORTD: ") <<bin << PORTD <<endl;
// 	serpt << PMS ("DDRD:  ") <<bin << DDRD <<endl;
// 	serpt << PMS ("PORTB: ") <<bin << PORTB <<endl;
// 	serpt << PMS ("DDRB:  ") <<bin << DDRB <<endl;
// 	serpt << PMS ("PWM 1: ") <<bin << OCR1A <<endl;
// 	serpt << PMS ("PWM 2: ") <<bin << OCR1B <<endl;
	return (serpt);
}