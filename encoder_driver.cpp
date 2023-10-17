//*************************************************************************************
/** @file encoder_driver.cpp
 *    This file contains a driver for an incremental optical encoder.This driver tracks
 *	  the encoder count and the number of errors in reading during operation. The 
 *    encoder ISR code is also located within this file.
 *
 *  Revisions:
 *    @li 02-09-2015 CAL, TT, JH encoder driver class implemented. Original file was
 *	  a LED brightness control created by JRR
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
#include <avr/interrupt.h>
#include <math.h>

#include "rs232int.h"                       // Include header for serial port class
#include "encoder_driver.h"                 // Include header for the encoder driver class

#include "shares.h"                         // Shared inter-task communications

//-------------------------------------------------------------------------------------
/** \brief This constructor sets up an instance of the class encoder_driver
 *  \details The encoder is made ready so that when a method such as @c view_count() is
 *  called, the corresponding registers can be correctly set. The constructor contains
 *  a protected pointer to the serial port for outputs. The constructor contains a 
 * 	protected pointer to the serial port for outputs. It also sets pointers for 
 *  encoder PORTs, DDRs, and pin addresses. It generates an interrupt to run on any
 *  logic change on the external interrupt input pins.
 *  @param p_serial_port A pointer to the serial port which writes debugging info. 
 *  @param i_port A pointer to the corresponding PORT for the encoder.
 *  @param i_ddr  A pointer to the corresponding DDR for the encoder.
 *  @param i_pin_0_a The pin number for EICRB register bit 0 of external pin A
 *					 the encoder is wired to
 *  @param i_pin_1_a The pin number for EICRB register bit 1 of external pin A
 *					 the encoder is wired to
 *  @param e_pin_a The pin number for the external input pin A the encoder is wired to.
 *  @param i_pin_0_b A pointer to the EICRB register bit 0 of the external pin B 
 *					 the encoder is wired to
 *  @param i_pin_1_b The pin number for EICRB register bit 0 of external pin B
 *					 the encoder is wired to
 *  @param e_pin_b The pin number for the external input pin A the encoder is wired to.
 */

Encoder::Encoder (emstream* p_serial_port, volatile uint8_t* i_port,  volatile uint8_t* i_ddr,
				 volatile uint8_t* i_port_in, uint8_t i_pin_0_a, uint8_t i_pin_1_a, 
				 uint8_t e_pin_a, uint8_t i_pin_0_b, uint8_t i_pin_1_b, uint8_t e_pin_b)
{	
		ptr_to_serial = p_serial_port;
			
		// Take constructor arguments and save them in corresponding protected variables
		// for the motor object
		INTERRUPT_PORT = i_port;
		INTERRUPT_DDR = i_ddr;
		INTERRUPT_PIN_0_A = i_pin_0_a;
		INTERRUPT_PIN_1_A = i_pin_1_a;
		INTERRUPT_PIN_0_B = i_pin_0_b;
		INTERRUPT_PIN_1_B = i_pin_1_b;
		EXT_PIN_NUMBER_A = e_pin_a;	
		EXT_PIN_NUMBER_B = e_pin_b;
		
		// Since two encoders are being used for Jankbot, this snippet of code ensures
		// that the correct set of external pins are configured for each instance of class
		// encoder_driver
		if(e_pin_b < 6)
		{
		p_ext_pin_A -> put (e_pin_a);
		p_ext_pin_B -> put (e_pin_b);
		}
		
		else
		{
		p_ext_pin_C -> put (e_pin_a);
		p_ext_pin_D -> put (e_pin_b);
		}
		
		// Initialize external interrupt pin pullup resistors
		*INTERRUPT_PORT |= (1 << EXT_PIN_NUMBER_A) | (1 << EXT_PIN_NUMBER_B);
		
		// Initialize external interrupt pins as inputs into AVR
		*INTERRUPT_DDR &= ~(1 << EXT_PIN_NUMBER_A) & ~(1 << EXT_PIN_NUMBER_B);
		
		// Enable interrupt triggering for external pins
		EIMSK |= (1 << EXT_PIN_NUMBER_A) | (1 << EXT_PIN_NUMBER_B);
		
		// Any logic change in input pins generate an interrupt
		EICRB |= (1 << INTERRUPT_PIN_0_A) | (1 << INTERRUPT_PIN_0_B);
		
		// Initializes the encoder's error count
		ERROR_COUNT = 0;
		p_error_cntr -> put(ERROR_COUNT);
		
		// Encoder debugging message
		DBG (ptr_to_serial, "Encoder constructor OK" << endl);
}

//-------------------------------------------------------------------------------------
/** @brief   Displays the current error count
 *  @details By using the shared variable p_error_cntr the error count is pulled and
 * 			 displayed through a debug message sent to the serial port.
 */

uint16_t Encoder::error_count(uint8_t enc_num)
{
	if (enc_num == 1)
	{
		ERROR_COUNT = p_error_cntr ->get();
	}
	
	else if (enc_num == 2)
	{
		ERROR_COUNT = p_error_cntr ->get();
	}

	// This message was used for debugging purposes, but is unused in this iteration
	// of the code.
// 	DBG (ptr_to_serial, "Can has error? " << ERROR_COUNT << endl); 
}

//-------------------------------------------------------------------------------------
/** @brief   Sets the encoder count to 0
 * 	@details Sets the share variable p_encoder_cntr to 0
 */

void Encoder::clear_count(uint8_t enc_num)
{
	ENCODER_COUNT = 0;
	if (enc_num == 1)
	{
		p_encoder_cntr_1 -> put(ENCODER_COUNT);
	}
	
	else if (enc_num == 2)
	{
		p_encoder_cntr_2 -> put(ENCODER_COUNT);
	}
}

//-------------------------------------------------------------------------------------
/** @brief   Displays the current encoder count
 *  @details Collects data from the shared variable p_encoder_cntr and displays it
 *  		 through a debug message sent to the serial port.
 */

void Encoder::view_count(uint8_t enc_num)
{
	if (enc_num == 1)
	{
		ENCODER_COUNT = p_encoder_cntr_1 ->get();
	}
	
	else if (enc_num == 2)
	{
		ENCODER_COUNT = p_encoder_cntr_2 ->get();
	}
	// This message was used for debugging purposes, but is unused in this iteration
	// of the code.
// 	DBG (ptr_to_serial, "Counting with Dora:" << (int32_t)ENCODER_COUNT << endl);
}

//-------------------------------------------------------------------------------------
/** @brief   Sets the encoder count to a specific input
 *  @details Sets the shared variable p_encoder_cntr to the NEW_COUNT
 * 	@param	 NEW_COUNT A uint32_t variable containing the new encoder count number
 */

void Encoder::set_count(uint8_t enc_num, uint32_t NEW_COUNT)
{
	if (enc_num == 1)
	{
		p_encoder_cntr_1->put(NEW_COUNT);
	}
	
	else if (enc_num == 2)
	{
		p_encoder_cntr_2->put(NEW_COUNT);
	}
}

//-------------------------------------------------------------------------------------
/** \brief   This overloaded operator "prints the encoder"
 *  \details This prints the relevant encoder registers PORTE. It is useful for
 * 			 debugging purposes but is not utilized in this build of the driver.
 *  @param   serpt Reference to a serial port to which the printout will be printed
 *  @param   vroom Reference to the motor driver which is being printed
 *  @return  A reference to the same serial device on which we write information.
 *           This is used to string together things to write with @c << operators
 */

emstream& operator << (emstream& serpt, Encoder& vroom)
{
//  These messages were used for debugging purposes, and are not printed during normal
//  operation.
	
// 	serpt << PMS ("PORTE: ") <<bin << PORTE <<endl;

	return (serpt);
}

//-------------------------------------------------------------------------------------
/** This interrupt service routine runs whenever an external input pin changes either
 *  from low to high or from high to low. The ISR stores the encoder count and
 *  determines the direction the encoder is running by checking the current state with
 * 	the previous state. It also checks that the encoder does not skip a count, if so it
 *	increments the error counter.
 */

// Encoder 1:
ISR (INT4_vect)
{
	// Declaration of state variables and input of shared values
	uint8_t STATE = PINE;
	uint8_t STATE_OLD= p_state_old_1 -> ISR_get();
	uint8_t EXT_PIN_NUMBER_A= p_ext_pin_A-> ISR_get();
	uint8_t EXT_PIN_NUMBER_B= p_ext_pin_B-> ISR_get();
	uint32_t ENCODER_COUNT= p_encoder_cntr_1-> ISR_get();
	uint32_t ERROR_COUNT= p_error_cntr-> ISR_get();
	
	// For Channel A and Channel B state: 00
	if (!(STATE & (1 <<EXT_PIN_NUMBER_A)) && !(STATE & (1 <<EXT_PIN_NUMBER_B)))
	{	
		// If previous state was 01, increment ENCODER_COUNT in positive direction
		if (!(STATE_OLD & (1 <<EXT_PIN_NUMBER_A)) && (STATE_OLD & (1 <<EXT_PIN_NUMBER_B)))
		{
			ENCODER_COUNT++;
		}
		
		// If previous state was 10, increment ENCODER_COUNT in negative direction
		else if ((STATE_OLD & (1 <<EXT_PIN_NUMBER_A)) && !(STATE_OLD & (1 <<EXT_PIN_NUMBER_B)))
		{
			ENCODER_COUNT--;
		}
		
		// If previous state was neither 01 or 10, increment ERROR_COUNT
		else
		{
			ERROR_COUNT++;
		}
	}
	
	// 01
	if (!(STATE & (1 <<EXT_PIN_NUMBER_A)) && (STATE & (1 <<EXT_PIN_NUMBER_B)))
	{
		// If previous state was 11, increment ENCODER_COUNT in positive direction
		if ((STATE_OLD & (1 <<EXT_PIN_NUMBER_A)) && (STATE_OLD & (1 <<EXT_PIN_NUMBER_B)))
		{
			ENCODER_COUNT++;
		}
		
		// If previous state was 00, increment ENCODER_COUNT in negative direction
		else if (!(STATE_OLD & (1 <<EXT_PIN_NUMBER_A)) && !(STATE_OLD & (1 <<EXT_PIN_NUMBER_B)))
		{
			ENCODER_COUNT--;
		}
		
		// If previous state was neither 11 or 00, increment ERROR_COUNT
		else
		{
			ERROR_COUNT++;
		}
	}
	
	// 10
	if ((STATE & (1 <<EXT_PIN_NUMBER_A)) && !(STATE & (1 <<EXT_PIN_NUMBER_B)))
	{
		// If previous state was 00, increment ENCODER_COUNT in positive direction
		if (!(STATE_OLD & (1 <<EXT_PIN_NUMBER_A)) && !(STATE_OLD & (1 <<EXT_PIN_NUMBER_B)))
		{
			ENCODER_COUNT++;
		}
		
		// If previous state was 11, increment ENCODER_COUNT in negative direction
		else if ((STATE_OLD & (1 <<EXT_PIN_NUMBER_A)) && (STATE_OLD & (1 <<EXT_PIN_NUMBER_B)))
		{
			ENCODER_COUNT--;
		}
		
		// If previous state was neither 00 or 11, increment ERROR_COUNT
		else
		{
			ERROR_COUNT++;
		}
	}
	
	// 11
	if ((STATE & (1 <<EXT_PIN_NUMBER_A)) && (STATE & (1 <<EXT_PIN_NUMBER_B)))
	{
		// If previous state was 10, increment ENCODER_COUNT in positive direction
		if ((STATE_OLD & (1 <<EXT_PIN_NUMBER_A)) && !(STATE_OLD & (1 <<EXT_PIN_NUMBER_B)))
		{
			ENCODER_COUNT++;
		}
		
		// If previous state was 01, increment ENCODER_COUNT in negative direction
		else if (!(STATE_OLD & (1 <<EXT_PIN_NUMBER_A)) && (STATE_OLD & (1 <<EXT_PIN_NUMBER_B)))
		{
			ENCODER_COUNT--;
		}
		
		// If previous state was neither 10 or 01, increment ERROR_COUNT
		else
		{
			ERROR_COUNT++;
		}
	}
		
		p_encoder_cntr_1->ISR_put(ENCODER_COUNT);
		p_state_old_1->ISR_put(STATE);
		p_error_cntr->ISR_put(ERROR_COUNT);	
}

// Alias external interrupt pin 5 to input pin 4 interrupt service routine
ISR_ALIAS(INT5_vect, INT4_vect);


// Encoder 2:
ISR (INT6_vect)
{
	// Declaration of state variables and input of shared values
	uint8_t STATE = PINE;
	uint8_t STATE_OLD= p_state_old_2 -> ISR_get();
	uint8_t EXT_PIN_NUMBER_A= p_ext_pin_C-> ISR_get();
	uint8_t EXT_PIN_NUMBER_B= p_ext_pin_D-> ISR_get();
	uint32_t ENCODER_COUNT= p_encoder_cntr_2-> ISR_get();
	uint32_t ERROR_COUNT= p_error_cntr-> ISR_get();
	
	// For Channel A and Channel B state: 00
	if (!(STATE & (1 <<EXT_PIN_NUMBER_A)) && !(STATE & (1 <<EXT_PIN_NUMBER_B)))
	{	
		// If previous state was 01, increment ENCODER_COUNT in positive direction
		if (!(STATE_OLD & (1 <<EXT_PIN_NUMBER_A)) && (STATE_OLD & (1 <<EXT_PIN_NUMBER_B)))
		{
			ENCODER_COUNT++;
		}
		
		// If previous state was 10, increment ENCODER_COUNT in negative direction
		else if ((STATE_OLD & (1 <<EXT_PIN_NUMBER_A)) && !(STATE_OLD & (1 <<EXT_PIN_NUMBER_B)))
		{
			ENCODER_COUNT--;
		}
		
		// If previous state was neither 01 or 10, increment ERROR_COUNT
		else
		{
			ERROR_COUNT++;
		}
	}
	
	// 01
	if (!(STATE & (1 <<EXT_PIN_NUMBER_A)) && (STATE & (1 <<EXT_PIN_NUMBER_B)))
	{
		// If previous state was 11, increment ENCODER_COUNT in positive direction
		if ((STATE_OLD & (1 <<EXT_PIN_NUMBER_A)) && (STATE_OLD & (1 <<EXT_PIN_NUMBER_B)))
		{
			ENCODER_COUNT++;
		}
		
		// If previous state was 00, increment ENCODER_COUNT in negative direction
		else if (!(STATE_OLD & (1 <<EXT_PIN_NUMBER_A)) && !(STATE_OLD & (1 <<EXT_PIN_NUMBER_B)))
		{
			ENCODER_COUNT--;
		}
		
		// If previous state was neither 11 or 00, increment ERROR_COUNT
		else
		{
			ERROR_COUNT++;
		}
	}
	
	// 10
	if ((STATE & (1 <<EXT_PIN_NUMBER_A)) && !(STATE & (1 <<EXT_PIN_NUMBER_B)))
	{
		// If previous state was 00, increment ENCODER_COUNT in positive direction
		if (!(STATE_OLD & (1 <<EXT_PIN_NUMBER_A)) && !(STATE_OLD & (1 <<EXT_PIN_NUMBER_B)))
		{
			ENCODER_COUNT++;
		}
		
		// If previous state was 11, increment ENCODER_COUNT in negative direction
		else if ((STATE_OLD & (1 <<EXT_PIN_NUMBER_A)) && (STATE_OLD & (1 <<EXT_PIN_NUMBER_B)))
		{
			ENCODER_COUNT--;
		}
		
		// If previous state was neither 00 or 11, increment ERROR_COUNT
		else
		{
			ERROR_COUNT++;
		}
	}
	
	// 11
	if ((STATE & (1 <<EXT_PIN_NUMBER_A)) && (STATE & (1 <<EXT_PIN_NUMBER_B)))
	{
		// If previous state was 10, increment ENCODER_COUNT in positive direction
		if ((STATE_OLD & (1 <<EXT_PIN_NUMBER_A)) && !(STATE_OLD & (1 <<EXT_PIN_NUMBER_B)))
		{
			ENCODER_COUNT++;
		}
		
		// If previous state was 01, increment ENCODER_COUNT in negative direction
		else if (!(STATE_OLD & (1 <<EXT_PIN_NUMBER_A)) && (STATE_OLD & (1 <<EXT_PIN_NUMBER_B)))
		{
			ENCODER_COUNT--;
		}
		
		// If previous state was neither 10 or 01, increment ERROR_COUNT
		else
		{
			ERROR_COUNT++;
		}
	}
		
		p_encoder_cntr_2->ISR_put(ENCODER_COUNT);
		p_state_old_2->ISR_put(STATE);
		p_error_cntr->ISR_put(ERROR_COUNT);	
}

// Alias external interrupt pin 7 to input pin 6 interrupt service routine
ISR_ALIAS(INT7_vect, INT6_vect);
