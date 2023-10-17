//*************************************************************************************
/** @file adc.cpp
 *    This file contains a simple A/D converter driver.
 *
 *  Revisions:
 *    @li 01-15-2008 JRR Original (somewhat useful) file
 *    @li 10-11-2012 JRR Less original, more useful file with FreeRTOS mutex added
 *    @li 10-12-2012 JRR There was a bug in the mutex code, and it has been fixed
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

#include "rs232int.h"                       // Include header for serial port class
#include "adc.h"                            // Include header for the A/D class


//-------------------------------------------------------------------------------------
/** \brief This constructor sets up an A/D converter. 
 *  \details The A/D is made ready so that when a method such as @c read_once() is 
 *  called, correct A/D conversions can be performed. The ADC Control and Status
 *  Register is set such that the reference voltage source is from AVCC with external
 *  Capacitor at AREF pin and the clock prescaler is at a division factor of 32. 
 *  @param p_serial_port A pointer to the serial port which writes debugging info. 
 */

adc::adc (emstream* p_serial_port)
{
	ptr_to_serial = p_serial_port;

	ADCSRA |= (1<<ADEN)|(1<<ADPS0)|(1<<ADPS2);
	ADMUX  |= (1<<REFS0);

	// Print a handy debugging message
	DBG (ptr_to_serial, "A/D constructor OK" << endl);
}


//-------------------------------------------------------------------------------------
/** @brief   This method takes one A/D reading from the given channel and returns it. 
 *  @details This function performs a single A/D conversion based on the paramenter ch. The
 * 			 selected channel input is read and the ADMUX and ADCSRA registers are set
 * 			 accordingly. The A/D conversion is initiated and the code waits until
 * 			 the ADSC bit returns to 0, signifying the conversion is complete. Finally,
 * 			 the A/D conversion result is stored and the value returned.
 *  @param   ch The A/D channel which is being read must be from 0 to 7
 *  @return  The result of the A/D conversion
 */

uint16_t adc::read_once (uint8_t ch)
{
	uint16_t result;
		
	// Set channels depending on input parameter ch
	switch (ch) 
	{
	  case 0:
	    ADMUX  &= ~(1<<MUX4)& ~(1<<MUX3)& ~(1<<MUX2)& ~(1<<MUX1)& ~(1<<MUX0);
	    ADCSRA |= (1<<ADSC);
	    break;
	  
	  case 1:
	    ADMUX  &= ~(1<<MUX4)& ~(1<<MUX3)& ~(1<<MUX2)& ~(1<<MUX1);
	    ADMUX  |= (1<<MUX0);
	    ADCSRA |= (1<<ADSC);
	    break;
	    
	  case 2:
	    ADMUX  &= ~(1<<MUX4)& ~(1<<MUX3)& ~(1<<MUX2)& ~(1<<MUX0);
	    ADMUX  |= (1<<MUX1);
	    ADCSRA |= (1<<ADSC);
	    break;
	    
	  case 3:
	    ADMUX  &= ~(1<<MUX4)& ~(1<<MUX3)& ~(1<<MUX2);
	    ADMUX  |= (1<<MUX1)|(1<<MUX0);
	    ADCSRA |= (1<<ADSC);
	    break;
	    
	  case 4:
	    ADMUX  &= ~(1<<MUX4)& ~(1<<MUX3)& ~(1<<MUX1)& ~(1<<MUX0);
	    ADMUX  |= (1<<MUX2);
	    ADCSRA |= (1<<ADSC);
	    break;
	    
	  case 5:
	    ADMUX  &= ~(1<<MUX4)& ~(1<<MUX3)& ~(1<<MUX1);
	    ADMUX  |= (1<<MUX2)|(1<<MUX0);
	    ADCSRA |= (1<<ADSC);
	    break;
	    
	  case 6:
	    ADMUX  &= ~(1<<MUX4)& ~(1<<MUX3)& ~(1<<MUX0);
	    ADMUX  |= (1<<MUX2)|(1<<MUX1);
	    ADCSRA |= (1<<ADSC);
	    break;
	    
	  case 7:
	    ADMUX  &= ~(1<<MUX4)& ~(1<<MUX3);
	    ADMUX  |= (1<<MUX2)|(1<<MUX1)|(1<<MUX0);
	    ADCSRA |= (1<<ADSC);
	    break;
	    
	  default :
	    break;
	}

	while(ADCSRA & (1<<ADSC))
	{}

 	result = (ADCL | (ADCH<<8));
	return (result);
}


//-------------------------------------------------------------------------------------
/** @brief   This method averages multiple readings from the A/D conversion and returns the average.
 *  \details This function takes a set number of samples from the read_once function and averages them
 *			together to reduce the effects of noise.  
 *  @param   channel The A/D channel which is being read must be from 0 to 7.
 *  @param   samples Number of samples to be averaged.
 *  @return  The result of the averaged A/D conversion
 */

uint16_t adc::read_oversampled (uint8_t channel, uint8_t samples)
{
	uint8_t  count;
	uint16_t sum;
	uint16_t average;
	uint16_t result;
	
	average = 0;
	count = 0;
	sum = 0;
	
	if (samples > 10)
	{
	  samples = 10;
	}
	
	while (count <= samples)
	{
	result = adc::read_once(channel);
	sum = (result + sum);
	count ++;
	}

	average = (sum/count);
	count = 0;
	
	return (average);
}

//-------------------------------------------------------------------------------------
/** \brief   This overloaded operator "prints the A/D converter." 
 *  \details This prints the A/D control registers ADCSRA and ADMUX as well as the 
 * 			 conversion value from an oversampled conversion with oversampling of 5
 *  @param   serpt Reference to a serial port to which the printout will be printed
 *  @param   a2d   Reference to the A/D driver which is being printed
 *  @return  A reference to the same serial device on which we write information.
 *           This is used to string together things to write with @c << operators
 */

emstream& operator << (emstream& serpt, adc& a2d)
{
	// Print contents of A/D control registers (ADMUX, ADCSRA) in binary
	// Show the current reading of channel 0 with oversampling of 5
	 
// 	 serpt << PMS ("ADCSRA: ") <<bin <<ADCSRA <<endl;
// 	 serpt << PMS ("ADMUX: ") <<bin <<ADMUX <<endl;
// 	 serpt << PMS ("A/D Conversion Value: ") <<dec << a2d.read_oversampled (0,5) <<endl;
	
	return (serpt);
}

