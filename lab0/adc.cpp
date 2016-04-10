//*************************************************************************************
/** @file adc.cpp
 *    This file contains a very simple A/D converter driver. This driver should be
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
 *  \details The A/D is made ready so that when a  method such as @c read_once() is 
 *  called, correct A/D conversions can be performed. 
 *  @param p_serial_port A pointer to the serial port which writes debugging info. 
 */

adc::adc (emstream* p_serial_port)
{
	ptr_to_serial = p_serial_port;

	// Enables the ADC and sets the division factor to 32.
	ADCSRA |= ((0x01 << ADEN) | (0x01 << ADPS2) | (0x01 << ADPS0));
	ADMUX |= (0x01 << REFS0); // Sets AVCC with external capacitor at AREF pin
	// Print a handy debugging message
	DBG (ptr_to_serial, "A/D constructor OK" << endl);
}


//-------------------------------------------------------------------------------------
/** @brief   This method takes one A/D reading from the given channel and returns it. 
 *  @details Reads from channel 1, waits for conversion to finish, outputs the conversion
 *  @param   channel The A/D channel which is being read must be from 0 to 7
 *  @return  The result of the A/D conversion
 */

uint16_t adc::read_once (uint8_t channel)
{
	uint16_t result = 0;
	static uint8_t oldChannel = 0;
	
	ADMUX &= 0xF8; // by masking the upper 5 bits, clear the selected channel
	ADMUX |= (channel & 0x07); // set new channel, limit to 0-7 by ignoring higher bits

	ADCSRA |= (0x01 << ADSC); // start a conversion
	
	while (ADCSRA & (0x01 << ADSC)) {} // while ADSC is 1, conversion is in progress, so wait.
	if (channel != oldChannel) // if we've changed channels since the last read,
	{
	  oldChannel = channel; // note that this has happened
	  result = read_once(channel); // throw away the current value, take a second sample
	}
	else result = ((uint16_t)(ADCL) | (uint16_t)(ADCH << 8)); // 16 bit number: ADCH:ADCL
	return result;
}


//-------------------------------------------------------------------------------------
/** @brief   This method takes mulitiple A/D readings and takes the average
 *  \details Reads n A/D conversion samples and takes the average. It also checks for
 *  overflow each time a new sample is read. Once the given number of samples is reached
 *  or adding the next sample creates overflow, the average is computed and returned         
 *  @param   channel The A/D channel which is being read. Must be from 0:7
 *  @param   samples The number of samples to be read
 *  @return  The result of the A/D conversion average
 */

uint16_t adc::read_oversampled (uint8_t channel, uint8_t samples)
{
        int i;               // tracks samples taken
        uint16_t oldSum = 0; // previous sum of samples
	uint16_t newSum = 0; // current loop's sum of samples
        
        for(i = 0; i <= samples; i++)
	{
           newSum = oldSum + read_once(channel); // add in the latest sample
	   if(newSum < oldSum) // if overflow has occured
	   {
	      newSum = oldSum; // revert to the previous value
	      *ptr_to_serial << "sample overflow caught" << endl;
	      break; // stop taking samples
	   }
	   oldSum = newSum; // update the sum
	}
	//*ptr_to_serial << "sample: " << (newSum / i) << endl;
	return (newSum / i); // return the average of the samples
}


//-------------------------------------------------------------------------------------
/** \brief   This overloaded operator prints A/D values
 *  \details Prints outs the value fo the ADMUX and ADCSRA register values at time of conversion
 *  Also prints the current value being read by the A/D.
 *  @param   serpt Reference to a serial port to which the printout will be printed
 *  @param   a2d   Reference to the A/D driver which is being printed
 *  @return  A reference to the same serial device on which we write information.
 *           This is used to string together things to write with @c << operators
 */

emstream& operator << (emstream& serpt, adc& a2d)
{
	serpt << bin << PMS ("ADMUX : ") << ADMUX << PMS (" ADCSRA : ") << ADCSRA
	 << dec << endl;
	return (serpt);
}

