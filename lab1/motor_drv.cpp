/*************************************************************************************
/** @file motor_drv.cpp
 *    This file contains a very generic motor driver. 
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
#include "motor_drv.h"
//-------------------------------------------------------------------------------------
/** \brief This constructor sets up a motor driver
 *  \details The motor driver is initialized so that the board sends signals to the motor chip
 *  @param p_serial_port A pointer to the serial port which writes debugging info. 
 */

motor_drv::motor_drv(emstream* p_serial_port, 
		     volatile uint8_t* port_select,      uint8_t port_pin_offset, 
		     volatile uint8_t* pwm_select,       uint8_t pwm_pin_select,
		     u_int8_t TCCRNO);
{      
   ptr_to_serial = p_serial_port;             
   volatile uint8_t* p_motor_port = port_select;        //Pointer to port on ATmega that connects to motor driver
   volatile uint8_t* p_motor_dir = (p_motor_port - 1);  //Pointer to I/O register of port
   volatile uint8_t  pin_shift = port_bitshift ;        //Sets the pin shift for INA, INB, DIAG pins
   volatile uint8_t* p_pwm = pwm_select;                //Pointer to the PWM Port on ATmega
   volatile uint8_t* p_pwm_dir = (p_pwm - 1);           //Pointer to the PWM direction register
   volatile uint8_t OC_XX = pwm_pin_select;             //Selects pin for OCXX (OC1A)
   
   *p_motor_port &= ~(0b011 << offset);          //Sets INA, INB to 0, enables DIAG (pull up resistor;
   *p_motor_dir |= (0b111 << offset);            //Sets INA, INB to outputs, DIAG to input
    
   *p_pwm_dir &= ~(0x01 << 0C_XX);		//Sets OCXX low
   *p_pwm |= (0x1 < 0C_XX);                      //Sets OCXX to output
   TCCRNO = (1 << WGM13) | (1 << WGM12) | (1 << WGM11);
}

  
motor_drv:: set_power(int16_t torque, u_int16_t OCRNO) 
{
   u_int8_t duty_cycle; 
   
   //Checks for a postitive input, which indicates forward motion
   if(torque < 0)
   {
      *p_port &= ~(0b11 << offset);    // clears INA INB
      *p_port |= (0b1 << offset);      // sets INA high
   }
   //Checks for negative input, which indicated reverse motion
   else if(torque > 0)
   { 
      *p_port &= ~(0b11 << offset);     // clears INA INB
      *p_port |= (0x10 << offset);     // sets INB hight
   } 
   //Zero input inidcates no motion
   else
   {
      *p_port &= ~(0b11 << offset);     //clears INA INB
   }
    
   duty_cycle = abs(torque);
   
   switch (OCRNO) 
   {   
      case (OCR1A):
        OCR1A = duty_cycle;
        break;
	
      case (OCR1B):
        OCR1B = duty_cycle;
	break;

      case (OCR3A):
        OCR3A = duty_cycle;
	break;

      case (OCR3B):
	OCR3B = duty_cycle;
	break;
	
      case (OCR5A):
	OCR5A = duty_cycle;
	break;
        
      case (OCR5B):
	OCR5B = duty_cycle;
	break;
   }
}

motor_drv:: brake(int16_t strength, u_int16_t OCRNO)
{
   set_power(0, OCRNO);
}
