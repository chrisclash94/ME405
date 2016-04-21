//*************************************************************************************
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
/** @brief This constructor sets up a motor driver
 *  @details The motor driver is initialized so that the board sends signals to a given motor chip
 *  @param  p_serial_port Pointer to the serial output port  
 *  @param  uint8_t* port_select pointer to port on ATmega that connects to motor driver
 *  @param  port_pin_offset Pointer to I/O register of port 
 *  @param  pwm_select Sets the pin shift for INA, INB, DIAG pins
 *  @param  pwm_pin_select Pointer to the PWM Port on ATmega
 *  @param  TCCRNO Pointer to the PWM direction register
 *  @param  OCRNO  Selects pin for OCXX (OC1A)
 */

motor_drv:: motor_drv(emstream* p_serial_port, 
		     volatile uint8_t* port_select,      uint8_t port_pin_offset, 
		     volatile uint8_t* pwm_select,       uint8_t pwm_pin_select,
		     volatile uint8_t* TCCRNO,           volatile uint16_t* OCRNO)
{      
    ptr_to_serial = p_serial_port;               
    p_port = port_select;              
    p_motor_dir = (p_port - 1);        
    pin_shift = port_pin_offset;       
    p_pwm = pwm_select;                
    p_pwm_dir = (p_pwm - 1);           
    OC_XX = pwm_pin_select;         
    OCR = OCRNO;
    
    *p_motor_dir &= ~(0b100 << pin_shift);   //Sets DIAG to input
    *p_motor_dir |= (0b11 << pin_shift);     //Sets INA INB to output
    
    *p_port &= ~(0b11 << pin_shift);         //Sets INA, INB to 0
    *p_port |= (0b100 << pin_shift);         //Turn on DIAG pullup resistor
    
    *p_pwm_dir |= (0x1 << OC_XX);            //Sets pwm to output          
    *p_pwm &= ~(0x1 << OC_XX);               //Sets OC1(A/B) low
    
    *p_serial_port << "TCCR1A : " << TCCR1A << endl;

    TCC = TCCRNO;//Sets OCXX to output
    *TCC = (1 << WGM10) | (1 << COM1A1) | (0 << COM1A0); 
    *p_serial_port << "TCCR1A : " << TCCR1A << endl;
    *p_serial_port << "TCCR1B : " << TCCR1B << endl;
    
    TCC = TCCRNO + 1;
    *TCC = (1 << WGM12) | (1 << CS11) | (1 << CS10);
    *p_serial_port << "TCCR1B : " << TCCR1B << endl;
  
}

//-------------------------------------------------------------------------------------
/** @brief This functions sets the "power" of the motor
 *  @details Sets the direction of the motor and runs at a speed according to the set duty cycle
 *  @param torque a signed valued that determines the direction and is passed as the duty cylce
 */  
void motor_drv:: set_power(int16_t torque)	      
{
   uint16_t duty_cycle; 
   
   //Checks for a postitive input, which indicates forward motion
   if(torque > 0)
   {
      *p_port &= ~(0b11 << pin_shift);    // clears INA INB
      *p_port |= (0b1 << pin_shift);      // sets INA high
   }
   //Checks for negative input, which indicated reverse motion
   else if(torque < 0)
   { 
      *p_port &= ~(0b11 << pin_shift);     // clears INA INB
      *p_port |= (0b10 << pin_shift);     // sets INB hight
   } 
   //Zero input inidcates no motion
   else
   {
      *p_port &= ~(0b11 << pin_shift);     //clears INA INB
   }
    
   duty_cycle = abs(torque);
   
   *OCR = duty_cycle;
}

//-------------------------------------------------------------------------------------
/** @brief This functions causes the motor to stop
 *  @details Calls the set power function with a value of 0 to force the motor to stop 
 */

void motor_drv:: brake()
{
   set_power(0);
}
