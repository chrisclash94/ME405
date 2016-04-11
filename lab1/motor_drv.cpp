/*************************************************************************************
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
#include "motor_drv.h"
//-------------------------------------------------------------------------------------
/** \brief This constructor sets up a motor driver
 *  \details The motor driver is initialized so that the board sends signals to the motor chip
 *  @param p_serial_port A pointer to the serial port which writes debugging info. 
 */

motor_drv::motor_drv(emstream* p_serial_port, volatile uint8_t* port_select, volatile uint8_t* pwm_select, uint8_t oc_select) 
{
   ptr_to_serial = p_serial_port;
   volatile uint8_t* p_port = port_select;
   volatile uint8_t* p_pwm = pwm_select;
   
}

motor_drv:: set_power(int16_t torqe) 
{
   //Checks for a postitive input, which indicates forward motion
   if(torque < 0)
   {
      INA |= 0x01;
      INB |= 0x00;
       
   }
   //Checks for negative input, which indicated reverse motion
   else if(torque > 0)
   {
       INA = 0x01;
       INB = 0x00;
   }
   //Zero input inidcates no motion
   else
   {
       INA = 0x00;
       INB = 0x00;
   }
  
}

 motor_drv:: brake(int16_t strength)
{
  
  
}
