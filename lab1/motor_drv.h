//======================================================================================
/** @file motor_drv.h
 *    This file contains a very simple A/D converter driver. The driver is hopefully
 *    thread safe in FreeRTOS due to the use of a mutex to prevent its use by multiple
 *    tasks at the same time. There is no protection from priority inversion, however,
 *    except for the priority elevation in the mutex.
 *
 *  Revisions:
 *    @li 01-15-2008 JRR Original (somewhat useful) file
 *    @li 10-11-2012 JRR Less original, more useful file with FreeRTOS mutex added
 *    @li 10-12-2012 JRR There was a bug in the mutex code, and it has been fixed
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
#ifndef MOTOR_DRV_H
#define MOTOR_DRV_H

#include "emstream.h"                       // Header for serial ports and devices
#include "FreeRTOS.h"                       // Header for the FreeRTOS RTOS
#include "task.h"                           // Header for FreeRTOS task functions
#include "queue.h"                          // Header for FreeRTOS queues
#include "semphr.h"                         // Header for FreeRTOS semaphores

#define OC_1A 5  
#define OC_1B 6

/** @brief This class defines the motor driver
 *  @detials The class includes the motor constructor and the set_power (duty cycle set) 
 *  and brake functions. These are standard motor functions. 
 */ 
class motor_drv
{
         protected:
            
	    ///Pointer to the serial port
            emstream* ptr_to_serial;              
	    ///Pointer to port on ATmega that connects to motor driver
	    volatile uint8_t* p_port;             
            ///Pointer to I/O register of port
            volatile uint8_t* p_motor_dir;        
            ///Sets the pin shift for INA, INB, DIAG pins
            volatile uint8_t  pin_shift;        
            ///Pointer to the PWM Port on ATmega
            volatile uint8_t* p_pwm;               
            ///Pointer to the PWM direction register
            volatile uint8_t* p_pwm_dir;          
            ///Selects pin for OCXX (OC1A)
            volatile uint8_t  OC_XX;             
	    ///Pointer to the output register
	    volatile uint16_t* OCR;             
	    ///Pointer to pwm
	    volatile uint8_t* TCC;            
	    
	    
         public:
	   
	    //Sets the Ports of the ATmega to connect the motor driver chip
             motor_drv(emstream* p_serial_port, 
		     volatile uint8_t* port_select,      uint8_t port_pin_offset, 
		     volatile uint8_t* pwm_select,       uint8_t pwm_pin_select,
		     volatile uint8_t* TCCRNO,           volatile uint16_t* OCRNO);	     
	    //Set the duty cycle and direction of motor
             void set_power(int16_t torque); 	     
	     //Slows the motor until it reaches a stop
	     void brake();    
  
};

emstream& operator<< (emstream&, motor_drv&);

#endif

