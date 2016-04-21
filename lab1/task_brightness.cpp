//**************************************************************************************
/** @file task_brightness.cpp
 *    This file contains the code for a task class which controls the brightness of an
 *    LED using a voltage measured from the A/D as input. The fun part: the brightness
 *    that is being controlled can be on another AVR computer, with signals being sent
 *    and received via wireless transceivers. 
 *
 *  Revisions:
 *    @li 09-30-2012 JRR Original file was a one-file demonstration with two tasks
 *    @li 10-05-2012 JRR Split into multiple files, one for each task
 *    @li 10-25-2012 JRR Changed to a more fully C++ version with class task_sender
 *    @li 10-27-2012 JRR Altered from data sending task into LED blinking class
 *    @li 11-04-2012 JRR Altered again into the multi-task monstrosity
 *    @li 12-13-2012 JRR Yet again transmogrified; now it controls LED brightness
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
//**************************************************************************************

#include "textqueue.h"                      // Header for text queue class
#include "task_brightness.h"                // Header for this task
#include "shares.h"                         // Shared inter-task communications


//-------------------------------------------------------------------------------------
/** This constructor creates a task which controls the brightness of an LED using
 *  input from an A/D converter. The main job of this constructor is to call the
 *  constructor of parent class (\c frt_task ); the parent's constructor the work.
 *  @param a_name A character string which will be the name of this task
 *  @param a_priority The priority at which this task will initially run (default: 0)
 *  @param a_stack_size The size of this task's stack in bytes 
 *                      (default: configMINIMAL_STACK_SIZE)
 *  @param p_ser_dev Pointer to a serial device (port, radio, SD card, etc.) which can
 *                   be used by this task to communicate (default: NULL)
 */

task_brightness::task_brightness (const char* a_name, 
								 unsigned portBASE_TYPE a_priority, 
								 size_t a_stack_size,
								 emstream* p_ser_dev
								)
	: TaskBase (a_name, a_priority, a_stack_size, p_ser_dev)
{
	// Nothing is done in the body of this constructor. All the work is done in the
	// call to the frt_task constructor on the line just above this one
}


//-------------------------------------------------------------------------------------
/** This method is called once by the RTOS scheduler. Each time around the for (;;)
 *  loop, it reads the A/D converter and uses the result to control the brightness of 
 *  an LED. 
 */

void task_brightness::run (void)
{
	// Make a variable which will hold times to use for precise task scheduling
	TickType_t previousTicks = xTaskGetTickCount ();

	// Create an analog to digital converter driver object and a variable in which to
	// store its output. The variable p_my_adc only exists within this run() method,
	// so the A/D converter cannot be used from any other function or method
	adc* p_my_adc = new adc (p_serial);
	//Constructs new motor for motor 1 on ATmega board
	//motor_drv* p_motor_1 = new motor_drv(p_serial, &PORTC, 0, &PORTB, 6, &TCCR1A, &OCR1B);
	//Constructs new motor for motor2 on ATmega board
 	motor_drv* p_motor_1 = new motor_drv(p_serial, &PORTD, 5, &PORTB, 5, &TCCR1A,  &OCR1A);


	// This is the task loop for the brightness control task. This loop runs until the
	// power is turned off or something equally dramatic occurs
	for (;;)
	{
		// Read the A/D converter
		int16_t a2d_reading = p_my_adc->read_oversampled(0,10);//read_once (0);

		// Convert the A/D reading into a PWM duty cycle. The A/D reading is between 0
		// and 1023; the duty cycle should be between 0 and 255. Thus, divide by 4
		int16_t torque; 
	
		if (a2d_reading < 425) 
		{
		   torque = (255 - (a2d_reading / 5) * 3);
		   torque = torque * -1;
		}
		else if (a2d_reading > 598) 
		{
		   a2d_reading = a2d_reading - 598 ;
		   torque = (a2d_reading / 5) * 3;
		}
		else 
		{
		   torque = 0;  
		}
		// Set the brightness. Since the PWM has already been set up, we only need to
		// put a new value into the duty cycle control register, which on an AVR is 
		// the output compare register for a given timer/counter

		// Increment the run counter. This counter belongs to the parent class and can
		// be printed out for debugging purposes
		runs++;
		// /*
		// debug printing code for testing the output
		if ((runs%10) == 0)
		{
		   *p_serial << "PORTD : " << PORTD << endl;
   		   *p_serial << "Duty Cycle : " << OCR1A << endl;
		    p_motor_1->set_power(torque);   //Calls set_power on motor 1
		   *p_serial << "Torque : " << torque << endl;
		   *p_serial << "Duty Cycle : " << OCR1A << endl;
		   *p_serial << endl;
		}
		// */
		// This is a method we use to cause a task to make one run through its task
		// loop every N milliseconds and let other tasks run at other times
		delay_from_for_ms (previousTicks, 100);
	}
}

