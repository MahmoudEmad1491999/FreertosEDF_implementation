/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>
#define configUSE_EDF_SCHEDULAR 1

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "lpc21xx.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"


/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )

#define Button_1_Monitor_Frequency 50
#define Button_2_Monitor_Frequency 50
#define Periodic_Transmitter_Frequency 100
#define Uart_Receiver_Frequency 20
#define Load_1_Frequency 10
#define Load_2_Frequency 100
#define Load_1_Capacity 5
#define Load_2_Capacity 12

#define IDLE_TASK_PEROID 1000

#define NUMBER_OF_MESSEGES 10
#define MESSEGE_MAX_LENGTH 30
// through gpio signal we have 1ms for nearly every 1540 iteration;
#define X_ms(cap_ms) 2222 * cap_ms

typedef enum {
	BUTTON1PRESSED,
	BUTTON2PRESSED,
	PERIODIC_TRANSMITTER_TIMEOUT
} Event;

typedef enum 
{
	BUTTON_IS_HIGH,
	BUTTON_IS_LOW
}Button_State;

const char* EventMesseges[3][MESSEGE_MAX_LENGTH] = {
	"Button1Pressed",
	"Button2Pressed",
	"PeriodicTransmitterTimeout",
};


QueueHandle_t messegeQueue;
void Button_1_Monitor_Task(void* pvParameters);
void Button_2_Monitor_Task(void* pvParameters);
void Periodic_Transmitter(void* pvParameters);
void Uart_Receiver(void* pvParameters);
void Load_1(void* pvParameters);
void Load_2(void* pvParameters);

/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/

/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */


int main( void )
{
	
	messegeQueue = xQueueCreate(NUMBER_OF_MESSEGES, sizeof(char[30]));
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();
	
	/* Create Tasks here */
	xTaskCreatePeriodic(Button_1_Monitor_Task, "Button_1_Monitor_Task", configMINIMAL_STACK_SIZE, NULL, 1, NULL,Button_1_Monitor_Frequency);
	xTaskCreatePeriodic(Button_2_Monitor_Task, "Button_2_Monitor_Task", configMINIMAL_STACK_SIZE, NULL, 1, NULL,Button_2_Monitor_Frequency);
	xTaskCreatePeriodic(Periodic_Transmitter, "Periodic_Transmitter", configMINIMAL_STACK_SIZE, NULL, 1, NULL,Periodic_Transmitter_Frequency);
	xTaskCreatePeriodic(Uart_Receiver, "Uart_Receiver", configMINIMAL_STACK_SIZE, NULL, 1, NULL,Uart_Receiver_Frequency);
	xTaskCreatePeriodic(Load_1, "Load_1", configMINIMAL_STACK_SIZE, NULL, 1, NULL,Load_1_Frequency);
	xTaskCreatePeriodic(Load_2, "Load_2", configMINIMAL_STACK_SIZE, NULL, 1, NULL,Load_2_Frequency);
	
	/* this part is to determine the required number or iterations to get 1ms
	GPIO_write(PORT_0, PIN0, PIN_IS_HIGH);
	
	for( temp = 0; temp < 1000000; temp++);
	
	GPIO_write(PORT_0, PIN0, PIN_IS_LOW);
	
	for( temp = 0; temp < 1000000; temp++);
	*/
	/* Now all the tasks have been started - start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

/* Function to reset timer 1 */
void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}

/* Function to initialize and start timer 1 */
static void configTimer1(void)
{
	T1PR = 1000;
	T1TCR |= 0x1;
}

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();
	
	/* Config trace timer 1 and read T1TC to get current tick */
	configTimer1();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/
void vApplicationTickHook(void)
{
	static volatile uint8_t temp = 0;
	GPIO_write(PORT_0, PIN0, PIN_IS_HIGH);
	for(temp = 0; temp < 128; temp++);
	GPIO_write(PORT_0, PIN0, PIN_IS_LOW);
	return;
}


void Button_1_Monitor_Task(void* pvParameters)
{
	TickType_t xLastTimeWaken = xTaskGetTickCount();
	static Button_State previousState = BUTTON_IS_LOW;
	static volatile uint8_t temp = 0;
	for(;;)
	{
		// this part contribute to the calculation of the cpu load for this task.
		GPIO_write(PORT_0, PIN1, PIN_IS_HIGH);
		for(temp = 0; temp < 128; temp++);
		GPIO_write(PORT_0, PIN1, PIN_IS_LOW);
		
		// actual task.
		if(previousState == BUTTON_IS_LOW)	// if the previous state was low
		{
			if(GPIO_read(PORT_1, PIN1) == PIN_IS_HIGH) // read the pin to see if the button is raised to high.
			{
				previousState = BUTTON_IS_HIGH;
			}
		}
		else	// if the previous state was high.
		{
			if(GPIO_read(PORT_1, PIN1) == PIN_IS_LOW) // check the pin to see if the button is released to low.
			{
				previousState = BUTTON_IS_LOW;
				// send the event messege to the consumer task through the Queue
				// Note: the task will not block if the queue is already full.
				
				if(xQueueSend(messegeQueue,&(EventMesseges[BUTTON1PRESSED]), 0) != pdPASS)
				{
					// this part should not be reached if the messege was sent successfully.
					while(1);
				}
			}
		}
		// actual task end.
		
		// this part is to measure the cpu load of this task.
		GPIO_write(PORT_0, PIN1, PIN_IS_HIGH);
		for(temp = 0; temp < 128; temp++);
		GPIO_write(PORT_0, PIN1, PIN_IS_LOW);
		
		
		vTaskDelayUntil(&xLastTimeWaken, Button_1_Monitor_Frequency);
	}
}

void Button_2_Monitor_Task(void* pvParameters)
{
	TickType_t xLastTimeWaken = xTaskGetTickCount();
	static Button_State previousState = BUTTON_IS_LOW;
	
	static volatile uint8_t temp = 0;
	for(;;)
	{
		// this part contribute to the calculation of the cpu load for this task.
		GPIO_write(PORT_0, PIN2, PIN_IS_HIGH);
		for(temp = 0; temp < 128; temp++);
		GPIO_write(PORT_0, PIN2, PIN_IS_LOW);
		
		if(previousState == BUTTON_IS_LOW) // if the previous state was low
		{
			if(GPIO_read(PORT_1, PIN2) == PIN_IS_HIGH) // read the pin to see if the button is raised to high.
			{
				previousState = BUTTON_IS_HIGH;
			}
		}
		else // if the previous state was high.
		{
			if(GPIO_read(PORT_1, PIN2) == PIN_IS_LOW) // check the pin to see if the button is released to low.
			{
				previousState = BUTTON_IS_LOW;
				// send the event messege to the consumer task through the Queue
				// Note: the task will not block if the queue is already full.
				
				if(xQueueSend(messegeQueue,&(EventMesseges[BUTTON2PRESSED]), 0) != pdPASS)
				{
					// this part should not be reached if the messege was sent successfully.
					while(1);
				}
			}
		}
		
		// this part is to measure the cpu load of this task.
		GPIO_write(PORT_0, PIN2, PIN_IS_HIGH);
		for(temp = 0; temp < 128; temp++);
		GPIO_write(PORT_0, PIN2, PIN_IS_LOW);
		
		vTaskDelayUntil(&xLastTimeWaken, Button_2_Monitor_Frequency);
	}
}

void Periodic_Transmitter(void* pvParameters)
{
	TickType_t xLastTimeWaken = xTaskGetTickCount();
	static volatile uint8_t temp = 0;
	for(;;)
	{
		GPIO_write(PORT_0, PIN3, PIN_IS_HIGH);
		for(temp = 0; temp < 128; temp++);
		GPIO_write(PORT_0, PIN3, PIN_IS_LOW);
		
		
		if(xQueueSend(messegeQueue,&(EventMesseges[PERIODIC_TRANSMITTER_TIMEOUT]), 0) != pdPASS)
		{
					// this part should not be reached if the messege was sent successfully.
					while(1);
		}
		
		
		GPIO_write(PORT_0, PIN3, PIN_IS_HIGH);
		for(temp = 0; temp < 128; temp++);
		GPIO_write(PORT_0, PIN3, PIN_IS_LOW);
		
		
		vTaskDelayUntil(&xLastTimeWaken, Periodic_Transmitter_Frequency);
	}
}

void Uart_Receiver(void* pvParameters)
{
	TickType_t xLastTimeWaken = xTaskGetTickCount();
	char buffer[MESSEGE_MAX_LENGTH];
	static volatile uint8_t temp = 0;
	for(;;)
	{
		GPIO_write(PORT_0, PIN4, PIN_IS_HIGH);
		for(temp = 0; temp < 128; temp++);
		GPIO_write(PORT_0, PIN4, PIN_IS_LOW);
		
		while(xQueueReceive(messegeQueue, &buffer, 0) == pdPASS)
		{
			vSerialPutString((signed char*)buffer, MESSEGE_MAX_LENGTH);
		}
		
		
		GPIO_write(PORT_0, PIN4, PIN_IS_HIGH);
		for(temp = 0; temp < 128; temp++);
		GPIO_write(PORT_0, PIN4, PIN_IS_LOW);
		
		vTaskDelayUntil(&xLastTimeWaken, Uart_Receiver_Frequency);
	}
}



void Load_1(void* pvParameters)
{
	TickType_t xLastTimeWaken = xTaskGetTickCount();
	static volatile uint64_t load_iterations_counter = 0;
	static volatile uint8_t temp = 0;
	for(;;)
	{
		GPIO_write(PORT_0, PIN5, PIN_IS_HIGH);
		for(temp = 0; temp < 128; temp++);
		GPIO_write(PORT_0, PIN5, PIN_IS_LOW);
		
		for(load_iterations_counter = 0; load_iterations_counter < X_ms(Load_1_Capacity); load_iterations_counter++);
		
		GPIO_write(PORT_0, PIN5, PIN_IS_HIGH);
		for(temp = 0; temp < 128; temp++);
		GPIO_write(PORT_0, PIN5, PIN_IS_LOW);
		
		vTaskDelayUntil(&xLastTimeWaken, Load_1_Frequency);
	}
}
void Load_2(void* pvParameters)
{
	TickType_t xLastTimeWaken = xTaskGetTickCount();
	static volatile uint64_t load_iterations_counter = 0;
	static volatile uint8_t temp = 0;
	for(;;)
	{
		GPIO_write(PORT_0, PIN6, PIN_IS_HIGH);
		for(temp = 0; temp < 128; temp++);
		GPIO_write(PORT_0, PIN6, PIN_IS_LOW);
		
		for(load_iterations_counter = 0; load_iterations_counter < X_ms(Load_2_Capacity); load_iterations_counter++);
		
		GPIO_write(PORT_0, PIN6, PIN_IS_HIGH);
		for(temp = 0; temp < 128; temp++);
		GPIO_write(PORT_0, PIN6, PIN_IS_LOW);
		
		vTaskDelayUntil(&xLastTimeWaken, Load_2_Frequency);
	}
}

