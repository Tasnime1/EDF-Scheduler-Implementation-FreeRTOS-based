/*MAP 0x000000000, 0x0000FFFF READ WRITE EXEC
 * 80000 of looping i=i ->exactly 12 mseconds, 37000 = 5.6 ms
 * FreeRTOS V202212.00
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
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
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
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
//#include <lpc21xx.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Demo application includes. */
#include "partest.h"
#include "flash.h"
#include "comtest2.h"
#include "serial.h"
#include "PollQ.h"
#include "BlockQ.h"
#include "semtest.h"
#include "dynamic.h"
#include "GPIO.h"
#include "GPIO_cfg.h"
#include <LPC21xx.H>                     /* LPC21xx definitions               */

/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainTX_ENABLE		( ( unsigned long ) 0x00010000 )	/* UART1. */
#define mainRX_ENABLE		( ( unsigned long ) 0x00040000 ) 	/* UART1. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )
#define mainLED_TO_OUTPUT	( ( unsigned long ) 0xff0000 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )
#define mainCOM_TEST_LED		( 3 )

/* Priorities for the demo application tasks. */
#define mainLED_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define mainCOM_TEST_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define mainQUEUE_POLL_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define mainBLOCK_Q_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define mainSEM_TEST_PRIORITY		( tskIDLE_PRIORITY + 1 )
#define mainCHECK_TASK_PRIORITY		( tskIDLE_PRIORITY + 3 )

/* Constants used by the "check" task.  As described at the head of this file
the check task toggles an LED.  The rate at which the LED flashes is used to
indicate whether an error has been detected or not.  If the LED toggles every
3 seconds then no errors have been detected.  If the rate increases to 500ms
then an error has been detected in at least one of the demo application tasks. */
#define mainCHECK_LED				( 7 )
#define mainNO_ERROR_FLASH_PERIOD	( ( TickType_t ) 3000 / portTICK_PERIOD_MS  )
#define mainERROR_FLASH_PERIOD		( ( TickType_t ) 500 / portTICK_PERIOD_MS  )

/*-----------------------------------------------------------*/

/*
 * Checks that all the demo application tasks are still executing without error
 * - as described at the top of the file.
 */
static long prvCheckOtherTasksAreStillRunning( void );

/*
 * The task that executes at the highest priority and calls
 * prvCheckOtherTasksAreStillRunning().  See the description at the top
 * of the file.
 */
static void vErrorChecks( void *pvParameters );

/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );

/*-----------------------------------------------------------*/
/*Tasks' prototypes*/
void Button_1_Monitor (void * pvParameters); //3.6 microSec
void Button_2_Monitor (void * pvParameters); //7.76 microSec
void Periodic_Transmitter (void * pvParameters);
void Uart_Receiver (void * pvParameters);
void Load_1_Simulation (void * pvParameters);
void Load_2_Simulation (void * pvParameters);

/* Buttons' State */
char Button1_State[50] = "";
char Button2_State[50] = "";

/*UART Initialization*/
void Uart_Init(void)
 {
   U0LCR=0X83;                           /* line control registor                                     */
   U0DLL=0XC3;                           /* baud rate registor                                        */
   U0DLM=0X00;                           /* baud rate registor                                        */
   U0LCR=0X03;                           /* line control registor                                     */
 } 
 
/******************************************************************************************************
* Function    : Uart_Data
*
* Description : UART_0 data transmission function
*               
* Parameter   : data
******************************************************************************************************/
void Uart_Data(unsigned char data )
 {
   U0THR = data;
   while((U0LSR & 0X20)!= 0X20);
 }
 
/******************************************************************************************************
* Function    : Uart_String
*
* Description : UART_0 group of data transmission function
*               
* Parameter   : dat
******************************************************************************************************/
void Uart_String(char data[])
 {
    while(*data!='\0') 
    {
         Uart_Data(*data);
         data++;
    }
 }
 
/*******************************************************************************************************
*                                PORT INITIALIZATION FUNCTION                                          *
*******************************************************************************************************/
void Port_Initial(void)    
 {
    PINSEL0  =  0x00000005;
 } 


/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler.
 */
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();		
	
	/*Initialize UART*/
	Port_Initial();                         /* Function to initialize the ports                       */
  Uart_Init();                            /* Initialization of Uart0                                */
	
	/*Tasks' Creation*/
	xTaskCreate(
                    Button_1_Monitor,         /* Function that implements the task. */
                    "Button_1_Monitor",        /* Text name for the task. */
                    configMINIMAL_STACK_SIZE,           /* Stack size in words, not bytes. */
                    ( void * ) NULL,  /* Parameter passed into the task. */
                    1 | portPRIVILEGE_BIT,						 /* Priority at which the task is created. */
                    NULL /* Used to pass out the created task's handle. */
										); 

	xTaskCreate(
                    Button_2_Monitor,         /* Function that implements the task. */
                    "Button_2_Monitor",        /* Text name for the task. */
                    configMINIMAL_STACK_SIZE,           /* Stack size in words, not bytes. */
                    ( void * ) NULL,  /* Parameter passed into the task. */
                    1 | portPRIVILEGE_BIT,						 /* Priority at which the task is created. */
                    NULL /* Used to pass out the created task's handle. */
										); 
	
	xTaskCreate(
                    Periodic_Transmitter,         /* Function that implements the task. */
                    "Periodic_Transmitter",        /* Text name for the task. */
                    configMINIMAL_STACK_SIZE,           /* Stack size in words, not bytes. */
                    ( void * ) NULL,  /* Parameter passed into the task. */
                    1 | portPRIVILEGE_BIT,						 /* Priority at which the task is created. */
                    NULL /* Used to pass out the created task's handle. */
										);
										
  xTaskCreate(
                    Uart_Receiver,         /* Function that implements the task. */
                    "Uart_Receiver",        /* Text name for the task. */
                    configMINIMAL_STACK_SIZE,           /* Stack size in words, not bytes. */
                    ( void * ) NULL,  /* Parameter passed into the task. */
                    1 | portPRIVILEGE_BIT,						 /* Priority at which the task is created. */
                    NULL /* Used to pass out the created task's handle. */
										); 									
	xTaskCreate(
                    Load_1_Simulation,         /* Function that implements the task. */
                    "Load_1_Simulation",        /* Text name for the task. */
                    configMINIMAL_STACK_SIZE,           /* Stack size in words, not bytes. */
                    ( void * ) NULL,  /* Parameter passed into the task. */
                    1 | portPRIVILEGE_BIT,						 /* Priority at which the task is created. */
                    NULL /* Used to pass out the created task's handle. */
										);
	xTaskCreate(
                    Load_2_Simulation,         /* Function that implements the task. */
                    "Load_2_Simulation",        /* Text name for the task. */
                    configMINIMAL_STACK_SIZE,           /* Stack size in words, not bytes. */
                    ( void * ) NULL,  /* Parameter passed into the task. */
                    1 | portPRIVILEGE_BIT,						 /* Priority at which the task is created. */
                    NULL /* Used to pass out the created task's handle. */
										);									
	
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

void vApplicationIdleHook( void )
{
	GPIO_write(PORT_1, PIN2, PIN_IS_HIGH);
	//GPIO_write(PORT_1, PIN2, PIN_IS_LOW);
}

void vApplicationTickHook( void )
{
	GPIO_write(PORT_1, PIN1, PIN_IS_HIGH);
	GPIO_write(PORT_1, PIN1, PIN_IS_LOW);
}

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
/*-----------------------------------------------------------*/

static void vErrorChecks( void *pvParameters )
{
TickType_t xDelayPeriod = mainNO_ERROR_FLASH_PERIOD;

	/* Parameters are not used. */
	( void ) pvParameters;

	/* Cycle for ever, delaying then checking all the other tasks are still
	operating without error.  If an error is detected then the delay period
	is decreased from mainNO_ERROR_FLASH_PERIOD to mainERROR_FLASH_PERIOD so
	the on board LED flash rate will increase.

	This task runs at the highest priority. */

	for( ;; )
	{
		/* The period of the delay depends on whether an error has been
		detected or not.  If an error has been detected then the period
		is reduced to increase the LED flash rate. */
		vTaskDelay( xDelayPeriod );

		if( prvCheckOtherTasksAreStillRunning() != pdPASS )
		{
			/* An error has been detected in one of the tasks - flash faster. */
			xDelayPeriod = mainERROR_FLASH_PERIOD;
		}

		/* Toggle the LED before going back to wait for the next cycle. */
		vParTestToggleLED( mainCHECK_LED );
	}
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure the UART1 pins.  All other pins remain at their default of 0. */
	PINSEL0 |= mainTX_ENABLE;
	PINSEL0 |= mainRX_ENABLE;

	/* LED pins need to be output. */
	IODIR1 = mainLED_TO_OUTPUT;

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/

static long prvCheckOtherTasksAreStillRunning( void )
{
long lReturn = pdPASS;

	/* Check all the demo tasks (other than the flash tasks) to ensure
	that they are all still running, and that none of them have detected
	an error. */
	if( xAreComTestTasksStillRunning() != pdPASS )
	{
		lReturn = pdFAIL;
	}

	if( xArePollingQueuesStillRunning() != pdTRUE )
	{
		lReturn = pdFAIL;
	}

	if( xAreBlockingQueuesStillRunning() != pdTRUE )
	{
		lReturn = pdFAIL;
	}

	if( xAreSemaphoreTasksStillRunning() != pdTRUE )
	{
		lReturn = pdFAIL;
	}

	if( xAreDynamicPriorityTasksStillRunning() != pdTRUE )
	{
		lReturn = pdFAIL;
	}

	return lReturn;
}
/*-----------------------------------------------------------*/

void Button_1_Monitor (void * pvParameters)
{
	for(;;)
	{
		GPIO_write(PORT_1, PIN3, PIN_IS_HIGH);
		if(GPIO_read(PORT_0, PIN0) == PIN_IS_HIGH)
		{
			strcpy(Button1_State, "Button 1 has a rising edge on it");
		}
		else
		{
			strcpy(Button1_State, "Button 1 has a falling edge on it");
		}
		GPIO_write(PORT_1, PIN3, PIN_IS_LOW);
		
		vTaskDelay(200);
		GPIO_write(PORT_1, PIN2, PIN_IS_LOW);
	}
}

void Button_2_Monitor (void * pvParameters)
{
	for(;;)
	{
		GPIO_write(PORT_1, PIN4, PIN_IS_HIGH);
		
		if(GPIO_read(PORT_0, PIN1) == PIN_IS_HIGH)
		{
			strcpy(Button2_State, "Button 2 has a rising edge on it");
		}
		else
		{
			strcpy(Button2_State, "Button 2 has a falling edge on it");
		}
		
		GPIO_write(PORT_1, PIN4, PIN_IS_LOW);
		vTaskDelay(200);
		GPIO_write(PORT_1, PIN2, PIN_IS_LOW);
	}
}

void Periodic_Transmitter (void * pvParameters)
{
	for(;;)
	{
		GPIO_write(PORT_1, PIN5, PIN_IS_HIGH);
		
		Uart_String(Button1_State);
		Uart_Data('\n');
		Uart_String(Button2_State);
		Uart_Data('\n');
		
		GPIO_write(PORT_1, PIN5, PIN_IS_LOW);
		vTaskDelay(200);
		GPIO_write(PORT_1, PIN2, PIN_IS_LOW);
	}
}

void Uart_Receiver (void * pvParameters)
{
	for(;;)
	{
		GPIO_write(PORT_1, PIN6, PIN_IS_HIGH);
		//Receive data on UART and write it to UART again	
		if((U0LSR&0X01)==0X01)
    {
     U0THR  = U0RBR;
     while((U0LSR&0X20)!=0X20);
    }
		
		GPIO_write(PORT_1, PIN6, PIN_IS_LOW);
		vTaskDelay(200);
		GPIO_write(PORT_1, PIN2, PIN_IS_LOW);
	}
}

void Load_1_Simulation (void * pvParameters)
{
	for(;;)
	{	
		int i;
		
		GPIO_write(PORT_1, PIN7, PIN_IS_HIGH);
		for(i=0; i<37000; i++)
		{
			i=i;
		}
		GPIO_write(PORT_1, PIN7, PIN_IS_LOW);
		vTaskDelay(200);
		GPIO_write(PORT_1, PIN2, PIN_IS_LOW);
	}
}

void Load_2_Simulation (void * pvParameters)
{
	for(;;)
	{	
		int i;
		
		GPIO_write(PORT_1, PIN8, PIN_IS_HIGH);
		for(i=0; i<80000; i++)
		{
			i=i;
		}
		GPIO_write(PORT_1, PIN8, PIN_IS_LOW);
		vTaskDelay(200);
		GPIO_write(PORT_1, PIN2, PIN_IS_LOW);
	}
}
