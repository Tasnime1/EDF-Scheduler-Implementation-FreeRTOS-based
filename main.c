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
#include <LPC21xx.H>                   

/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainTX_ENABLE		( ( unsigned long ) 0x00010000 )	/* UART1. */
#define mainRX_ENABLE		( ( unsigned long ) 0x00040000 ) 	/* UART1. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )
#define mainLED_TO_OUTPUT	( ( unsigned long ) 0xff0000 )

/*UART*/
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )
xComPortHandle xPort;;

/*-----------------------------------------------------------*/

/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );

/*-----------------------------------------------------------*/
/*Tasks' prototypes*/
void Button_1_Monitor (void * pvParameters); //17 milliseconds
void Button_2_Monitor (void * pvParameters); //17 milliseconds
void Periodic_Transmitter (void * pvParameters); //17.6 milliseconds
void Uart_Receiver (void * pvParameters); //0.85 microSec
void Load_1_Simulation (void * pvParameters); //requirement: 5 milliseconds
void Load_2_Simulation (void * pvParameters); //requirement: 12 milliseconds



/*Total CPU Load Variables' Calculation*/
int tasks_total_time=0, task_in_time=0, task_out_time=0;
int system_time=0;
float cpu_load=0;



/*********************************************************************************************************
 *                                  Application entry point:                                             *
 *                     Starts all the other tasks, then starts the scheduler.                            *
 ********************************************************************************************************/
int main( void )

{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();		
	

	/*Tasks' Creation*/
	/*Tasks' Creation*/
	xTaskPeriodicCreate(
                    Button_1_Monitor,         /* Function that implements the task. */
                    "Button_1_Monitor",        /* Text name for the task. */
                    configMINIMAL_STACK_SIZE,           /* Stack size in words, not bytes. */
                    ( void * ) NULL,  /* Parameter passed into the task. */
                    1 | portPRIVILEGE_BIT,						 /* Priority at which the task is created. */
                    NULL, /* Used to pass out the created task's handle. */
										50); 

	xTaskPeriodicCreate(
                    Button_2_Monitor,         /* Function that implements the task. */
                    "Button_2_Monitor",        /* Text name for the task. */
                    configMINIMAL_STACK_SIZE,           /* Stack size in words, not bytes. */
                    ( void * ) NULL,  /* Parameter passed into the task. */
                    1 | portPRIVILEGE_BIT,						 /* Priority at which the task is created. */
                    NULL, /* Used to pass out the created task's handle. */
										50); 
	
	xTaskPeriodicCreate(
                    Periodic_Transmitter,         /* Function that implements the task. */
                    "Periodic_Transmitter",        /* Text name for the task. */
                    configMINIMAL_STACK_SIZE,           /* Stack size in words, not bytes. */
                    ( void * ) NULL,  /* Parameter passed into the task. */
                    1 | portPRIVILEGE_BIT,						 /* Priority at which the task is created. */
                    NULL, /* Used to pass out the created task's handle. */
										100);
										
  xTaskPeriodicCreate(
                    Uart_Receiver,         /* Function that implements the task. */
                    "Uart_Receiver",        /* Text name for the task. */
                    configMINIMAL_STACK_SIZE,           /* Stack size in words, not bytes. */
                    ( void * ) NULL,  /* Parameter passed into the task. */
                    1 | portPRIVILEGE_BIT,						 /* Priority at which the task is created. */
                    NULL, /* Used to pass out the created task's handle. */
										20);
										
	xTaskPeriodicCreate(
                    Load_1_Simulation,         /* Function that implements the task. */
                    "Load_1_Simulation",        /* Text name for the task. */
                    configMINIMAL_STACK_SIZE,           /* Stack size in words, not bytes. */
                    ( void * ) NULL,  /* Parameter passed into the task. */
                    1 | portPRIVILEGE_BIT,						 /* Priority at which the task is created. */
                    NULL, /* Used to pass out the created task's handle. */
										10);
										
	xTaskPeriodicCreate(
                    Load_2_Simulation,         /* Function that implements the task. */
                    "Load_2_Simulation",        /* Text name for the task. */
                    configMINIMAL_STACK_SIZE,           /* Stack size in words, not bytes. */
                    ( void * ) NULL,  /* Parameter passed into the task. */
                    1 | portPRIVILEGE_BIT,						 /* Priority at which the task is created. */
                    NULL, /* Used to pass out the created task's handle. */
										100);	


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
	GPIO_write(PORT_1, PIN1, PIN_IS_HIGH);
	//GPIO_write(PORT_1, PIN1, PIN_IS_LOW);
}

void vApplicationTickHook( void )
{
	GPIO_write(PORT_1, PIN8, PIN_IS_HIGH);
	GPIO_write(PORT_1, PIN8, PIN_IS_LOW);
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

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */
	
	xPort = xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE, 30);
	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
	
	configTimer1();
	GPIO_init();
}
/*-----------------------------------------------------------*/

void Button_1_Monitor (void * pvParameters)
{
	vTaskSetApplicationTaskTag( NULL, ( void * ) 1 );
	for(;;)
	{
		TickType_t lastWakeTime = xTaskGetTickCount();
		
		if(GPIO_read(PORT_0, PIN0) == PIN_IS_HIGH)
		{
			vSerialPutString(xPort , (signed char*)"Button 1 has a falling edge on it\n", 35);
		}
		else
		{
			vSerialPutString(xPort, (signed char*)"Button 1 has a rising edge on it\n", 34);
		}
		
		GPIO_write(PORT_1, PIN2, PIN_IS_LOW);
		vTaskDelayUntil(&lastWakeTime, 50);
		GPIO_write(PORT_1, PIN1, PIN_IS_LOW);
		GPIO_write(PORT_1, PIN2, PIN_IS_HIGH);
	}
}

void Button_2_Monitor (void * pvParameters)
{
	vTaskSetApplicationTaskTag( NULL, ( void * ) 2 );
	for(;;)
	{
		TickType_t lastWakeTime = xTaskGetTickCount();
		//GPIO_write(PORT_1, PIN3, PIN_IS_HIGH);
		
		if(GPIO_read(PORT_0, PIN0) == PIN_IS_HIGH)
		{
			vSerialPutString(xPort , (signed char*)"Button 2 has a falling edge on it\n", 35);
		}
		else
		{
			vSerialPutString(xPort, (signed char*)"Button 2 has a rising edge on it\n", 34);
		}
		
		GPIO_write(PORT_1, PIN3, PIN_IS_LOW);
		vTaskDelayUntil(&lastWakeTime, 50);
		GPIO_write(PORT_1, PIN1, PIN_IS_LOW);
		GPIO_write(PORT_1, PIN3, PIN_IS_HIGH);
	}
}

void Periodic_Transmitter (void * pvParameters)
{
	vTaskSetApplicationTaskTag( NULL, ( void * ) 3 );
	for(;;)
	{
		TickType_t lastWakeTime = xTaskGetTickCount();
		
		//Sending a random string every 100ms as stated in the project
		vSerialPutString(NULL, "Random String from Transmitter Task", 36);
		
		GPIO_write(PORT_1, PIN4, PIN_IS_LOW);
		vTaskDelayUntil(&lastWakeTime, 100);
		GPIO_write(PORT_1, PIN1, PIN_IS_LOW);
		GPIO_write(PORT_1, PIN4, PIN_IS_HIGH);
	}
}

void Uart_Receiver (void * pvParameters)
{
	vTaskSetApplicationTaskTag( NULL, ( void * ) 4 );
	for(;;)
	{
		TickType_t lastWakeTime = xTaskGetTickCount();
		//GPIO_write(PORT_1, PIN5, PIN_IS_HIGH);
		
		//Receive data on UART and write it to UART again	
		if((U0LSR&0X01)==0X01) /* Checking for available data*/
    {
     U0THR  = U0RBR;			/*Writing data available to UART0 again*/
     while((U0LSR&0X20)!=0X20);
    }
		
		GPIO_write(PORT_1, PIN5, PIN_IS_LOW);
		vTaskDelayUntil(&lastWakeTime, 20);
		GPIO_write(PORT_1, PIN1, PIN_IS_LOW);
		GPIO_write(PORT_1, PIN5, PIN_IS_HIGH);
	}
}

void Load_1_Simulation (void * pvParameters)
{
	vTaskSetApplicationTaskTag( NULL, ( void * ) 5 );
	for(;;)
	{	
		TickType_t lastWakeTime = xTaskGetTickCount();
		int i;
		
		//GPIO_write(PORT_1, PIN6, PIN_IS_HIGH);
		for(i=0; i<37000; i++)
		{
			i=i;
		}
		GPIO_write(PORT_1, PIN6, PIN_IS_LOW);
		vTaskDelayUntil(&lastWakeTime, 10);
		GPIO_write(PORT_1, PIN1, PIN_IS_LOW);
		GPIO_write(PORT_1, PIN6, PIN_IS_HIGH);
	}
}

void Load_2_Simulation (void * pvParameters)
{
	vTaskSetApplicationTaskTag( NULL, ( void * ) 6 );
	for(;;)
	{	
		TickType_t lastWakeTime = xTaskGetTickCount();
		int i;
		
		//GPIO_write(PORT_1, PIN7, PIN_IS_HIGH);
		for(i=0; i<80000; i++)
		{
			i=i;
		}
		GPIO_write(PORT_1, PIN7, PIN_IS_LOW);
		vTaskDelayUntil(&lastWakeTime, 100);
		GPIO_write(PORT_1, PIN1, PIN_IS_LOW);
		GPIO_write(PORT_1, PIN7, PIN_IS_HIGH);
	}
}
