/* FreeRTOS includes. */
#include <asf.h>
#include "stdio_serial.h"
#include "conf_board.h"
#include "conf_clock.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/*-----------------------------------------------------------*/
//Define PINS
#define MY_INTERRUPT_A PIO_PC24_IDX //Pin6

/** PWM frequency in Hz */
#define PWM_FREQUENCY      1000
/** Period value of PWM output waveform */
#define PERIOD_VALUE       100
/** Initial duty cycle value */
#define INIT_DUTY_VALUE    0

/** PWM channel instance for motor */
pwm_channel_t g_pwm_channel_led0;
static uint32_t ul_count = 0;  /* PWM counter value */
static uint32_t led_0_ul_duty = INIT_DUTY_VALUE;  /* PWM duty cycle rate */
static uint8_t led0_fade_in = 1;  /* LED fade in flag */

//Create queue handle
xQueueHandle xQueue;

//Create semaphore handle
xSemaphoreHandle xCountSemaphore;

//Function Prototypes
static void configure_console(void);
void pin_edge_handler(const uint32_t id, const uint32_t index);

//RTOS Tasks
static void vHandlerTask( void *pvParameters );

int main( void )
{
	/* This function initializes the MCU clock */
	sysclk_init();
	/* Board initialization */
	board_init();
	ioport_init();

	//Set pin modes
	ioport_set_pin_dir(MY_INTERRUPT_A, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(MY_INTERRUPT_A, IOPORT_MODE_PULLUP);

	/* Initialize the serial I/O(console ) */
	configure_console();

	pmc_enable_periph_clk(ID_PIOC);
	pio_set_input(PIOC, PIO_PC24, PIO_PULLUP);
	pio_handler_set(PIOC, ID_PIOC, PIO_PC24, PIO_IT_HIGH_LEVEL, pin_edge_handler);
	pio_enable_interrupt(PIOC, PIO_PC24);
	NVIC_SetPriority(PIOC_IRQn, (11 << (8 - configPRIO_BITS)));
	NVIC_EnableIRQ(PIOC_IRQn);

	/* Enable PWM peripheral clock */
	pmc_enable_periph_clk(ID_PWM);

	/* Disable PWM channels for LEDs */
	pwm_channel_disable(PWM, PWM_CHANNEL_6);

	/* Set PWM clock A as PWM_FREQUENCY*PERIOD_VALUE (clock B is not used) */
	pwm_clock_t clock_setting = {
		.ul_clka = PWM_FREQUENCY * PERIOD_VALUE,
		.ul_clkb = 0,
		.ul_mck = sysclk_get_cpu_hz()
	};
	pwm_init(PWM, &clock_setting);

	/* Initialize PWM channel for PWM_CHANNEL_6 */
	/* Period is left-aligned */
	g_pwm_channel_led0.alignment = PWM_ALIGN_LEFT;
	/* Output waveform starts at a low level */
	g_pwm_channel_led0.polarity = PWM_LOW;
	/* Use PWM clock A as source clock */
	g_pwm_channel_led0.ul_prescaler = PWM_CMR_CPRE_CLKA;
	/* Period value of output waveform */
	g_pwm_channel_led0.ul_period = PERIOD_VALUE;
	/* Duty cycle value of output waveform */
	g_pwm_channel_led0.ul_duty = INIT_DUTY_VALUE;
	g_pwm_channel_led0.channel = PWM_CHANNEL_6;

	pwm_channel_init(PWM, &g_pwm_channel_led0);
	
	/* Enable PWM channels for LEDs */
	pwm_channel_enable(PWM, PWM_CHANNEL_6);
	
	/* The queue is created to hold a maximum of 5 long values. */
	xQueue = xQueueCreate( 100, sizeof( uint32_t ) );

	/* Before a semaphore is used it must be explicitly created.  In this example
	a binary semaphore is created. */
	xCountSemaphore = xSemaphoreCreateCounting(100, 0);
	
	if( xCountSemaphore != NULL && xQueue != NULL ){
		xTaskCreate( vHandlerTask, "Handler", 960, NULL, 3, NULL );
		vTaskStartScheduler();
	}

	else
	{
		/* The queue and semaphore could not be created. */
		printf("Error: Could not able to create Queue and Semaphore \n");
	}

	for( ;; );
}

/*-----------------------------------------------------------*/

static void vHandlerTask( void *pvParameters )
{
	portBASE_TYPE xStatus;
	uint32_t lReceivedValue;
	const portTickType xTicksToWait = 100 / portTICK_RATE_MS;
	xSemaphoreTake( xCountSemaphore, 0 );

	for( ;; ){
		xSemaphoreTake( xCountSemaphore, portMAX_DELAY );

		xStatus = xQueueReceive( xQueue, &lReceivedValue, xTicksToWait);

		if( xStatus == pdPASS )
		{
			/* Set new duty cycle */
			g_pwm_channel_led0.channel = PWM_CHANNEL_6;
			pwm_channel_update_duty(PWM, &g_pwm_channel_led0, PERIOD_VALUE-lReceivedValue);
			printf( "Received = %d \n ", lReceivedValue );
			fflush(stdout);
		}
		else
		{
			/* We did not receive anything from the queue even after waiting for 100ms.
			This must be an error as the sending tasks are free running and will be
			continuously writing to the queue. */
			taskENTER_CRITICAL();
			{
				printf( "Could not receive from the queue.\r\n" );
				fflush(stdout);
			}
			taskEXIT_CRITICAL();
		}

	}
}

/*************************************************************************************/
//Interrupt handler
void pin_edge_handler(const uint32_t id, const uint32_t index)
{
	unsigned int pinVal = 0;
	unsigned int priority = 0;
	portBASE_TYPE xStatus;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	if ((id == ID_PIOC) && (index == PIO_PC24)){
		priority = NVIC_GetPriority(PIOC_IRQn);
		pinVal = ioport_get_pin_level(MY_INTERRUPT_A);
	}
	
	if(pinVal){
		if(ul_count < PERIOD_VALUE)
		ul_count++;
		printf("Count sent = %d\n",ul_count);
	}
	else{

		if(xQueue != NULL && xCountSemaphore != NULL){
			xStatus=xQueueSendFromISR(xQueue, &ul_count, &xHigherPriorityTaskWoken);

			if( xStatus == pdPASS )
			{
				xSemaphoreGiveFromISR( xCountSemaphore, &xHigherPriorityTaskWoken );
			}
			
			else{
				printf("Could not create Queue\n\n");
			}
			ul_count =0;
		}
	}


	
	
}

/*************************************************************************************/
//General functions
static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		#ifdef CONF_UART_CHAR_LENGTH
		.charlength = CONF_UART_CHAR_LENGTH,
		#endif
		.paritytype = CONF_UART_PARITY,
		#ifdef CONF_UART_STOP_BITS
		.stopbits = CONF_UART_STOP_BITS,
		#endif
	};
	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONSOLE_UART, &uart_serial_options);
}

void vApplicationIdleHook( void )
{
}
void vApplicationMallocFailedHook( void )
{
	for( ;; );
}
void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
	for( ;; );
}
void vApplicationTickHook( void )
{
}