#include "nrf52840.h"
#include "nrf52840_bitfields.h"
#include "FreeRTOS.h"

#include <nrf_clock.h>
#include <nrf_gpio.h>
#include <nrf_rtc.h>
#include <nrf_uart.h>

#include "nrf_delay.h"
#include "printf.h"
#include "task.h"

#define PIN_LED1 13
#define PIN_LED2 14
#define PIN_LED3 15
#define PIN_LED4 16
#define PIN_UART_TX 6

extern int system_init(void);
void set_event_ticks(int , int ,int ,int );
int get_event_ticks( );

/* configSUPPORT_STATIC_ALLOCATION is set to 1, so the application must provide
an implementation of vApplicationGetIdleTaskMemory() to provide the memory that
is used by the Idle task. */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
	/* If the buffers to be provided to the Idle task are declared inside this
  function then they must be declared static - otherwise they will be allocated
  on the stack and so not exists after this function exits. */
	static StaticTask_t xIdleTaskTCB;
	static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

	/* Pass out a pointer to the StaticTask_t structure in which the Idle task's
  state will be stored. */
	*ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

	/* Pass out the array that will be used as the Idle task's stack. */
	*ppxIdleTaskStackBuffer = uxIdleTaskStack;

	/* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
  Note that, as the array is necessarily of type StackType_t,
  configMINIMAL_STACK_SIZE is specified in words, not bytes. */
	*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void _putchar(char character)
{
	NRF_UART0->TXD = character;
	NRF_UART0->EVENTS_TXDRDY = 0;
	NRF_UART0->TASKS_STARTTX = 1UL;
	while (NRF_UART0->EVENTS_TXDRDY == 0) {
	};
	NRF_UART0->EVENTS_TXDRDY = 0;
	NRF_UART0->TASKS_STOPTX = 1UL;
}

static int uart_init(unsigned int pin_tx)
{
	nrf_uart_configure(NRF_UART0, NRF_UART_PARITY_EXCLUDED, NRF_UART_HWFC_DISABLED);
	NRF_UART0->ENABLE = 4UL;
	/* Setup TX pin only */
	NRF_UART0->PSEL.TXD = pin_tx;
	/* Disconnect RX pin */
	NRF_UART0->PSEL.RXD |= (1UL << 31);
	nrf_uart_baudrate_set(NRF_UART0, NRF_UART_BAUDRATE_1000000);
	return 0;
}

/* Configure the GPIO pin for LED 1 */
static void vLed0Function(void *pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
	nrf_gpio_pin_set(PIN_LED2);
	nrf_gpio_cfg_output(PIN_LED2);
	for (;;) {
		for (unsigned int i = 0; i < 50; i++) {
			nrf_gpio_pin_toggle(PIN_LED2);
			nrf_delay_us(1);
		}
	}
}

/* Configure the GPIO pin for LED 1 */
static void vLed1Function(void *pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
	nrf_gpio_pin_set(PIN_LED4);
	nrf_gpio_cfg_output(PIN_LED4);
	for (;;) {
		for (unsigned int i = 0; i < 25; i++) {
			nrf_gpio_pin_toggle(PIN_LED4);
			nrf_delay_us(4);
		}
		//vTaskDelay(1);
	}
}

static void vPrintFunction(void *pvParameter)
{
	UNUSED_PARAMETER(pvParameter);

	for (;;) {
		vTaskDelay(3);
		//printf("Tick!\r\n");
	}
}

int main(void)
{
	/* Used to pass a handle to the created task out of the xTaskCreate() function. */
	TaskHandle_t xLed0Handle;
	TaskHandle_t xLed1Handle;
	TaskHandle_t xPrintHandle;
	/*todo: unused variable */
	int rc;

	uart_init(PIN_UART_TX);

    /*todo: not configured completely. */
	nrf_gpio_cfg_output(27);

	/* Create the task, storing the handle. */
	xTaskCreate(vLed0Function, "L0", configMINIMAL_STACK_SIZE + 200, NULL, 2, &xLed0Handle);
	xTaskCreate(vLed1Function, "L1", configMINIMAL_STACK_SIZE + 200, NULL, 2, &xLed1Handle);
	xTaskCreate(vPrintFunction, "P0", configMINIMAL_STACK_SIZE + 200, NULL, 3, &xPrintHandle);

    set_event_ticks(3276,30,65535,100);
	//while(true){
      //  printf("n1 : %d\r\n",get_event_ticks(1));
	  //  printf("n2 : %d\r\n",get_event_ticks(2));
	  //  nrf_delay_ms(2000);
	//}

	system_init();
	//vPortSetupTimerInterrupt();

	/* Start the real time scheduler. */
	vTaskStartScheduler();

    nrf_gpio_pin_set(PIN_LED1);
	nrf_gpio_cfg_output(PIN_LED1);

    /* todo: The below part is not executed unless there is insufficient RAM. */
	while (1) {
		nrf_gpio_pin_toggle(PIN_LED1);
		nrf_delay_ms(100);
	}
}
