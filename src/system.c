#include "nrf52840.h"
#include "nrf52840_bitfields.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <nrf_gpio.h>

#include "printf.h"
#include "math.h"
#include "stdlib.h"

#define GPIO_BURNER 15

static SemaphoreHandle_t smphr_on;
static SemaphoreHandle_t smphr_off;
static TaskHandle_t sys_task_handle;
static TaskHandle_t idle_task_handle;

typedef struct 
{
	int mean_ev1,mean_ev2;
	int sd_ev1,sd_ev2;
}event_tick_struct;

event_tick_struct event_tick;

int sampleNormal (int mu, int sigma)
{
  float U1, U2, W, mult;
  float X1;
 
  do
    {
      U1 = -1 + ((float) rand () / RAND_MAX) * 2;
      U2 = -1 + ((float) rand () / RAND_MAX) * 2;
      W = pow (U1, 2) + pow (U2, 2);
    }
  while (W >= 1 || W == 0);

  mult = sqrt ((-2 * log (W)) / W);
  X1 = U1 * mult;
  return (int)(mu + sigma * X1);
}

void set_event_ticks(int lmean_ev1, int lsd_ev1,int lmean_ev2,int lsd_ev2)
{
    event_tick.mean_ev1 = lmean_ev1;
	event_tick.sd_ev1 = lsd_ev1;
	event_tick.mean_ev2 = lmean_ev2;
	event_tick.sd_ev2 = lsd_ev2;
}

int get_event_ticks(int event_num)
{
    if(event_num == 1){
       return sampleNormal(event_tick.mean_ev1,event_tick.sd_ev1);
	}else if(event_num == 2){
      return sampleNormal(event_tick.mean_ev2,event_tick.sd_ev2);
	}else{
		// do nothing
	}

}

void vPortSetupTimerInterrupt(void)
{
	/* Start LFCLK in LFRC low power mode */
	NRF_CLOCK->LFRCMODE = CLOCK_LFRCMODE_MODE_ULP;
	/* 32.768 kHz crystal oscillator (LFXO) */
	NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_Xtal;
	NRF_CLOCK->TASKS_LFCLKSTART = 1;

    /* 1 ms compare value, generates EVENTS_COMPARE[0] */
	NRF_RTC1->CC[0] = get_event_ticks(1);
	/* 100 ms compare value, generates EVENTS_COMPARE[1] */
	NRF_RTC1->CC[1] = get_event_ticks(2);
    /* Enable EVENTS_COMPARE[0] and EVENTS_COMPARE[1] generation */
	NRF_RTC1->EVTENSET = RTC_EVTENSET_COMPARE0_Msk | RTC_EVTENSET_COMPARE1_Msk;
	 /* Enable IRQ on EVENTS_COMPARE[0] and EVENTS_COMPARE[1] */
	NRF_RTC1->INTENSET = RTC_INTENSET_COMPARE0_Msk | RTC_INTENSET_COMPARE1_Msk;

    /* Configure PPI channel with connection between NRF_RTC1->EVENTS_COMPARE[1] and NRF_RTC1->TASKS_CLEAR */
	NRF_PPI->CH[0].EEP = (uint32_t)&NRF_RTC1->EVENTS_COMPARE[1];
	NRF_PPI->CH[0].TEP = (uint32_t)&NRF_RTC1->TASKS_CLEAR;
	/* Enable PPI channel */	
	NRF_PPI->CHENSET = PPI_CHENSET_CH0_Msk;

	NRF_RTC1->TASKS_CLEAR = 1;
	/* start RTC1 */
	NRF_RTC1->TASKS_START = 1;

	NVIC_SetPriority(RTC1_IRQn, configKERNEL_INTERRUPT_PRIORITY);
	NVIC_EnableIRQ(RTC1_IRQn);
}

// This IRQ handler triggers every 1ms and 100ms
void RTC1_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if (NRF_RTC1->EVENTS_COMPARE[0] == 1) {
		NRF_RTC1->EVENTS_COMPARE[0] = 0;
		xSemaphoreGiveFromISR(smphr_off, &xHigherPriorityTaskWoken);
	}
	if (NRF_RTC1->EVENTS_COMPARE[1] == 1) {
		NRF_RTC1->EVENTS_COMPARE[1] = 0;
		xTaskIncrementTick();
		xSemaphoreGiveFromISR(smphr_on, &xHigherPriorityTaskWoken);
	}
#if 0
	uint32_t isrstate = portSET_INTERRUPT_MASK_FROM_ISR();

	/* Increment the RTOS tick as usual which checks if there is a need for rescheduling */
	if (xTaskIncrementTick() != pdFALSE) {
		/* A context switch is required.  Context switching is performed in
        the PendSV interrupt.  Pend the PendSV interrupt. */
		SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
		__SEV();
	}

	portCLEAR_INTERRUPT_MASK_FROM_ISR(isrstate);
#endif
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void burner_start(void)
{
	NRF_P0->OUTCLR = (1 << GPIO_BURNER);
	//printf("started burner!\r\n");
}
static void burner_stop(void)
{
	NRF_P0->OUTSET = (1 << GPIO_BURNER);
	//printf("stopped burner!\r\n");
}
static bool burner_is_active(void)
{
	if (NRF_P0->OUT & (1 << GPIO_BURNER)){
		return false;
		//printf("active burner!\r\n");
	}
	else{
		return true;
		//printf("active burner!\r\n");
	}
}

void TIMER2_IRQHandler(void)
{
	if (NRF_TIMER2->EVENTS_COMPARE[0] == 1) {
		NRF_TIMER2->EVENTS_COMPARE[0] = 0;
		NRF_TIMER2->TASKS_STOP = 1;
		burner_stop();
	}
}

static void vIdleTask(void *pvParameter)
{
	//printf("idle task\r\n");
	UNUSED_PARAMETER(pvParameter);

	burner_stop();

	//nrf_gpio_pin_set(GPIO_BURNER);
	nrf_gpio_cfg_output(GPIO_BURNER);

	NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;
	NRF_TIMER2->CC[0] = 16;
	NRF_TIMER2->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
	NVIC_EnableIRQ(TIMER2_IRQn);
	/* The priority specifies the interrupt priority value, whereby lower values indicate a higher priority. 
	  The default priority is 0 for every interrupt. This is the highest possible priority.*/
	NVIC_SetPriority(TIMER2_IRQn, 0);

	for (;;) {
		if (burner_is_active() == false) {
			NRF_TIMER2->TASKS_CLEAR = 1;
			NRF_TIMER2->TASKS_START = 1;
			burner_start();
		}
		NRF_TIMER2->TASKS_CLEAR = 1;
	}
}

unsigned int factorial(unsigned int n)
{
 if (n == 0)
  return 1;
 return n * factorial(n - 1);
}

static void vSystemTask(void *pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
    //printf("system task\r\n");
	for (;;) {
		/* See if we can obtain the semaphore.  If the semaphore is not
        available wait portMAX_DELAY ticks to see if it becomes free. */
		if(xSemaphoreTake(smphr_off, portMAX_DELAY) == pdTRUE){
            //printf("off!\r\n");
        }
		/* A block time of zero can be used to poll the semaphore. */
		if (xSemaphoreTake(smphr_on, portMAX_DELAY) == pdTRUE){
            
		   int num = 31; 
           //printf("Factorial of %d is %d \r\n", num, factorial(num));
			//printf("on!\r\n");
		}
			//__NOP();
		/*When using the semaphore for synchronisation with an ISR in this 
		manner there is no need to 'give' the semaphore back. */ 
	}
}

int system_init(void)
{
	/* Create binary semaphore for synchronisation of tasks with ISR */
	smphr_on = xSemaphoreCreateBinary();
	smphr_off = xSemaphoreCreateBinary();
	/* Create task SYS with a high priority  */
	xTaskCreate(vSystemTask, "SYS", configMINIMAL_STACK_SIZE + 200, NULL, 0xFF, &sys_task_handle);
	/* Create task IDL with a low priority  */
	xTaskCreate(vIdleTask, "IDL", configMINIMAL_STACK_SIZE + 200, NULL, tskIDLE_PRIORITY, &idle_task_handle);
	return 0;
}