#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/************************************************************************/
/*                        DEFINES                                       */
/************************************************************************/


#define BUT_PIO     PIOA
#define BUT_PIO_ID  ID_PIOA
#define BUT_PIO_PIN 11
#define BUT_PIO_PIN_MASK (1 << BUT_PIO_PIN)

#define USART_COM_ID ID_USART1
#define USART_COM USART1

#define AFEC_POT AFEC0
#define AFEC_POT_ID ID_AFEC0
#define AFEC_POT_CHANNEL 0 // Canal do pino PD30

/** RTOS  */
#define TASK_ADC_STACK_SIZE                (1024*10/sizeof(portSTACK_TYPE))
#define TASK_ADC_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/** prototypes */
void but_callback(void);
static void BUT_init(void);

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/
volatile int text[20];

TimerHandle_t xTimer;
int countdown = 0;
QueueHandle_t xQueueADC;
volatile int state=0;
volatile int but_flag;
uint32_t ul_status;
typedef struct {
  uint value;
} adcData;

static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);

static void USART1_init(void);
static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel,
                            afec_callback_t callback);
static void configure_console(void);



extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

static void RTT_init(float freqPrescale, uint32_t IrqNPulses,
uint32_t rttIRQSource) {

	uint16_t pllPreScale = (int)(((float)32768) / freqPrescale);

	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);

	if (rttIRQSource & RTT_MR_ALMIEN) {
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT))
		;
		rtt_write_alarm_time(RTT, IrqNPulses + ul_previous_time);
	}

	/* config NVIC */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);

	/* Enable RTT interrupt */
	if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
	else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
}

void RTT_Handler(void) {
  uint32_t ul_status;
  ul_status = rtt_get_status(RTT);
  /* IRQ due to Alarm */
  if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		countdown = countdown -2;
   }  
}


void but_callback(void) {
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/



static void AFEC_pot_callback(void) {
  adcData adc;
  adc.value = afec_channel_get_value(AFEC_POT, AFEC_POT_CHANNEL)/205+1;
  BaseType_t xHigherPriorityTaskWoken = pdTRUE;
  xQueueSendFromISR(xQueueADC, &adc, &xHigherPriorityTaskWoken);
}

void vTimerCallback(TimerHandle_t xTimer) {
  /* Selecina canal e inicializa conversï¿½o */
  afec_channel_enable(AFEC_POT, AFEC_POT_CHANNEL);
  afec_start_software_conversion(AFEC_POT);
}

int cont = 0;


static void task_adc(void *pvParameters) {
	config_AFEC_pot(AFEC_POT, AFEC_POT_ID, AFEC_POT_CHANNEL, AFEC_pot_callback);
	//h = RTT_init(rtt_get_status(RTT), 100, RTT_MR_ALMIEN);
	
	gfx_mono_ssd1306_init();

	xTimer = xTimerCreate(/* Just a text name, not used by the RTOS
                      kernel. */
                      "Timer",
                      /* The timer period in ticks, must be
                      greater than 0. */
                      100,
                      /* The timers will auto-reload themselves
                      when they expire. */
                      pdTRUE,
                      /* The ID is used to store a count of the
                      number of times the timer has expired, which
                      is initialised to 0. */
                      (void *)0,
                      /* Timer callback */
                      vTimerCallback);
	xTimerStart(xTimer, 0);

	adcData adc;
  	char vetor1[10];
	char vetor2[10];
  	int valor;
	

	int valor_atual = 0;
	int valor_previo = 0;
	while(1)  {
		if (state == 0) {
			while (cont != 100){
			if (xQueueReceive(xQueueADC, &(adc), 10)){
				gfx_mono_draw_filled_rect(0, 0, 128, 32, GFX_PIXEL_CLR);
				sprintf(vetor1, "%u", adc.value);
			}
			    gfx_mono_draw_string("Freq", 0, 0, &sysfont);
			    gfx_mono_draw_string(vetor1, 0, 15, &sysfont);

				//RTT_init(adc.value, 100, RTT_MR_ALMIEN);
                vTaskDelay(500 / (adc.value));  
					sprintf(vetor2, "%d", cont);
					gfx_mono_draw_string("Contagem", 70, 0, &sysfont);
					gfx_mono_draw_string(vetor2, 70, 15, &sysfont);
					vTaskDelay(500 / (adc.value));
					cont++;
					if (cont==100){
						state=1;	
					}

		}		
	}
	}
}

static void task_adc_2(void *pvParameters) {
	config_AFEC_pot(AFEC_POT, AFEC_POT_ID, AFEC_POT_CHANNEL, AFEC_pot_callback);

	gfx_mono_ssd1306_init();
	xTimer = xTimerCreate("Timer",100,pdTRUE,(void *)0,vTimerCallback);
	xTimerStart(xTimer, 0);

	adcData adc;
	
	countdown=10;
	while (1) {
		if (state == 1) {
			gfx_mono_draw_filled_rect(0, 0, 128, 32, GFX_PIXEL_CLR);
			RTT_init(4, 4, RTT_MR_ALMIEN);
			sprintf(text, "Alarme %d", countdown);
			gfx_mono_draw_string(text, 0, 0, &sysfont);
			vTaskDelay(1000);

			if (countdown == 0){
				gfx_mono_draw_filled_rect(0, 0, 128, 32, GFX_PIXEL_CLR);
				state=0;
				cont=0;
			}
			
		}
	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}


static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel,
                            afec_callback_t callback) {
  /*************************************
   * Ativa e configura AFEC
   *************************************/
  /* Ativa AFEC - 0 */
  afec_enable(afec);

  /* struct de configuracao do AFEC */
  struct afec_config afec_cfg;

  /* Carrega parametros padrao */
  afec_get_config_defaults(&afec_cfg);

  /* Configura AFEC */
  afec_init(afec, &afec_cfg);

  /* Configura trigger por software */
  afec_set_trigger(afec, AFEC_TRIG_SW);

  /*** Configuracao especï¿½fica do canal AFEC ***/
  struct afec_ch_config afec_ch_cfg;
  afec_ch_get_config_defaults(&afec_ch_cfg);
  afec_ch_cfg.gain = AFEC_GAINVALUE_0;
  afec_ch_set_config(afec, afec_channel, &afec_ch_cfg);

  /*
  * Calibracao:
  * Because the internal ADC offset is 0x200, it should cancel it and shift
  down to 0.
  */
  afec_channel_set_analog_offset(afec, afec_channel, 0x200);

  /***  Configura sensor de temperatura ***/
  struct afec_temp_sensor_config afec_temp_sensor_cfg;

  afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
  afec_temp_sensor_set_config(afec, &afec_temp_sensor_cfg);

  /* configura IRQ */
  afec_set_callback(afec, afec_channel, callback, 1);
  NVIC_SetPriority(afec_id, 4);
  NVIC_EnableIRQ(afec_id);
}
static void BUT_init(void) {
	/* configura prioridae */
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4);

		// Desativa WatchDog Timer
	WDT->WDT_MR = WDT_MR_WDDIS;

	/* conf bot�o como entrada */
	pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PIO, BUT_PIO_PIN_MASK, 60);
	pio_enable_interrupt(BUT_PIO, BUT_PIO_PIN_MASK);
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIO_PIN_MASK, PIO_IT_FALL_EDGE , but_callback);
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/


int main(void) {
	sysclk_init();
	board_init();
	configure_console();

	xQueueADC = xQueueCreate(100, sizeof(adcData));
	if (xQueueADC == NULL)
	printf("falha em criar a queue xQueueADC \n");

	
	if (xTaskCreate(task_adc, "adc", TASK_ADC_STACK_SIZE, NULL, TASK_ADC_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create adc task\r\n");
	}

	if (xTaskCreate(task_adc_2, "adc2", TASK_ADC_STACK_SIZE, NULL, TASK_ADC_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create adc2 task\r\n");
	}

		//adcData adc;

	//RTT_init(adc.value, 100, RTT_MR_ALMIEN);
	/* Start the scheduler. */
	vTaskStartScheduler();

  /* RTOS n�o deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
