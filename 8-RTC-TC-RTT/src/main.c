#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/* Botao da placa */
#define BUT_PIO     PIOA
#define BUT_PIO_ID  ID_PIOA
#define BUT_PIO_PIN 11
#define BUT_PIO_PIN_MASK (1 << BUT_PIO_PIN)

#define LED_PIO         PIOC
#define LED_PIO_ID      ID_PIOC
#define LED_PIO_IDX	8
#define LED_IDX_MASK    (1<<LED_PIO_IDX)

#define LED_1_PIO PIOA
#define LED_1_PIO_ID ID_PIOA
#define LED_1_IDX 0
#define LED_1_IDX_MASK (1 << LED_1_IDX)

#define LED_2_PIO PIOC
#define LED_2_PIO_ID ID_PIOC
#define LED_2_IDX 30
#define LED_2_IDX_MASK (1 << LED_2_IDX)

#define LED_3_PIO PIOB
#define LED_3_PIO_ID ID_PIOB
#define LED_3_IDX 2
#define LED_3_IDX_MASK (1 << LED_3_IDX)


#define BUT_1_PIO PIOD
#define BUT_1_PIO_ID ID_PIOD
#define BUT_1_IDX 28
#define BUT_1_IDX_MASK (1u << BUT_1_IDX)

#define BUT_2_PIO PIOA
#define BUT_2_PIO_ID ID_PIOA
#define BUT_2_IDX 19
#define BUT_2_IDX_MASK (1u << BUT_2_IDX)

#define BUT_3_PIO PIOC
#define BUT_3_PIO_ID ID_PIOC
#define BUT_3_IDX 31
#define BUT_3_IDX_MASK (1u << BUT_3_IDX)

/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

typedef struct  {
	uint32_t year;
	uint32_t month;
	uint32_t day;
	uint32_t week;
	uint32_t hour;
	uint32_t minute;
	uint32_t second;
} calendar;

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/** prototypes */
void but_callback(void);
static void BUT_init(void);
void io_init(void);
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type);


/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

void pin_toggle(Pio *pio, uint32_t mask) {
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

SemaphoreHandle_t xSemaphoreMode;
SemaphoreHandle_t xSemaphoreRTC;
SemaphoreHandle_t xSemaphoreBut;
SemaphoreHandle_t xSemaphoreClock;
/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/
void RTT_Handler(void) {
	uint32_t ul_status;
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		//Aqui dentro modificar para avisar qual fila ou semaforo deseja-se controlar
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(xSemaphoreMode,xHigherPriorityTaskWoken);
	}
}

void RTC_Handler(void) {
	uint32_t ul_status = rtc_get_status(RTC);
	
	/* seccond tick */
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		// o código para irq de segundo vem aqui
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(xSemaphoreClock,xHigherPriorityTaskWoken);
	}
	
	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
		// o código para irq de alame vem aqui
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(xSemaphoreRTC,xHigherPriorityTaskWoken);
	}

	rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
}


void TC0_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	* Isso é realizado pela leitura do status do periférico
	**/
	volatile uint32_t status = tc_get_status(TC0, 0);

	/** Muda o estado do LED (pisca) **/
	pin_toggle(LED_PIO, LED_IDX_MASK);  
}

void TC1_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	* Isso é realizado pela leitura do status do periférico
	**/
	volatile uint32_t status = tc_get_status(TC0, 1);

	/** Muda o estado do LED (pisca) **/
	pin_toggle(LED_1_PIO, LED_1_IDX_MASK);  
}

void TC2_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	* Isso é realizado pela leitura do status do periférico
	**/
	volatile uint32_t status = tc_get_status(TC0, 2);

	/** Muda o estado do LED (pisca) **/
	pin_toggle(LED_3_PIO, LED_3_IDX_MASK);  
}


void but1_callback(void) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreBut,xHigherPriorityTaskWoken);
}

void but2_callback(void) {
}

void but3_callback(void) {
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_oled(void *pvParameters) {
	gfx_mono_ssd1306_init();
	TC_init(TC0, ID_TC1, 1, 2);
	tc_start(TC0, 1);
	RTT_init(10,40,RTT_MR_ALMIEN);
	TC_init(TC0, ID_TC0, 0, 5);
	tc_start(TC0, 0);
	
	/** Configura RTC */
	calendar rtc_initial = {2018, 3, 19, 12, 15, 45 ,1};
	RTC_init(RTC, ID_RTC, rtc_initial, RTC_IER_ALREN | RTC_IER_SECEN);
	
	int rtc_count = 1;
	for (;;)  {
		if (xSemaphoreTake(xSemaphoreMode,100)){
			pin_toggle(LED_2_PIO, LED_2_IDX_MASK); 
			RTT_init(10,40,RTT_MR_ALMIEN);
		}
		if (xSemaphoreTake(xSemaphoreBut,0)){
			/* Leitura do valor atual do RTC */
			if (rtc_count){
				uint32_t current_hour, current_min, current_sec;
				uint32_t current_year, current_month, current_day, current_week;
				rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
				rtc_get_date(RTC, &current_year, &current_month, &current_day, &current_week);
				rtc_set_date_alarm(RTC, 1, current_month, 1, current_day);
				rtc_set_time_alarm(RTC, 1, current_hour, 1, current_min, 1, current_sec + 20);
			}
			
		}
		if (xSemaphoreTake(xSemaphoreRTC,0)){
			rtc_count = 0;
			for (int i=0;i<6;i++){
				pin_toggle(LED_3_PIO, LED_3_IDX_MASK);
				delay_ms(300);
			}
			TC_init(TC0, ID_TC2, 2, 3);
			tc_start(TC0, 2);
		}
		if (xSemaphoreTake(xSemaphoreClock,0)){
			gfx_mono_draw_string("        ", 5, 16, &sysfont);
			char str[512];
			uint32_t current_hour, current_min, current_sec;
			rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
			sprintf(str, "%d:%d:%d", current_hour,current_min,current_sec);
			gfx_mono_draw_string(str, 5, 16, &sysfont);
		}
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
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

static void BUT_init(void) {
	/* configura prioridae */
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4);

	/* conf botão como entrada */
	pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PIO, BUT_PIO_PIN_MASK, 60);
	pio_enable_interrupt(BUT_PIO, BUT_PIO_PIN_MASK);
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIO_PIN_MASK, PIO_IT_FALL_EDGE , but_callback);
}

void io_init(void) {
	pmc_enable_periph_clk(LED_1_PIO_ID);
	pmc_enable_periph_clk(LED_2_PIO_ID);
	pmc_enable_periph_clk(LED_3_PIO_ID);
	pmc_enable_periph_clk(LED_PIO_ID);
	pmc_enable_periph_clk(BUT_1_PIO_ID);
	pmc_enable_periph_clk(BUT_2_PIO_ID);
	pmc_enable_periph_clk(BUT_3_PIO_ID);

	pio_configure(LED_1_PIO, PIO_OUTPUT_0, LED_1_IDX_MASK, PIO_DEFAULT);
	pio_configure(LED_2_PIO, PIO_OUTPUT_0, LED_2_IDX_MASK, PIO_DEFAULT);
	pio_configure(LED_3_PIO, PIO_OUTPUT_0, LED_3_IDX_MASK, PIO_DEFAULT);
	pio_configure(LED_PIO, PIO_OUTPUT_0, LED_IDX_MASK, PIO_DEFAULT);

	pio_configure(BUT_1_PIO, PIO_INPUT, BUT_1_IDX_MASK, PIO_PULLUP| PIO_DEBOUNCE);
	pio_configure(BUT_2_PIO, PIO_INPUT, BUT_2_IDX_MASK, PIO_PULLUP| PIO_DEBOUNCE);
	pio_configure(BUT_3_PIO, PIO_INPUT, BUT_3_IDX_MASK, PIO_PULLUP| PIO_DEBOUNCE);

	pio_handler_set(BUT_1_PIO, BUT_1_PIO_ID, BUT_1_IDX_MASK, PIO_IT_FALL_EDGE,
	but1_callback);
	pio_handler_set(BUT_2_PIO, BUT_2_PIO_ID, BUT_2_IDX_MASK, PIO_IT_FALL_EDGE,
	but3_callback);
	pio_handler_set(BUT_3_PIO, BUT_3_PIO_ID, BUT_3_IDX_MASK, PIO_IT_FALL_EDGE,
	but2_callback);

	pio_enable_interrupt(BUT_1_PIO, BUT_1_IDX_MASK);
	pio_enable_interrupt(BUT_2_PIO, BUT_2_IDX_MASK);
	pio_enable_interrupt(BUT_3_PIO, BUT_3_IDX_MASK);

	pio_get_interrupt_status(BUT_1_PIO);
	pio_get_interrupt_status(BUT_2_PIO);
	pio_get_interrupt_status(BUT_3_PIO);

	NVIC_EnableIRQ(BUT_1_PIO_ID);
	NVIC_SetPriority(BUT_1_PIO_ID, 4);

	NVIC_EnableIRQ(BUT_2_PIO_ID);
	NVIC_SetPriority(BUT_2_PIO_ID, 4);

	NVIC_EnableIRQ(BUT_3_PIO_ID);
	NVIC_SetPriority(BUT_3_PIO_ID, 4);
}
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configura o PMC */
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  freq hz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura NVIC*/
  	NVIC_SetPriority(ID_TC, 4);
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
}

void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type) {
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(rtc, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.second);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 4);
	NVIC_EnableIRQ(id_rtc);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(rtc,  irq_type);
}

static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

	uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	if (rttIRQSource & RTT_MR_ALMIEN) {
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT));
		rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
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

/************************************************************************/
/* main                                                                 */
/************************************************************************/


int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	io_init();

	/* Initialize the console uart */
	configure_console();
	xSemaphoreMode = xSemaphoreCreateBinary();
	xSemaphoreBut = xSemaphoreCreateBinary();
	xSemaphoreRTC = xSemaphoreCreateBinary();
	xSemaphoreClock = xSemaphoreCreateBinary();

	/* Create task to control oled */
	if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create oled task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

  /* RTOS não deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
