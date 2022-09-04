#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

//LED1
#define LED1_PIO           PIOA                 // periferico que controla o LED
#define LED1_PIO_ID        ID_PIOA              // ID do perif�rico PIOC (controla LED)
#define LED1_PIO_IDX       0                   // ID do LED no PIO
#define LED1_PIO_IDX_MASK  (1 << LED1_PIO_IDX)   // Mascara para CONTROLARMOS o LED

// Configuracoes do botao1
#define BUT1_PIO  PIOD
#define BUT1_PIO_ID ID_PIOD
#define BUT1_PIO_IDX 28
#define BUT1_PIO_IDX_MASK (1u << BUT1_PIO_IDX) // esse j� est� pronto.

// Configuracoes do botao2
#define BUT2_PIO  PIOC
#define BUT2_PIO_ID ID_PIOC
#define BUT2_PIO_IDX 31
#define BUT2_PIO_IDX_MASK (1u << BUT2_PIO_IDX) // esse j� est� pronto.

// Configuracoes do botao3
#define BUT3_PIO  PIOA
#define BUT3_PIO_ID ID_PIOA
#define BUT3_PIO_IDX 19
#define BUT3_PIO_IDX_MASK (1u << BUT3_PIO_IDX) // esse j� est� pronto.

#define COUNT 20

volatile char but1_flag = 0;
volatile char is_pressed;
volatile char but2_flag = 0;
volatile char stop = 1;
float time;
int contagem = 0;

void pisca_led(int t);
void but1_callback(void);
void but2_callback(void);
void init(void);


void pisca_led(int t){
	for(int i=120;i>=40;i-=2){
		gfx_mono_draw_rect(i, 5, 2, 10, GFX_PIXEL_CLR);
	}
	
	for (int i=0;i < COUNT && !stop; i++){
		gfx_mono_draw_rect(40+4*i, 5, 2, 10, GFX_PIXEL_SET);
		pio_clear(LED1_PIO, LED1_PIO_IDX_MASK);
		delay_ms(t);
		pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
		delay_ms(t);
	}
}

void but1_callback(void)
{
	is_pressed = 1;
	if (pio_get(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK)) {
		but1_flag = 1;
		
	}
	else{
		but1_flag = 0;
	}
}

void but2_callback(void)
{
	stop = 1;
	but1_flag = 0;
	but1_flag = 0;
}

void but3_callback(void)
{
	but1_flag = 1;
	but2_flag = 0;
	stop = 0;
}

void init(void)
{
	board_init();
	sysclk_init();
	delay_init();
	
	// Configura led
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_configure(LED1_PIO, PIO_OUTPUT_0, LED1_PIO_IDX_MASK, PIO_DEFAULT);

	// Inicializa clock do perif�rico PIO responsavel pelo botao
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pmc_enable_periph_clk(BUT3_PIO_ID);

	// Configura PIO para lidar com o pino do bot�o como entrada
	// com pull-up
	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT1_PIO, BUT1_PIO_IDX_MASK, 60);
	
	pio_configure(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT2_PIO, BUT2_PIO_IDX_MASK, 60);
	
	pio_configure(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT3_PIO, BUT3_PIO_IDX_MASK, 60);

	// Configura interrup��o no pino referente ao botao e associa
	// fun��o de callback caso uma interrup��o for gerada
	// a fun��o de callback � a: but_callback()
	pio_handler_set(BUT1_PIO,
	BUT1_PIO_ID,
	BUT1_PIO_IDX_MASK,
	PIO_IT_EDGE,
	but1_callback);
	
	pio_handler_set(BUT2_PIO,
	BUT2_PIO_ID,
	BUT2_PIO_IDX_MASK,
	PIO_IT_RISE_EDGE,
	but2_callback);
	
	pio_handler_set(BUT3_PIO,
	BUT3_PIO_ID,
	BUT3_PIO_IDX_MASK,
	PIO_IT_RISE_EDGE,
	but3_callback);

	// Ativa interrup��o e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT1_PIO);
	
	pio_enable_interrupt(BUT2_PIO, BUT2_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT2_PIO);
	
	pio_enable_interrupt(BUT3_PIO, BUT3_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT3_PIO);
	
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais pr�ximo de 0 maior)
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 4); // Prioridade 4
	
	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_SetPriority(BUT2_PIO_ID, 4); // Prioridade 4
	
	NVIC_EnableIRQ(BUT3_PIO_ID);
	NVIC_SetPriority(BUT3_PIO_ID, 4); // Prioridade 4
}

int main (void)
{
	WDT->WDT_MR = WDT_MR_WDDIS;
	init();
	char str[15]; //
	time = 200;
	sprintf(str, "pausado "); //

	// Init OLED
	gfx_mono_ssd1306_init();
	gfx_mono_draw_filled_circle(20, 16, 16, GFX_PIXEL_SET, GFX_WHOLE);
	gfx_mono_draw_string(str, 50,16, &sysfont);
	
	
	/* Insert application code here, after the board has been initialized. */
	while(1) {
		if (is_pressed){
			if (!but1_flag){
				contagem++;
			}
			else{
				if (contagem > 3000000){
					time += 100;
					sprintf(str, "%.2f hz", 1000.0/time); //
					gfx_mono_draw_string(str, 50,16, &sysfont);
				}
				else{
					if (time>100){
						time -= 100;
					}
					sprintf(str, "%.2f hz", 1000.0/time); //
					gfx_mono_draw_string(str, 50,16, &sysfont);
					contagem = 0;
				}
				contagem = 0;
				is_pressed = 0;
				
			}
		}
		
		if (but2_flag){
			if (time>100){
				time -= 100;
			}
			sprintf(str, "%.2f hz", 1000.0/time); //
			gfx_mono_draw_string(str, 50,16, &sysfont);
			
			but2_flag = 0;
		}
		// 		sprintf(str, "pausado "); //
		// 		gfx_mono_draw_string(str, 50,16, &sysfont);
		//pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
		

		
		
	}
}
