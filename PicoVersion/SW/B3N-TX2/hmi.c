/*
 * hmi.c
 *
 * The PTT is connected to GP15 and will be active, except when VOX is used.
 *
 */
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"

#include "main.h"
#include "lcd.h"
#include "hmi.h"
#include "dsp.h"
//#include "si5351.h"
#include "ssd1306_i2c.h"

/*
 * GPIO masks
 */
#define GP_MASK_PTT	(1<<GP_PTT)

/*
 * Event flags
 */
#define GPIO_IRQ_ALL		(GPIO_IRQ_LEVEL_LOW|GPIO_IRQ_LEVEL_HIGH|GPIO_IRQ_EDGE_FALL|GPIO_IRQ_EDGE_RISE)
#define GPIO_IRQ_EDGE_ALL	(GPIO_IRQ_EDGE_FALL|GPIO_IRQ_EDGE_RISE)

/* Event definitions */
#define HMI_E_PTTON			7
#define HMI_E_PTTOFF		8
																			// Set to 2 for certain types of mixer
uint32_t hmi_freq;
bool hmi_update;
#define PTT_DEBOUNCE	3													// Nr of cycles for debounce
int ptt_state;																// Debounce counter
bool ptt_active;															// Resulting state

/*
 * Some macros
 */
#ifndef MIN
#define MIN(x, y)        ((x)<(y)?(x):(y))  // Get min value
#endif
#ifndef MAX
#define MAX(x, y)        ((x)>(y)?(x):(y))  // Get max value
#endif


/*
 * GPIO IRQ callback routine
 * Sets the detected event and invokes the HMI state machine
 */
void hmi_callback(uint gpio, uint32_t events)
{

}


/*
 * Redraw the 16x2 LCD display, representing current state
 * This function is invoked regularly from the main loop.
 */
void hmi_evaluate(void)
{
	// char s[32];
	// uint8_t buf[SSD1306_BUF_LEN];
    // memset(buf, 0, SSD1306_BUF_LEN);
	
	// Print top line of display
	// if (tx_enabled)
	// 	sprintf(s, "%7.1f %c %-2d", (double)hmi_freq/1000.0, 0x07, 0);

	//lcd_writexy(0,0,s);
	// WriteString(buf, 0, 0, s);
	
	/* PTT debouncing */
	if (gpio_get(GP_PTT))													// Get PTT level
	{
		if (ptt_state<PTT_DEBOUNCE)											// Increment debounce counter when high
			ptt_state++;
	}
	else 
	{
		if (ptt_state>0)													// Decrement debounce counter when low
			ptt_state--;
	}
	if (ptt_state == PTT_DEBOUNCE)											// Reset PTT when debounced level high
		ptt_active = false;
	if (ptt_state == 0)														// Set PTT when debounced level low
		ptt_active = true;

	/* Set parameters corresponding to latest entered option value */
	
	// See if VFO needs update
	//si_evaluate(0, HMI_MULFREQ*(hmi_freq-FC_OFFSET));
	
	// Update peripherals according to menu setting
	// For frequency si5351 is set directly, HMI top line follows
	if (hmi_update)
	{
		dsp_setmode(1); //TODO: modu bul
		dsp_setvox(VOX_OFF);
		hmi_update = false;
	}
}


/*
 * Initialize the User interface
 */
void hmi_init(void)
{
	/*
	 * Notes on using GPIO interrupts: 
	 * The callback handles interrupts for all GPIOs with IRQ enabled.
	 * Level interrupts don't seem to work properly.
	 * For debouncing, the GPIO pins should be pulled-up and connected to gnd with 100nF.
	 * PTT has separate debouncing logic
	 */
	 
	// Init input GPIOs
	//gpio_init_mask(GP_MASK_IN);
	
	// Enable pull-ups
	gpio_pull_up(GP_PTT);
	gpio_set_oeover(GP_PTT, GPIO_OVERRIDE_HIGH);							// Enable output on PTT GPIO; bidirectional
	gpio_put(GP_PTT, true);											// Drive PTT high (inactive)   
	
	// Enable interrupt on level low
	gpio_set_irq_enabled(GP_PTT, GPIO_IRQ_EDGE_ALL, false);

	// Set callback, one for all GPIO, not sure about correctness!
	//gpio_set_irq_enabled_with_callback(GP_ENC_A, GPIO_IRQ_EDGE_ALL, true, hmi_callback);
		
	// Initialize LCD and set VFO
	hmi_freq = 7074000UL;													// Initial frequency

	//si_setphase(0, 1);														// Set phase to 90deg (depends on mixer type)
	//si_evaluate(0, HMI_MULFREQ*(hmi_freq-FC_OFFSET));						// Set freq to 7074 kHz (depends on mixer type)
	
	ptt_state  = PTT_DEBOUNCE;
	ptt_active = false;
	
	dsp_setmode(1); //TODO: bul dogrusunu
	dsp_setvox(VOX_OFF);
	hmi_update = false;
}

