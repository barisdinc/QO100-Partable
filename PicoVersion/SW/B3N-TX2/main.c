/*
 * uSDR.c
 *
 * Created: Mar 2021
 * Author: Arjan te Marvelde
 * 
 * The main loop of the application.
 * This initializes the units that do the actual work, and then loops in the background. 
 * Other units are:
 * - dsp.c, containing all signal processing in RX and TX branches. This part runs on the second processor core.
 * - si5351.c, containing all controls for setting up the si5351 clock generator.
 * - lcd.c, contains all functions to put something on the LCD
 * - hmi.c, contains all functions that handle user inputs
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/sem.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"

#include "main.h"
#include "hmi.h"
#include "lcd.h"
#include "dsp.h"
#include "adf4351.h"
#include "monitor.h"
#include "ssd1306_i2c.h"

//static 
ADF4351_cfg vfo = 
{
	.pwrlevel = 0, //minimum power
	.RD2refdouble = 0, ///< ref doubler off
	.RD1Rdiv2 = 0,   ///< ref divider off
	.ClkDiv = 150,
	.BandSelClock = 200,
	.RCounter = 1,  ///< R counter to 1 (no division)
	.ChanStep = 10000,  ///< set to 1 MHz steps
	.pins = 
	{
		.gpio_ce = 12,
		.gpio_cs = PICO_DEFAULT_SPI_CSN_PIN, // dummy pin
		.gpio_le = 13, 
		.gpio_sclk = PICO_DEFAULT_SPI_SCK_PIN,
		.gpio_mosi = PICO_DEFAULT_SPI_TX_PIN, 
		.gpio_miso = PICO_DEFAULT_SPI_RX_PIN, // dummy pin
		.gpio_ld = 14,
	}
};

/*
 * Wrappers around i2c_write_blocking() and i2c_read_blocking()
 * The SDK functions return too soon, potentially causing overlapping calls
 */

int i2c_put_data(i2c_inst_t *i2c, uint8_t addr, const uint8_t *src, size_t len, bool nostop)
{
	int r = i2c_write_blocking(i2c, addr, src, len, nostop);
	sleep_us(I2C_LINGER_US);
	return(r);
}

int i2c_get_data(i2c_inst_t *i2c, uint8_t addr, uint8_t *dst, size_t len, bool nostop)
{
	int r = i2c_read_blocking(i2c, addr, dst, len, nostop);
	sleep_us(I2C_LINGER_US);
	return(r);
}


/* 
 * LED TIMER definition and callback routine
 */
struct repeating_timer led_timer;
bool led_callback(struct repeating_timer *t) 
{
	static bool led_state;
	
	gpio_put(PICO_DEFAULT_LED_PIN, led_state);
	led_state = (led_state?false:true);
	return true;
}


/*
 * Scheduler callback function.
 * This executes every LOOP_MS.
 */
semaphore_t loop_sem;
struct repeating_timer loop_timer;
bool loop_callback(struct repeating_timer *t)
{
	sem_release(&loop_sem);
	return(true);
}


int main()
{
	/* 
	 * Main loop rnning on Core 0
	 * Optional: increase core voltage (normally 1.1V)
	 * Optional: overclock the CPU to 250MHz  (normally 125MHz)
	 * Note that clk_peri (e.g. I2C) is derived from the SYS PLL
	 * Note that clk_adc sampling clock is derived from the 48MHz USB PLL.
	 */
	//vreg_set_voltage(VREG_VOLTAGE_1_25); sleep_ms(10);
	//set_sys_clock_khz(250000, false); sleep_ms(10);
	
	/* 
	 * Initialize LED pin output 
	 */
	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
	gpio_put(PICO_DEFAULT_LED_PIN, true);									// Set LED on
//	add_repeating_timer_ms(-LED_MS, led_callback, NULL, &led_timer);

	/*
	 * i2c0 is used for the si5351 interface
	 * i2c1 is used for the LCD and all other interfaces
	 * if the display cannot keep up, try lowering the i2c1 frequency
	 * Do not invoke i2c using functions from interrupt handlers!
	 */
	// i2c_init(i2c0, 400000);													// i2c0 initialisation at 400Khz
	// gpio_set_function(I2C0_SDA, GPIO_FUNC_I2C);
	// gpio_set_function(I2C0_SCL, GPIO_FUNC_I2C);
	// gpio_pull_up(I2C0_SDA);
	// gpio_pull_up(I2C0_SCL);
	// i2c_init(i2c1, 100000);													// i2c1 initialisation at 100Khz
	// gpio_set_function(I2C1_SDA, GPIO_FUNC_I2C);
	// gpio_set_function(I2C1_SCL, GPIO_FUNC_I2C);
	// gpio_pull_up(I2C1_SDA);
	// gpio_pull_up(I2C1_SCL);
	// spi initialisation for ADF4351 control
    // Enable SPI 0 at 1 MHz and connect to GPIOs
    spi_init(spi_default, 1000 * 1000); //spi0
    gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI); //GPIO16
    gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);//GPIO18
    gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI); //GPIO19
    gpio_set_function(PICO_DEFAULT_SPI_CSN_PIN, GPIO_FUNC_SPI);//GPIO17
	/* Initialize the SW units */
 	mon_init();																// Monitor shell on stdio
// 	si_init();																// VFO control unit
 	dsp_init();																// Signal processing unit

	// Initialize ADF4351
	busy_wait_ms(2000);
	printf("starting...\n");


	ADF4351_initialise(&vfo); // initialise the chip
    if(ADF4351_set_ref_freq(&vfo, 24999900) != 0)
        printf("Reference frequency input invalid");

    ADF4351_enable(&vfo); // power on the device
    ADF4351_set_freq(&vfo, 2400277000); // set output frequency to 440MHz


//   #if (LCD_TYPE != LCD_SSD1306)
//   	lcd_init();																// LCD output unit
//   #else
//   	SSD1306_init();

  	// struct render_area frame_area = {
  	// 	start_col: 0,
  	// 	end_col : SSD1306_WIDTH - 1,
  	// 	start_page : 0,
  	// 	end_page : SSD1306_NUM_PAGES - 1
  	// };	
    //   uint8_t buf[SSD1306_BUF_LEN];
    //   memset(buf, 0, SSD1306_BUF_LEN);
    //   render(buf, &frame_area);

// 	// mainX();
//   #endif
	add_repeating_timer_ms(-LED_MS, led_callback, NULL, &led_timer);

	hmi_init();																// HMI user inputs
	
	/* A simple round-robin scheduler */
	sem_init(&loop_sem, 1, 1) ;	
	add_repeating_timer_ms(-LOOP_MS, loop_callback, NULL, &loop_timer);
	while (1) 										
	{
		sem_acquire_blocking(&loop_sem);									// Wait until timer callback releases sem
		hmi_evaluate();														// Refresh HMI (and VFO, BPF, etc)
		mon_evaluate();														// Check monitor input
	}

    return 0;
}
