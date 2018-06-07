/*
* Neopixel I2C Slave application
*
* At boot, scrolls a bright spot (init_color) along the array
* Will stop as soon as an i2c transaction is received
*/

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "i2c/i2c_slave_defs.h"
#include "i2c/i2c_machine.h"
#include "ws2812/light_ws2812.h"

#define StartRegister 0x06

volatile uint8_t i2c_reg[I2C_N_REG];
volatile uint8_t led_data[N_LEDS*3];
const uint8_t init_color[3] PROGMEM = { 0x00, 0x00, 0xFF};

static inline void set_leds_global(void)
{
	ws2812_setleds_constant((struct cRGB *)&REG_GLB_G, N_LEDS);
}

static inline void update_leds(void)
{
	ws2812_sendarray(led_data, N_LEDS * 3);
}

static inline char getValue(int Addr)
{
	uint16_t iOn=0;
	uint16_t iOff=0;
	
	iOn=(i2c_reg[Addr]<<8) + i2c_reg[Addr+1];
	iOff=(i2c_reg[Addr]<<8) + i2c_reg[Addr+1];
	
	//return (char) (((uint32_t)iOn)<<8)/((uint32_t)iOff);
	return (char) ((float)iOn/iOff*255);
	
	//uint32_t temp=iOn;
	//temp=temp*255;
	//temp=temp/iOff;
	//return (char)temp;
}


static inline void calc_led_values(void)
{
	char i=0;
	for (i=0;i<N_LEDS;i++)
	{
		//change red and green channel
		led_data[3*i+1]=getValue(StartRegister+i*12+0);
		led_data[3*i]=getValue(StartRegister+i*12+4);
		led_data[3*i+2]=getValue(StartRegister+i*12+8);
	}
}

void do_reset(void)
{
	char i = N_LEDS * 3;
	volatile uint8_t *p = i2c_reg + I2C_N_GLB_REG;

	cli();
	REG_GLB_G = 0;
	REG_GLB_R = 0;
	REG_GLB_B = 0;
	ws2812_setleds_constant((struct cRGB *)&REG_GLB_G, N_LEDS);
	REG_GLB_G = pgm_read_byte(init_color);
	REG_GLB_R = pgm_read_byte(init_color + 1);
	REG_GLB_B = pgm_read_byte(init_color + 2);
	REG_CTRL = 0;

	/* Reset the registers or we'll just go back to the old values! */
	while (i--) {
		*p = 0;
		p++;
	}

	sei();
}

void swirly(void)
{
	uint8_t led = N_LEDS;
	volatile uint8_t *p = led_data + (N_LEDS * 3);//i2c_reg + I2C_N_GLB_REG + (N_LEDS * 3);
	uint8_t g = pgm_read_byte(init_color);
	uint8_t r = pgm_read_byte(init_color + 1);
	uint8_t b = pgm_read_byte(init_color + 2);
	uint8_t tmp;

	/* Initialise a bright spot with a tail:
	 * { 255, 127, 63, 31, 15, 15, 15, 15 ... }
	 */
	while (led--) {
		*(--p) = b;
		if (b & 0xf0)
			b >>= 1;

		*(--p) = r;
		if (r & 0xf0)
			r >>= 1;

		*(--p) = g;
		if (g & 0xf0)
			g >>= 1;

	}

	/* Shuffle the bright spot along */
	g = pgm_read_byte(init_color);
	r = pgm_read_byte(init_color + 1);
	b = pgm_read_byte(init_color + 2);
	while (1)
	{
		update_leds();

		led = N_LEDS;
		p = &i2c_reg[I2C_N_GLB_REG];
		while (led--) {
			tmp = *p;
			*(p++) = g;
			if (led)
				g = tmp;

			tmp = *p;
			*(p++) = r;
			if (led)
				r = tmp;

			tmp = *p;
			*(p++) = b;
			if (led)
				b = tmp;
		}

		/* As soon as there's a transaction to handle, bail out */
		tmp = 70;
		while (tmp--) {
			if (i2c_check_stop())
				return;
			_delay_ms(1);
		}
	}
}

int main(void)
{
	DDRB = (1 << 3);

	//enable second USI Port (PA0:2)
	USIPP=0x01;

	i2c_init();
	sei();

	swirly();
	goto inner;

	while(1)
	{
		if (i2c_check_stop()) {
inner:
			if (REG_CTRL & CTRL_RST)
				do_reset();
			else if (REG_CTRL & CTRL_GLB)
				set_leds_global();
			else
				update_leds();
				calc_led_values();
		}
	}
}