/*
 * LED.c
 *
 * Created: 23/07/2018 15:19:40
 *  Author: Marcus
 */ 
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include<stdio.h>
#include<stdlib.h>
#include "LED.h"
#include "../i2c/i2c_machine.h"
#include "../i2c/i2c_slave_defs.h"
#include "../ws2812/light_ws2812.h"

const uint8_t led_remappings[N_LEDS] = LED_REMAPS; // i2c/defined in i2c_slave_defs.h
volatile uint8_t led_data[N_LEDS*3];

//Calculates all values for the led's and send values
void update_led_values(void)
{
	uint8_t i=0;
	for (i=0;i<N_LEDS;i++)
	{
		// Remap from old to new convention (due to new wiring scheme)
		uint8_t j = led_remappings[i];

		//the analog led were common anonde led. this means, that the negative pad was switched. at a value of 255, the led was dark, at 0 it was bright. the addressable led are the other way around: 0=dark, 255= bright. Thats why we need to substract 255 from the value we calculate!
		//analog LED channel were as follows: R, G, B
		//addressable LED channel are as follows: G, R, B

		//led_data[3*i+0]=255-getI2CPWMValue(I2C_SLAVE_LED,i*3+1);
		//led_data[3*i+1]=255-getI2CPWMValue(I2C_SLAVE_LED,i*3+0);
		//led_data[3*i+2]=255-getI2CPWMValue(I2C_SLAVE_LED,i*3+2);
		// I guess that was wrong, they are mapped R, G, B as well:
		led_data[3*j+0]=255-getI2CPWMValue(I2C_SLAVE_LED,i*3+0);
		led_data[3*j+1]=255-getI2CPWMValue(I2C_SLAVE_LED,i*3+1);
		led_data[3*j+2]=255-getI2CPWMValue(I2C_SLAVE_LED,i*3+2);
	}
	//send led values
	ws2812_sendarray((uint8_t *)led_data, N_LEDS * 3);
}