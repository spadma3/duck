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

#include<stdio.h>
#include<stdlib.h>

#include "i2c/i2c_slave_defs.h"
#include "i2c/i2c_machine.h"
#include "ws2812/light_ws2812.h"
#include "ADC/ADC.h"
#include "SOC/SOC.h"
#include "LED/LED.h"
#include "Motor/Motor.h"
#include "Timer/Timer0.h"


int main(void)
{
	//if PERIODIC_LED_UPDATE is defined, the values of the leds are sent periodically. If new values arrive over i2c the leds are updated but most with every 1ms (1kHz). If no values arrive, the leds are updated at least every 500ms (2Hz).
	//if you like to update the LED each time, one single value is changed over i2c and only then, comment the following line out.
	//Warning: This could cause problems because much CPU time is used to update the led's and this could delay the i2c handeling.
	#define PERIODIC_LED_UPDATE 

	const uint16_t MinCyclesBetweenLEDUpdate=31;//min is 50us!, good is 1000us-->1kHz --> 6 Cykles
	const uint16_t MaxCyclesBetweenLEDUpdate=15625;//Min. Updaterate = 2Hz -->15625 Cykles
	
	static uint8_t LedDataSendRequest=0;	//1= new data can be transfered, 0=no new data
	static uint16_t LedDataLastSend=0;		//Counter value of last data transfer end
	static uint16_t ActualDelay;			//Cycles since last update
	static uint16_t counterTIM0 = 0;			
	
	
	//enable second USI Port (PA0:2)
	USIPP=0x01;
	i2c_init();
	
	//init motor outputs
	init_motor_outputs();
	
	//init Timer0
	TIM0_Init();

	//init ADC
	init_ADC();
	
	// init State of Charge moduel
	init_SOC();
	
	//enable interrupts
	sei();
	
	while(1)
	{
	
		//check if master has written new LED data, if yes, update leds
		if (i2c_check_stop(I2C_SLAVE_LED))
		{
			#ifdef PERIODIC_LED_UPDATE 
				LedDataSendRequest=1;	
			#else
				update_led_values();
			#endif					
		}
		//check if master has written new Motor data, if yes, update motor valued
		if (i2c_check_stop(I2C_SLAVE_MOTOR))
		{
			update_motor_values();
		}

		//Calculate Cycles since last sending
		counterTIM0 = TIM0_ReadTCNT0();
		ActualDelay = counterTIM0 - LedDataLastSend;

		#ifdef PERIODIC_LED_UPDATE 
			//update if minimal time reached and update request or if maximal time reached
			//minimal time reached and 
			if((ActualDelay>MinCyclesBetweenLEDUpdate && LedDataSendRequest==1) || (ActualDelay>MaxCyclesBetweenLEDUpdate))
			{
				//clear send request
				LedDataSendRequest=0;
			
				//update led
				update_led_values();
			
				//save current time
				LedDataLastSend=TIM0_ReadTCNT0();
			}
		#endif
		
		update_SOC(counterTIM0);
	}
}