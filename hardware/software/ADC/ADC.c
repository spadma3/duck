/*
 * ADC.c
 *
 * Created: 17/07/2018 
 *  Author: Marcus
 *	Description: Boilerplate code for setting up the ADC and a function for calculating the current through a shunt resistor 
 */ 
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include<stdio.h>
#include<stdlib.h>
#include "ADC.h"
 
uint16_t adc_diag;

// initialization function for ADC 
void init_ADC( void )
{
	// init ADC set pins as input
	DDRA &= ~(1<<DDA5); // PA5 Charging current ADC
	DDRA &= ~(1<<DDA6); // PA6 Motor current ADC
	DDRA &= ~(1<<DDA7); // PA7 RasPi current ADC
	
	// Section 15.13.1
	// Set voltage reference to Vcc, right adjust the results and Single-Ended Input 
	//NB: "The internal voltage reference options may not be used if an external voltage is being applied to the AREF pin." 
	
	ADMUX &= ~( (1<<REFS1) | (1<< REFS0) | (1<<ADLAR) );
	
	// Table 15-4
	ADMUX |=  (PA5ADC) ;
	
	// Section 15.13.4
	// Gain selected to 1, 	Free Running mode,
	ADCSRB	&=  ~( (1<<BIN) | (1<<GSEL) | (1<<REFS2) | (1<<MUX5) | (1<<ADTS2) | (1<<ADTS1) | (1<<ADTS0) );
	
	// Section 15.13.5
	// Turn of digital input on ADC6-ADC4 to reduce power consumption
	DIDR0 |= ( (1<<ADC6D) | (1<<ADC5D) | (1<<ADC4D) );
	
	// Section 15.13.2
	// Enable the ADC , free-running mode, interrupt with /64 prescaler i.e 125k
	ADCSRA = ( (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1)) ;//| (1<<ADIE) | (1<<ADATE) );
	
	adc_diag = 0;
		
}
// Get ADC reading from 0 - 1024 of a specific channel 
uint16_t read_ADC(uint8_t channel)
{
	uint16_t res = 0;
	if ( (channel == PA5ADC) || (channel == PA6ADC) || (channel == PA7ADC) )// just make sure we read from a defined ADC
	{
		ADMUX = channel; // set voltage reference to Vcc, right adjust the results and Single-Ended Input
		ADCSRA = (1<<ADEN) | (1<<ADSC);
		asm volatile ("NOP" ::);
		asm volatile ("NOP" ::);
		while ( ADCSRA & ( 1 << ADSC ) );
		//NB: need to store result in intermediate variable!
		uint8_t result_l = ADCL;
		uint8_t result_h = ADCH;
		res = (result_h << 8) | result_l;
	}
	else
	{
		adc_diag |= (1<<DIAG_UNDEF_CHANNEL);
	}
	return res; 

}
// Get current trough a specific shunt with result in mA
uint16_t read_current(uint8_t channel )
{
	
	uint32_t  steps = read_ADC(channel);
	
	uint32_t pinVolatge = (steps * STEP_VOLTAGE); // voltage at pin
	
	uint32_t shuntVoltage = pinVolatge / AMP_GAIN; // voltage over shunt
	
	uint16_t shuntcurrent = (uint16_t) ( (shuntVoltage) / SHUNT_RES); //  A = V/R current through shunt in mA 
	
	return shuntcurrent;	
}



