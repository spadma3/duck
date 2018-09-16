/*
 * Motor.c
 *
 * Created: 23/07/2018 15:21:13
 *  Author: Marcus
 */ 
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include<stdio.h>
#include<stdlib.h>
#include "Motor.h"
#include "../i2c/i2c_machine.h"


void init_motor_outputs(void)
{
	//Configure Data Direction of Output Port to output
	DDRB |= (1<<PORTINDEX_PWMA);
	DDRA |= (1<<PORTINDEX_AIN2);
	DDRA |= (1<<PORTINDEX_AIN1);
	DDRB |= (1<<PORTINDEX_BIN1);
	DDRB |= (1<<PORTINDEX_BIN2);
	DDRB |= (1<<PORTINDEX_PWMB);

	//PWM Configuration
	
	//set Fast PWM Mode
	TCCR1D &= ~(1<<WGM10);
	TCCR1D &= ~(1<<WGM11);
	
	//Activate Pulse With Modulator B
	TCCR1A |= (1<<PWM1B);
	
	//OCW1A Behavior: Clear on CompareMatch, Set when TCNT1=0x000, Connect just OC1B
	TCCR1A |= (1<<COM1B1);
	TCCR1A &= ~(1<<COM1B0);

	//Activate Pulse With Modulator D
	TCCR1C |= (1<<PWM1D);
	
	//OCW1A Behavior: Clear on CompareMatch, Set when TCNT1=0x000, Connect just OC1D
	TCCR1C |= (1<<COM1D1);
	TCCR1C &= ~(1<<COM1D0);

	//Set the prescaler to 1/16(0101b)--> 8Mhz System Clock --> 500kHz Counter Clock --> 0-255 Counting ca 2kHz
	TCCR1B |= (1<<CS10); //Warning: This Counter is also used for calculating the time for updating the LED (see main loop)!
	TCCR1B &= ~(1<<CS11);
	TCCR1B |= (1<<CS12);
	TCCR1B &= ~(1<<CS13);
	
	//Set PWM Counter Top Value to 255
	OCR1C  =0xFF;
}

inline void update_motor_values(void)
{
	if(getI2CPWMValue(I2C_SLAVE_MOTOR,I2CPWMCH_AIN1)>=128)
	{
		PORTA |= (1<<PORTINDEX_AIN1);//set output
	}
	else
	{
		PORTA &= ~(1<<PORTINDEX_AIN1);//reset output
	}

	if(getI2CPWMValue(I2C_SLAVE_MOTOR,I2CPWMCH_AIN2)>=128)
	{
		PORTA |= (1<<PORTINDEX_AIN2);//set output
	}
	else
	{
		PORTA &= ~(1<<PORTINDEX_AIN2);//reset output
	}
	
	if(getI2CPWMValue(I2C_SLAVE_MOTOR,I2CPWMCH_BIN1)>=128)
	{
		PORTB |= (1<<PORTINDEX_BIN1);//set output
	}
	else
	{
		PORTB &= ~(1<<PORTINDEX_BIN1);//reset output
	}

	if(getI2CPWMValue(I2C_SLAVE_MOTOR,I2CPWMCH_BIN2)>=128)
	{
		PORTB |= (1<<PORTINDEX_BIN2);//set output
	}
	else
	{
		PORTB &= ~(1<<PORTINDEX_BIN2);//reset output
	}

	//set PWM Values
	PWMOutput_PWMA = getI2CPWMValue(I2C_SLAVE_MOTOR,I2CPWMCH_PWMA);
	PWMOutput_PWMB = getI2CPWMValue(I2C_SLAVE_MOTOR,I2CPWMCH_PWMB);

}