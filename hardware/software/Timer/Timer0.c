/*
 * Timer0.c
 *
 * Created: 23/07/2018 16:14:47
 *  Author: Marcus
 */ 
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

void TIM0_Init(void)
{
	//Set Conter0 to 16Bit Mode, disable InputCapture
	TCCR0A = 0b10000000;

	//Set internal Clock and Prescaler to 256 --> 8MHz/256=31.25kHz --> Overflow 31.25kHz/65536 = every 2 sec
	TCCR0B = 0b100;
}

uint16_t TIM0_ReadTCNT0(void)
{
	unsigned char sreg;
	uint16_t i;
	/* Save global interrupt flag */
	sreg = SREG;
	/* Disable interrupts */
	cli();
	/* Read TCNT0 into i */
	i = TCNT0L;
	i |= ((unsigned int)TCNT0H << 8);
	/* Restore global interrupt flag */
	SREG = sreg;
	return i;
}