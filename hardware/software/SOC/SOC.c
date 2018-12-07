/*
 * SOC.c
 *
 * Created: 18/07/2018 07:52:10
 *  Author: Marcus
 */ 
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include<stdio.h>
#include<stdlib.h>
#include "SOC.h"

// extern global variables
uint8_t needToCharge;
uint16_t fuelGauge;
uint16_t minChargeLimit;
uint16_t maxCharge;
uint16_t maxPiCurrent;
uint16_t maxMotorCurrent;
uint16_t maxChargeCurrent;
uint8_t piCurrPosLimit;		
uint8_t motorCurrPosLimit;	
uint8_t chargeCurrPosLimit;

POS_STATE positionState;
uint8_t soc_diag;
uint16_t fullChargeMAXTime;

// local global variables
uint16_t lastCountValue; 
// Accumulators
int32_t secAccumulators;	// count mA in a second
int32_t mSecAccumulators;	// count mA in a 100ms intervals
uint16_t updateFreq;		// at what frequency shall we sample the ADC
uint16_t freqCounter;
uint8_t freqSampleCounter;	// count to 10
uint8_t	freqSampleSec;		// count to 60
// Time for how much longer we shall charge after we reach 10 000mA
uint16_t fullChargeTimer;

void sample_current( void );	

void init_SOC( void ) 
{
	needToCharge = 0;
	maxCharge = 10000;
	fuelGauge = maxCharge; 
	minChargeLimit = 2000;	// 20%
	lastCountValue = 0;
	updateFreq = 3125;		// Sys clk = 8MHz, prescale = 256 => timer0 = 31.25kHz => we want 10 saples/sec
	freqCounter = 0;
	freqSampleCounter = 0;
	freqSampleSec = 0;
	
	maxPiCurrent = 2200;		// Based on load test
	maxMotorCurrent  = 2700;	// Based on load test
	maxChargeCurrent = 2500;
	
	piCurrPosLimit = 100;		// 
	motorCurrPosLimit  = 100;	//
	chargeCurrPosLimit = 50;	// 
	
	fullChargeMAXTime = 1200;	// 20 min
	
	soc_diag = 0; 
	positionState = UNKNOWN_STATE;
	
}

// State machine for updating the fuel gauge
// TODO make the sampling dependent of the selected updateFreq
void update_SOC( uint16_t counter )
{
	freqCounter += abs(counter - lastCountValue);
	lastCountValue = counter;
	
	if(freqCounter>= updateFreq) // sample rate at 100ms
	{
		update_SOC_limits(); // limit the update freq slightly 
		freqCounter = 0;
		freqSampleCounter++;
		sample_current();	
	}
	
	if(freqSampleCounter >= FREQSAMPLEMAX) // sample rate at sec
	{
		freqSampleCounter = 0;
		accumulator(&freqSampleSec, &secAccumulators, &mSecAccumulators);
		update_full_charge_timer();
	}
	
	if (freqSampleSec >= SECSAMPLEMAX) // sample every min 
	{
		freqSampleSec = 0;
		update_fuel_gauge(&secAccumulators);
	}
}

void update_SOC_limits ( void )
{
	//TODO: check if Pi has send new limits. i.e maximum charge of battery or minimum charge limit 
	/*
	maxPiCurrent = ;
	maxMotorCurrent  = ;
	maxChargeCurrent = ;
	
	minChargeLimit = ;
	maxCharge = ;
	
	piCurrPosLimit = ;
	motorCurrPosLimit = ;
	chargeCurrPosLimit = ;
	
	fullChargeMAXTime = ;
	
	// Clear faults
	soc_diag = ;
	*/
}

void sample_current( void )
{
	uint16_t piUSB = read_current(PIUSB);
	uint16_t motorUSB = read_current(MOTORUSB);
	uint16_t chargeUSB = read_current(CHARGEUSB);
	
	update_diag(piUSB, motorUSB, chargeUSB);
	update_position(piUSB, motorUSB, chargeUSB);
	mSecAccumulators = mSecAccumulators - piUSB - motorUSB + chargeUSB;
}
void accumulator(uint8_t *counter, int32_t *accum , int32_t *sample )
{
	(*counter)++;
	(*accum) += (*sample);
	(*sample) = 0;
}

void update_fuel_gauge ( int32_t *sample )
{
	(*sample) = (*sample)/60;					// change from mA/sec to mA/min
	(*sample) = (*sample)/60;					// change from mA/min to mA/h
	
	int16_t accCharge  = (int16_t)(*sample);   // cast to int16 
	(*sample) = 0;
	
	fuelGauge += accCharge;  // note that is could be  + (-accCharge)

	if (fuelGauge >= maxCharge)
	{
		fuelGauge = maxCharge;
		
		if (fullChargeTimer >= fullChargeMAXTime)
		{
			needToCharge = 0;
			fullChargeTimer = 0;
		}	
	}
	
	if (fuelGauge <= minChargeLimit)
	{
		needToCharge = 1;
	}
} 

void update_position(uint16_t piCurrent, uint16_t motorCurrent, uint16_t chargeCurrent)
{
	
	uint8_t state = 0;
	
	if (piCurrent > piCurrPosLimit) // the Pi is ON
	{	
		state |= 1<<0;
	}
	if (motorCurrent > motorCurrPosLimit) // the motor is ON
	{
		state |= 1<<1;
	}
	if (chargeCurrent > chargeCurrPosLimit) // charging is ON
	{
		state |= 1<<2;
	}
	
	switch (state) 
	{
		case 1:
			positionState = STANDING_STILL;
		break;
		
		case 3:
			positionState = MOVING;
		break;
		
		case 4: // charging when the Pi is off? should work
		case 5:
			positionState = STANDING_STILL_AND_CHARGING;
		break;
		
		case 7:
			positionState = MOVING_AND_CHARGING;
		break;
		
		default: // can not be moving ( and charging)  without the pi case 2 and 6 
			positionState = UNKNOWN_STATE;
	}
	
}

void update_diag(uint16_t piCurrent, uint16_t motorCurrent, uint16_t chargeCurrent)
{
	if (piCurrent > maxPiCurrent) 
	{
		soc_diag |= 1<<PI_USB_OC;
	}
	if (motorCurrent > maxMotorCurrent) 
	{
		soc_diag |= 1<<MOTOR_USB_OC;
	}
	if (chargeCurrent > maxChargeCurrent)
	{
		soc_diag |= 1<<CHARGE_USB_OC;
	}
}

void update_full_charge_timer( void ) // calls with sec interval 
{
	if ( (fuelGauge >= maxCharge) && (positionState == STANDING_STILL_AND_CHARGING) )
	{
		fullChargeTimer++;
	}

}
