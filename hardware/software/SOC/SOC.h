/*
 * SOC.h
 *
 * Created: 18/07/2018 07:52:36
 *  Author: Marcus
 */ 


#ifndef SOC_H_
#define SOC_H_

#include "../ADC/ADC.h"

#define PIUSB PA5ADC		// pin PA5
#define MOTORUSB PA6ADC		// pin PA6
#define CHARGEUSB PA7ADC	// pin PA7

#define FREQSAMPLEMAX 10
#define SECSAMPLEMAX 60

 // Used for diagnostic purposes for the ADC
 
#define PI_USB_OC 0
#define MOTOR_USB_OC 1
#define CHARGE_USB_OC 2
extern uint8_t soc_diag;
 

 // State machine for understanding the position of the duckiebot    
 typedef enum {
	 STANDING_STILL = 1,
	 MOVING = 2,
	 MOVING_AND_CHARGING = 3,
	 STANDING_STILL_AND_CHARGING = 4,
	 UNKNOWN_STATE = 5,
 }POS_STATE;
 
extern POS_STATE positionState;

// Flag used to indicate to the duckiebot that it need to return to the charging station
extern uint8_t needToCharge;

// Holds the state of charge for the battery
// Preconditions:
// The first time a duckiebot is used it has to have a 100% full battery
extern uint16_t fuelGauge;	// holds the total charge in mAh

// Percentage if the max change where we set the flag needToCharge
extern uint16_t minChargeLimit;

// Battery dependent 
extern uint16_t maxCharge;

// Current thresholds to determined over current of the duckie  
extern uint16_t maxPiCurrent;
extern uint16_t maxMotorCurrent;
extern uint16_t maxChargeCurrent;

// Current thresholds to determined the state of the duckie  
extern uint8_t piCurrPosLimit;
extern uint8_t motorCurrPosLimit;
extern uint8_t chargeCurrPosLimit;

// Time for how much longer we shall charge after we reach 10 000mA
extern uint16_t fullChargeMAXTime;

extern void init_SOC( void );
extern void update_SOC( uint16_t );
extern void update_SOC_limits( void );
extern void accumulator(uint8_t *, int32_t *  , int32_t * );
extern void update_fuel_gauge ( int32_t * );
extern void update_position(uint16_t , uint16_t , uint16_t );
extern void update_diag(uint16_t , uint16_t , uint16_t );
extern void update_full_charge_timer( void );

#endif /* SOC_H_ */