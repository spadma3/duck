/*
 * ADC.h
 *
 * Created: 17/07/2018 11:44:54
 *  Author: Marcus
 */ 


#ifndef ADC_H_
#define ADC_H_
 
#define PA5ADC 4 // pin PA5
#define PA6ADC 5 // pin PA6
#define PA7ADC 6 // pin PA7

#define AMP_GAIN 200		// 200 times the gain
#define SHUNT_RES 2			// 2mOhm shunt resistor
#define STEP_VOLTAGE 3222	// 3.3V / 1024 = 3222uV

 // Used for diagnostic purposes for the ADC
#define DIAG_UNDEF_CHANNEL 0
#define DIAG_PA5_OV 1
#define DIAG_PA6_OV 2
#define DIAG_PA7_OV 3
extern uint16_t adc_diag;

// init ADC 
void init_ADC( void );
uint16_t read_ADC(uint8_t );
uint16_t read_current(uint8_t );
void update_ADC_limits( void );


#endif /* ADC_H_ */