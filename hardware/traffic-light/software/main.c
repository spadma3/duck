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

#define FirstRegisterAddress 0x06 //The LED0_ON_L Register on the PCA9685 is at 0x06
#define BytesPerLedPwmChannel 0x04 //For each LED, 4 Byte of date are used in the PCA9685

#define I2C_SLAVE_LED 0
#define I2C_SLAVE_MOTOR 1

const uint8_t led_remappings[N_LEDS] = LED_REMAPS; // i2c/defined in i2c_slave_defs.h

volatile uint8_t I2C_SLAVE_ADDR[I2C_N_SLAVES]={0x40,0x60};
volatile uint8_t i2c_reg[I2C_N_SLAVES][I2C_N_REG];

volatile uint8_t led_data[N_LEDS*3];


static inline void set_leds_global(void)
{
       ws2812_setleds_constant((struct cRGB *)&REG_GLB_G, N_LEDS);
}


//calculates the PWM Value (0-255) from a PWM Channel of the PCA9685 emlulated slave
static inline uint8_t getI2CPWMValue(uint8_t Slave, uint8_t PWMChannel)//from 0 to 15
{
	if (PWMChannel>=0 && PWMChannel<=15)
	{
		int RegisterStartAddress=PWMChannel*BytesPerLedPwmChannel+FirstRegisterAddress;//LEDx_ON_L is in register x*4+6
		int16_t iOn=(i2c_reg[Slave][RegisterStartAddress+1]<<8) + i2c_reg[Slave][RegisterStartAddress];
		int16_t iOff=(i2c_reg[Slave][RegisterStartAddress+3]<<8) + i2c_reg[Slave][RegisterStartAddress+2];
		int16_t iOnTime=iOff-iOn;//can be from -4095 to +4095
		if (iOnTime<0)
		{
			iOnTime=iOnTime+4096; //is now from 0 to +4095
		}
		if (iOnTime>4095)
		{
			iOnTime=4095;
		}
		uint8_t ret = (iOnTime>>4); //is now from 0 to 255
		return  ret;
	}
	else
	{
		return 0;
	}
}

//Calculates all values for the led's and send values
static inline void update_led_values(void)
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



//Channel Description Motor
//PWM8	PWMA	OC1B/PB3
//PWM9	AIN2	PA4
//PWM10	AIN1	PA3
//PWM11	BIN1	PB4
//PWM12	BIN2	PB6
//PWM13 PWMB	OC1D/PB5

#define I2CPWMCH_PWMA 8
#define I2CPWMCH_AIN2 9
#define I2CPWMCH_AIN1 10
#define I2CPWMCH_BIN1 11
#define I2CPWMCH_BIN2 12
#define I2CPWMCH_PWMB 13

#define PORT_PWMA PORTB
#define PORT_AIN2 PORTA
#define PORT_AIN1 PORTA
#define PORT_BIN1 PORTB
#define PORT_BIN2 PORTB
#define PORT_PWMB PORTB

#define DDR_PWMA DDRB
#define DDR_AIN2 DDRA
#define DDR_AIN1 DDRA
#define DDR_BIN1 DDRB
#define DDR_BIN2 DDRB
#define DDR_PWMB DDRB

#define PORTINDEX_PWMA 3
#define PORTINDEX_AIN2 4
#define PORTINDEX_AIN1 3
#define PORTINDEX_BIN1 4
#define PORTINDEX_BIN2 6
#define PORTINDEX_PWMB 5

#define PWMOutput_PWMA OCR1B
#define PWMOutput_PWMB OCR1D

static inline void update_motor_values(void)
{
	if(getI2CPWMValue(I2C_SLAVE_MOTOR,I2CPWMCH_AIN1)>=128)
	{
		PORT_AIN1 |= (1<<PORTINDEX_AIN1);//set output
	}
	else
	{
		PORT_AIN1 &= ~(1<<PORTINDEX_AIN1);//reset output
	}

	if(getI2CPWMValue(I2C_SLAVE_MOTOR,I2CPWMCH_AIN2)>=128)
	{
		PORT_AIN2 |= (1<<PORTINDEX_AIN2);//set output
	}
	else
	{
		PORT_AIN2 &= ~(1<<PORTINDEX_AIN2);//reset output
	}

	if(getI2CPWMValue(I2C_SLAVE_MOTOR,I2CPWMCH_BIN1)>=128)
	{
		PORT_BIN1 |= (1<<PORTINDEX_BIN1);//set output
	}
	else
	{
		PORT_BIN1 &= ~(1<<PORTINDEX_BIN1);//reset output
	}

	if(getI2CPWMValue(I2C_SLAVE_MOTOR,I2CPWMCH_BIN2)>=128)
	{
		PORT_BIN2 |= (1<<PORTINDEX_BIN2);//set output
	}
	else
	{
		PORT_BIN2 &= ~(1<<PORTINDEX_BIN2);//reset output
	}

	//set PWM Values
	PWMOutput_PWMA=getI2CPWMValue(I2C_SLAVE_MOTOR,I2CPWMCH_PWMA);
	PWMOutput_PWMB=getI2CPWMValue(I2C_SLAVE_MOTOR,I2CPWMCH_PWMB);

}


static inline void init_motor_outputs(void)
{
	//Configure Data Direction of Output Port to output
	DDR_PWMA |= (1<<PORTINDEX_PWMA);
	DDR_AIN2 |= (1<<PORTINDEX_AIN2);
	DDR_AIN1 |= (1<<PORTINDEX_AIN1);
	DDR_BIN1 |= (1<<PORTINDEX_BIN1);
	DDR_BIN2 |= (1<<PORTINDEX_BIN2);
	DDR_PWMB |= (1<<PORTINDEX_PWMB);


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

static inline void TIM0_Init(void)
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



	//enable second USI Port (PA0:2)
	USIPP=0x01;
	i2c_init();

	//init motor outputs
	init_motor_outputs();

	//init Timer0
	TIM0_Init();

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
		ActualDelay=TIM0_ReadTCNT0()-LedDataLastSend;

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
	}
}
