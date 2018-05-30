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

#define FirstRegisterAddress 0x26 //The LED0_ON_L Register on the PCA9685 is at 0x06
#define BytesPerLedPwmChannel 0x04 //For each LED, 4 Byte of date are used in the PCA9685


#define I2C_SLAVE_LED 0
#define I2C_SLAVE_MOTOR 1
volatile uint8_t I2C_SLAVE_ADDR[I2C_N_SLAVES];
volatile uint8_t i2c_reg[I2C_N_SLAVES][I2C_N_REG];



volatile uint8_t led_data[N_LEDS*3];
const uint8_t init_color[3] PROGMEM = { 0x00, 0x00, 0xFF};


volatile uint16_t motor_data[2];
volatile uint16_t h_data[4];
volatile uint16_t h_data_old[4];

static inline void set_leds_global(void)
{
       ws2812_setleds_constant((struct cRGB *)&REG_GLB_G, N_LEDS);
}

static inline void update_leds(void)
{
       ws2812_sendarray(led_data, N_LEDS * 3);
}

static inline char getValue(int PWMChannel)//from 0 to 15
{
       if (PWMChannel>=0 && PWMChannel<=15)
       {
              int RegisterStartAddress=PWMChannel*BytesPerLedPwmChannel+FirstRegisterAddress;//LEDx_ON_L is in register 06h+x*4
              int16_t iOn=(i2c_reg[I2C_SLAVE_LED][RegisterStartAddress+1]<<8) + i2c_reg[I2C_SLAVE_LED][RegisterStartAddress];
              int16_t iOff=(i2c_reg[I2C_SLAVE_LED][RegisterStartAddress+3]<<8) + i2c_reg[I2C_SLAVE_LED][RegisterStartAddress+2];
              int16_t iOnTime=iOff-iOn;//can be from -4095 to +4095
              if (iOnTime<0)
              {
                    iOnTime=iOnTime+4096; //is now from 0 to +4095
              }
              return (char) (iOnTime>>4); //is now from 0 to 255
       }
       else
       {
              return 0;
       }
}

static inline char getMotorValue(int addr)
{
       if (addr>=0 && addr <=69)
       {
              int16_t iOn=(i2c_reg[I2C_SLAVE_MOTOR][addr+1]<<8) + i2c_reg[I2C_SLAVE_MOTOR][addr];
              int16_t iOff=(i2c_reg[I2C_SLAVE_MOTOR][addr+3]<<8) + i2c_reg[I2C_SLAVE_MOTOR][addr+2];
              int16_t iOnTime=iOff-iOn;//can be from -4095 to +4095
              if (iOnTime<0)
              {
                    iOnTime=iOnTime+4096; //is now from 0 to +4095
              }
              return (char) (iOnTime>>4); //is now from 0 to 255
       }
       else
       {
              return 0;
       }
}

static inline void calc_led_values(void)
{
	char i=0;
	for (i=0;i<N_LEDS;i++)
	{
		//the analog led were common anonde led. this means, that the negative pad was switched. at a value of 255, the led was dark, at 0 it was bright. the addressable led are the other way arround: 0=dark, 255= bright. Thats why we need to substract 255 from the value we calculate!
		led_data[3*i+1]=255-getValue(i*3+0);
		led_data[3*i]=255-getValue(i*3+1);
		led_data[3*i+2]=255-getValue(i*3+2);
	}
}





static inline uint8_t getHValue(int address)
{
        uint8_t bit_getter = 0x10;
        //if(i2c_transaction_ongoing())
        volatile uint8_t  led_full_on =  i2c_reg[I2C_SLAVE_MOTOR][address +1] & bit_getter  ;
        volatile uint8_t led_full_off =  i2c_reg[I2C_SLAVE_MOTOR][address+3] &  bit_getter  ;
        if(led_full_on==16 && led_full_off!=16)
		{
                return (uint8_t) 1;
        }
        else
		{
                return (uint8_t) 0;
        }
}


static inline void calc_motor_values(void){
	if (i2c_check_stop(I2C_SLAVE_MOTOR))
	{
        PORTB &= ~ ( 1 << PORTB6);
        PORTB &= ~ ( 1 << PORTB4);
        PORTA &= ~ ( 1 << PORTA3) ;
        PORTA &= ~ ( 1 << PORTA4);
        motor_data[0] = getMotorValue(FirstRegisterAddress);
        motor_data[1] = getMotorValue(FirstRegisterAddress+20);
        h_data[0] = getHValue(FirstRegisterAddress + 8);  //AIN1 - PA3
        h_data[1] = getHValue(FirstRegisterAddress + 4);  //AIN2 - PA4
        h_data[2] = getHValue(FirstRegisterAddress + 12); //BIN1 - PB4
        h_data[3] = getHValue(FirstRegisterAddress + 16); //BIN2 - PB6
        PORTB |= (h_data[3] << PB6);    //PWMB 
//      _delay_ms(50);
        PORTB |= (h_data[2] << PB4);
//      PORTB |= (h_data[3] << PB6);
//       _delay_ms(50);
        PORTA |= (h_data[0] << PA3);
//      _delay_ms(50);
        PORTA |=  (h_data[1] << PA4);    //PWMA         
/*
        // Definiere TOP count
        //OCR1C  |=0xFF;*/
        OCR1B = 255 - motor_data[0];
        OCR1D = 255 - motor_data[1];
        return;
	}
	else
	{
        return;
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



static inline void calc_led(void)
{
	if (i2c_check_stop(I2C_SLAVE_LED))
	{
		/*if (REG_CTRL & CTRL_RST)
		do_reset();
		else if (REG_CTRL & CTRL_GLB)
		set_leds_global();
		else
		{*/
		calc_led_values();	
		update_leds();
		
		//}
	}
	
}






int main(void)
{
	
	I2C_SLAVE_ADDR[I2C_SLAVE_LED]=0x40;
	I2C_SLAVE_ADDR[I2C_SLAVE_MOTOR]=0x60;
	
			
		DDRB = (1 << 3);
        // Unsere Modi werden gesetzt
//      PCMSK0 = 255;
//      PCMSK1 = 255;
        DDRB |= (1<<DDB3)|(1<<DDB5)|(1<<DDB6)|(1<<DDB4);
        DDRA |= (1<<DDA4)|(1<<DDA6);
        TCCR1A |= (1<<PWM1A)|(1<<PWM1B)|(1<<COM1B1)|(1<<COM1B0)|(1<<COM1A1)|(1<<COM1A0);
        TCCR1D |= (0<<WGM11)|(0<<WGM10);
        TCCR1B |= (0<<CS13)|(1<<CS12)|(0<<CS11)|(1<<CS10);
        TCCR1C |= (1<<COM1D1) | (1<<COM1D0)|(1<<PWM1D) ;
        OCR1C  =0xFF;
       //enable second USI Port (PA0:2)
       USIPP=0x01;
       i2c_init();
       sei();
       while(1)
       {
                 //calc_motor_values();
				 //calc_led();
				 i2c_check_stop(I2C_SLAVE_LED);
				 i2c_check_stop(I2C_SLAVE_MOTOR);
				 calc_led_values();
				 //update_leds();
       }
}