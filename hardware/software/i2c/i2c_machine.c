/*
 * Copyright Brian Starkey 2014 <stark3y@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

//#define DEBUG

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include "i2c_slave_defs.h"
#include "i2c_machine.h"

#define I2C_SDA_DIR_OUT 1
#define I2C_SDA_DIR_IN 0

#define NAK() USIDR = 0x80
#define ACK() USIDR = 0x00

#define I2C_STATE_ADDR_MATCH   0
#define I2C_STATE_REG_ADDR     1
#define I2C_STATE_MASTER_READ  2
#define I2C_STATE_MASTER_WRITE 3
#define I2C_STATE_IDLE         4

uint8_t volatile i2c_reg[I2C_N_SLAVES][I2C_N_REG];
volatile uint8_t I2C_SLAVE_ADDR[I2C_N_SLAVES]={0x40,0x60};

// #ifdef DEBUG
// #define LED_ON() PORTB |= 0x2
// #define LED_OFF() PORTB &= ~0x2
// #define LED_FLICKER() LED_OFF(); LED_ON()
// #else
// #define LED_ON()
// #define LED_OFF()
// #define LED_FLICKER()
// #endif

volatile uint8_t i2c_update[I2C_N_SLAVES] = {0}; //byte count written in the last i2c write command
volatile uint8_t i2c_current_Slave=0; //current slave index, which is communicating at the moment, 0xFF means no slave!
 
 
//these variables are just there once and are used all emulated slaves.
//because there is just one state machine and the i2c master can just
//talk to one slave at a time.
volatile uint8_t i2c_state = 0;  //state of the i2c state machine, see below  
volatile uint8_t i2c_offset = 0; //read or write array index of the current operation



//this function translates an I2C register index to an array index (e.g. to save memory because there could be gaps in the i2c address registers)
//PCA9685 has registers 0-69dez and 250-255dez. To Save memory, the registers 250-255dez are mapped to 70-75! This is achieved with this function.
uint16_t I2CAddressToArrayIndex(uint8_t I2CAddress) //Error= 0xFFFF
{
	if (I2CAddress <= 0x45)//i2c 0-69dez --> Index 0-69dez
	{
		return I2CAddress;
	}
	else if (I2CAddress >= 0xFA && I2CAddress <= 0XFF)// i2c 250-255dez --> Index 70-25dez
	{
		return I2CAddress - 0xFA+0x46;// the address 0xFA (250dez) should give 0x46 (70dez)
	}
	else
	{
		return 0xFFFF;//Error
	}
}





/* USI i2c Slave State Machine
 * ===========================
 *
 * 5 States:
 *     0 I2C_STATE_ADDR_MATCH
 *       Waiting for address (start)
 *
 *     1 I2C_STATE_REG_ADDR
 *       Receive register address*
 *
 *     2 I2C_STATE_MASTER_READ
 *       Transmit data to master
 *
 *     3 I2C_STATE_MASTER_WRITE
 *       Receive data from master
 *
 *     4 I2C_STATE_IDLE
 *       Bus idle/address not matched
 *
 * Valid state transitions:
 *      __To__________
 *      0  1  2  3  4
 * F 0|    a  b     h
 * r 1|          d  ci
 * o 2|       f     e
 * m 3|          g  c
 *   4| j
 *
 * Transition j - Start of transaction
 *  I2C_STATE_IDLE -> I2C_STATE_ADDR_MATCH
 *  Cond:   Start condition interrupt
 *  Action: None.
 *
 * Transition h - Address not matched.
 *  I2C_STATE_ADDR_MATCH -> I2C_STATE_IDLE
 *  Cond:   Pre-ack. Address doesn't match
 *  Action: NAK.
 *
 * Transition a - Address matched, write mode
 *  I2C_STATE_ADDR_MATCH -> I2C_STATE_REG_ADDR
 *  Cond:   Pre-ack. Address matches, bit0 == 0
 *  Action: ACK, Reset reg pointer.
 *
 * Transition b - Address matched, read mode
 *  I2C_STATE_ADDR_MATCH -> I2C_STATE_MASTER_READ
 *  Cond:   Pre-ack. Address matches, bit0 == 1
 *  Action: ACK.
 *
 * Transition c - Write finished
 *  I2C_STATE_XXX -> I2C_STATE_IDLE
 *  Cond:   Stop flag is set.
 *  Action: None.
 *
 * Transition d - Initialise write
 *  I2C_STATE_REG_ADDR -> I2C_STATE_MASTER_WRITE
 *  Cond:   Pre-ack.
 *  Action: ACK, reg_ptr = USIDR.
 *
 * Transition i - Invalid reg addr
 *  I2C_STATE_REG_ADDR -> I2C_STATE_IDLE
 *  Cond:   Pre-ack, USIDR > N_REG - 1
 *  Action: NAK.
 *
 * Transition e - Read finished
 *  I2C_STATE_MASTER_READ -> I2C_STATE_IDLE
 *  Cond:   Post-ack. Master NAK'd.
 *  Action: None.
 *
 * Transition f - Read continues
 *  I2C_STATE_MASTER_READ -> I2C_STATE_MASTER_READ
 *  Cond:   Post-ack. Master ACK'd.
 *  Action: USIDR = *reg_ptr++
 *
 * Transition g - Write continues
 *  I2C_STATE_MASTER_WRITE -> I2C_STATE_MASTER_WRITE
 *  Cond:   Pre-ack.
 *  Action: ACK, *reg_ptr++ = USIDR
 *
 */

 /*
  * For some reason, avr-libc uses different vector names for the USI
  * on different chips! We have to workaround that here
  */





//I2C Start Interrupt
#if defined(USI_START_vect)
ISR(USI_START_vect)
#elif defined(USI_STRT_vect)
ISR(USI_STRT_vect)
#else
#error "Couldn't figure out what i2c start interrupt to use!"
#endif
{
	i2c_state = 0;
	while (USI_PIN & (1 << I2C_SCL));
	USISR = 0xF0;
}


//I2C Overflow Interrupt
#if defined(USI_OVERFLOW_vect)
ISR(USI_OVERFLOW_vect)
#elif defined(USI_OVF_vect)
ISR(USI_OVF_vect)
#else
#error "Couldn't figure out what i2c overflow interrupt to use!"
#endif
{
	static uint8_t post_ack = 0;
	/* Writing USISR directly has side effects! */
	uint8_t usisr_tmp = 0xD0;
	uint8_t sda_direction;
	uint8_t tmp;

	if (!post_ack) {
		/* Work that needs to be done before the ACK cycle */
		sda_direction = I2C_SDA_DIR_OUT;

		switch (i2c_state) {
		case I2C_STATE_ADDR_MATCH:
			tmp = USIDR >> 1;//tmp=slave address
			
			i2c_current_Slave=0xFF;//set slave address invalid
			
			
			//check if the actual slave is in the slave address array
			for(uint8_t i=0;i<I2C_N_SLAVES;i++)
			{
				if (I2C_SLAVE_ADDR[i]==tmp)
				{
					//if slave address is found, stop searching and save index
					i2c_current_Slave=i;
					break;
				}
			}
			
			//Slave is found if current address is not initial value and temp is not 0			
			if (tmp==0 || i2c_current_Slave==0xFF) {
				/* Transition h: Address not matched */
				i2c_state = I2C_STATE_IDLE;
				NAK();
			} else {
				if (USIDR & 1) {
					/* Transition b: Address matched, read mode */
					i2c_state = I2C_STATE_MASTER_READ;
				} else {
					/* Transition a: Address matched, write mode */
					i2c_offset = 0;
					i2c_state = I2C_STATE_REG_ADDR;
					i2c_update[i2c_current_Slave] = 1;
				}
				ACK();
			}
			break;
		case I2C_STATE_REG_ADDR:
			if (I2CAddressToArrayIndex(USIDR)==0xFFFF) {
				/* Transition i:  Invalid reg addr*/
				i2c_state = I2C_STATE_IDLE;
				NAK();
			} else {
				/* Transition d:  Initialise write*/
				i2c_offset = I2CAddressToArrayIndex(USIDR);
				i2c_state = I2C_STATE_MASTER_WRITE;
				ACK();
			}
			break;
		case I2C_STATE_MASTER_READ:
			USIDR = 0;
			/* Listen for master NAK */
			sda_direction = I2C_SDA_DIR_IN;
			break;
		case I2C_STATE_MASTER_WRITE:
#if defined(I2C_GLOBAL_WRITE_MASK)
			tmp = I2C_GLOBAL_WRITE_MASK;
#else
			tmp = i2c_w_mask[i2c_offset];
#endif
			if (tmp) {
				/* Only heed writeable bits */
				i2c_reg[i2c_current_Slave][i2c_offset] &= ~tmp;
				i2c_reg[i2c_current_Slave][i2c_offset] |= USIDR & tmp;
			}
			i2c_update[i2c_current_Slave]++;
			i2c_offset++;
			ACK();
			break;
		default:
			NAK();
		}
		/* Counter will overflow again after ACK cycle */
		usisr_tmp |= 14 << USICNT0;
		post_ack = 1;
	} else {
		/* Work that needs to be done after the ACK cycle */
		sda_direction = I2C_SDA_DIR_IN;
		switch (i2c_state) {
		case I2C_STATE_MASTER_READ:
			if (USIDR) {
				/* Transition e: Read finished */
				i2c_offset = 0;
				i2c_state = I2C_STATE_IDLE;
			} else {
				/* Transition f: Read continues */
				sda_direction = I2C_SDA_DIR_OUT;
				USIDR = i2c_reg[i2c_current_Slave][i2c_offset++];
			}
			break;
		}
		post_ack = 0;
	}

	if (i2c_offset > (I2C_N_REG - 1))
		i2c_offset = 0;

	/* Set up SDA direction for next operation */
	if (sda_direction == I2C_SDA_DIR_OUT) {
		USI_DDR |= (1 << I2C_SDA);
	} else {
		USI_DDR &= ~(1 << I2C_SDA);
	}

	/* Clear flags and set counter */
	USISR = usisr_tmp;
}


/* Initialise the USI and I2C state machine */
void i2c_init( void )
{
	i2c_state = 0;
	USICR = (1 << USISIE) | (1 << USIOIE) | (3 << USIWM0) | (1 << USICS1);
	USI_DDR |= (1 << I2C_SCL);
	USI_DDR &= ~(1 << I2C_SDA);
	USI_PORT |= (1 << I2C_SDA) | (1 << I2C_SCL);
	USISR = 0xF0;
}


/*
 * Return non-zero if a transaction is ongoing
 * A transaction is considered ongoing if the slave address has
 * been matched, but a stop has not been received yet.
 */
uint8_t i2c_transaction_ongoing( void )
{
	if ((i2c_state != I2C_STATE_IDLE) &&
		(i2c_state != I2C_STATE_ADDR_MATCH)) {
		return 1;
	} else {
		return 0;
	}
}

/*
 * Check for and handle a stop condition.
 * Returns non-zero if any registers have been changed
 */
uint8_t i2c_check_stop(int8_t SlaveIndex)
{
	uint8_t ret = 0;

	if ((i2c_state == I2C_STATE_MASTER_WRITE) && i2c_update[SlaveIndex]) {
		cli();
		uint8_t tmp = USISR;
		if (tmp & (1 << USIPF)) {
			i2c_state = I2C_STATE_IDLE;
			ret = i2c_update[SlaveIndex];
			i2c_update[SlaveIndex] = 0;
		}
		sei();
	}
	return ret;
}


//calculates the PWM Value (0-255) from a PWM Channel of the PCA9685 emlulated slave
uint8_t getI2CPWMValue(uint8_t Slave, uint8_t PWMChannel)//from 0 to 15
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
