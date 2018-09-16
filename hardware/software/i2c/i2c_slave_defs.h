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

#ifndef __I2C_SLAVE_DEFS__
#define __I2C_SLAVE_DEFS__

/* Set these appropriately for your platform */
#define USI_PORT PORTA
#define USI_DDR DDRA
#define USI_PIN PINA
#define I2C_SCL 2
#define I2C_SDA 0
#define I2C_SLAVE_LED 0
#define I2C_SLAVE_MOTOR 1




#define N_LEDS 5 //maximal LED count, which are accessed by the addressable led bus
// old convention: [0,1,2,3,4]=[FL, BL, TOP, BR, FR]
// new convention: [0,1,2,3,4]=[FL, TOP, FR, BR, BL] (due to wiring of addr. LEDs)
// LED_REMAPS(old_convention)->new_convention
#define LED_REMAPS {0,4,1,3,2}
#define I2C_N_GLB_REG 4
#define I2C_N_REG 76 //PCA9685 has registers 0-69dez and 250-255dez. To Save memory, the registers 250-255 are mapped to 70-75! TODO

#define I2C_N_SLAVES 3 //maximal Count of i2c Slaves, which should be emulated, one is the motor, led and the ADC
extern volatile uint8_t I2C_SLAVE_ADDR[I2C_N_SLAVES];

/*
 * The library supports a write mask for each individual register (bits set are
 * writable) in the i2c_w_mask array. If you don't care about masks for each
 * individual register, you can define a global value to be used for all
 * registers here, saving flash and RAM
 */
#define I2C_GLOBAL_WRITE_MASK 0xFF
/*
#define REG_CTRL    i2c_reg[I2C_SLAVE_LED][0]
#define     CTRL_RST    (1 << 0)
#define     CTRL_GLB    (1 << 1)
#define REG_GLB_G   i2c_reg[I2C_SLAVE_LED][1]
#define REG_GLB_R   i2c_reg[I2C_SLAVE_LED][2]
#define REG_GLB_B   i2c_reg[I2C_SLAVE_LED][3]
*/
#endif /* __I2C_SLAVE_DEFS__ */
