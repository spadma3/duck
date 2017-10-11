# Using WS2812 leds for duckiebot

This is an attempt to use addressable leds with the duckiebot by means of ATTINY85 as an i2c slave, 
and 5 WS2812 addressable leds. 

The original duckiebot uses a PWM hat and a custom made PCB that connects to the leds that are on the bumpers. Since each LED requires 4 cables, this creates a cable clutter, and difficulties in assembly.

- ATTINY85 is connected to I2C on Raspberry PI and powered with 3V3 from Raspberry PI.
- PB3 pin of the ATTINY85 is connected to led strips data line.
- https://github.com/usedbytes/neopixel_i2c library is used with minor modification.
- This setup costs much less.

# Required Circuit

![alt text](https://raw.githubusercontent.com/altineller/Software/master/hardware/ws2812driver/ws2812-attiny85-i2c-driver.png "Required Circuit")

# Programming The ATTINY85

A cheap (5$) USBASP programmer is required to program the ATTINY85. The original code @ https://github.com/usedbytes/neopixel_i2c must be modified.

- Just uncomment the `swirly();` at line 124 of main.c 
- Set the `#define N_LEDS 5` in neopixel_i2c/i2c/i2c_slave_defs.h file.

After installing the avr-gcc tools, just type `make` and it will compile and upload the program. Also if it is the first time programming the ATTINY85 chip, you have to type `make fuses` to configure the fuses. The USBASP programmer has a jumper for 'SLOW_SCK'. Depending on your programmer, if the make command fails at first you might need to short this jumper on the programmer.

# Testing the hardware

- ssh into duckiebot and `sudo i2cdetect -t 1` you should see the device at 0x40. (Make sure that the original PWM hat is off)
- `sudo i2cset -y 1 0x40 0x04 0x80` should light the first led in GREEN.
- `sudo i2cset -y 1 0x40 0x07 0x80` should light the second led in GREEN.
- `sudo i2cset -y 1 0x40 0x00 0x01` all leds should be off. (RESET)
- 0x04,0x05,0x06 are the GRB of the first led and so on.
- 0x00 is a control register. Writing 0x01 to it causes reset, writing 0x02 will cause the global GRB values at 0x01,0x02,0x03 be applied to all the leds.

# Getting it to work duckiebot

The neopixel_i2c library can be modified to emulate PCA9685 chip so it could work transparently, but the PCA9685 uses 4 bytes per channel and this would be a waste of resources both on the ATTINY85 and the Raspberry PI's I2C bus.

It is also possible to modify the duckiebot code to use the led driver without further modification on the neopixel_i2c library. 

Below are steps required for such modification.

- Copy the `NeoPixel_I2C_Driver` directory into `duckietown/catkin_ws/src/05-teleop/adafruit_drivers/`
- Modify the `duckietown/catkin_ws/src/05-teleop/adafruit_drivers/setup.py` file and add `'NeoPixel_I2C_Driver'` to packages array to make it look like:
`packages=['Adafruit_ADS1x15', 'Adafruit_GPIO','Adafruit_I2C','Adafruit_LSM303','Adafruit_MotorHAT','Adafruit_PWM_Servo_Driver','Gyro_L3GD20', 'NeoPixel_I2C_Driver'],`
- Modify the `duckietown/catkin_ws/src/40-coordination/rgb_led/include/rgb_led/rgb_led.py` and add
`OFFSET_RED   = 1
 OFFSET_GREEN = 0
 OFFSET_BLUE  = 2`
 This is because the WS2812 expects GRB instead of RGB order.
 - Change `from Adafruit_PWM_Servo_Driver import PWM  # @UnresolvedImport` to `from NeoPixel_I2C_Driver import PWM # @UnresolvedImport`

After this launch `'roslaunch led_joy_mapper led_joy_with_led_emitter_test.launch veh:=robotname'` and it should respond to joystick buttons as described in documentation.

The WS2812 leds are in the same order. FL, BL, TOP, BR, FR are LED 1 to 5 respectively.
