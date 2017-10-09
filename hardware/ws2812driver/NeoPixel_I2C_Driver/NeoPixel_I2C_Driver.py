#!/usr/bin/python

import time
import math
from Adafruit_I2C import Adafruit_I2C

# ============================================================================
# NeoPixel_I2C_Driver
# ============================================================================

class PWM :
  # Registers/etc.
  __OFFSET             = 0x04
  __RESET	       = 0x01
  __GLOBAL	       = 0x02

  general_call_i2c = Adafruit_I2C(0x00)

  @classmethod
  def softwareReset(cls):
    "Resets the LEDS by writing 0x01 on the control register"
    cls.general_call_i2c.writeRaw8(self.__RESET)

  def __init__(self, address=0x40, debug=False):
    self.i2c = Adafruit_I2C(address)
    self.i2c.debug = debug
    self.address = address
    self.debug = debug
    self.setAllPWM(0, 0)
    time.sleep(0.01)

  def setPWM(self, channel, on, off):
    "Sets a single PWM channel"
    pwmval = int((on / float(off)) * 255)
    self.i2c.write8(self.__OFFSET + channel, pwmval)
    time.sleep(0.001)

  def setAllPWM(self, on, off):
    "Sets all PWM channels"
    if off==0 and on==0 :
       pwmval = 0
    elif off==0 :
       pwmval = 255
    else :
    	pwmval = int((on / float(off)) * 255)
    self.i2c.write8(1, pwmval)
    self.i2c.write8(2, pwmval)
    self.i2c.write8(3, pwmval)
    self.i2c.write8(0, self.__GLOBAL)
    self.i2c.write8(0, 0x00)
