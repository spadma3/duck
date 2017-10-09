# Using WS2812 leds for duckiebot

This is an attempt to use addressable leds with the duckiebot by means of ATTINY85 as an i2c slave, 
and 5 WS2812 addressable leds.

- ATTINY85 is connected to i2c.
- PB3 pin of the ATTINY85 is connected to leds.
- https://github.com/usedbytes/neopixel_i2c is used with minor modification.
