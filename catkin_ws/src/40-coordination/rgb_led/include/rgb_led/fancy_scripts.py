def turn_off_LEDs(speed=5):
	from .duckietown_lights import DuckietownLights, TOP, BACK_LEFT, BACK_RIGHT, FRONT_LEFT, FRONT_RIGHT
	import time
	from time import sleep
	from .rgb_led import RGB_LED
	led = RGB_LED()

	from math import sin, cos

	t0 = time.time()
	def get_config(t):
		t = t * speed
		a=0
		b=0
		c=0
		return {
			TOP: [a,b,c],
			BACK_LEFT: [b,c,a],
			BACK_RIGHT: [a, c,b],
			FRONT_LEFT: [b,a,c],
			FRONT_RIGHT: [c,a,b],
		}

	for i in range(1):
		t = time.time()
		config = get_config(t)
		for name, col in config.items():
			k = DuckietownLights.name2port[name]
			led.setRGB(k, col)		
		sleep(0.01)
	del led





