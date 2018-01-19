from pattern import Pattern
from math import cos, sin
from rgb_led import LED


class Fancy1(Pattern):
    """
    A fancy blinking patter that uses all LEDs
    """

    def __init__(self, speed=5):
        self._speed = speed

    def get_identifier(self):
        return "fancy1"

    def get_configuration(self, time):
        t = time * self._speed
        a = 0.5 + 0.5 * cos(t)
        b = 0.5 + 0.5 * sin(t)
        c = 0.5 + 0.5 * cos(2 * t)

        return {
            LED.TOP: [a, b, c],
            LED.BACK_LEFT: [b, c, a],
            LED.BACK_RIGHT: [a, c, b],
            LED.FRONT_LEFT: [b, a, c],
            LED.FRONT_RIGHT: [c, a, b],
        }
