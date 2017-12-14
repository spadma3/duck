from pattern import Pattern
from math import cos, sin
from rgb_led import LED


class Fancy2(Pattern):
    """
    A fancy blinking patter that uses all LEDs
    """

    def __init__(self, speed=5):
        self._speed = speed

    def get_identifier(self):
        return "fancy1"

    def get_configuration(self, time):
        slow = 0.00005 * time
        a = []
        for i in range(15):
            period = self._speed * (sin(i) + 0.00001 * cos(slow + i))
            x = 0.5 + 0.5 * cos(time * period)
            a.append(x)

        # nonlinear
        f = lambda x: x * x * x * 0.9 + 0
        s = sum([f(x) for x in a])
        a = [f(x) / s for x in a]
        return {
            LED.TOP: [a[0], a[1], a[2]],
            LED.BACK_LEFT: [a[3], a[4], a[5]],
            LED.BACK_RIGHT: [a[6], a[7], a[8]],
            LED.FRONT_LEFT: [a[9], a[10], a[11]],
            LED.FRONT_RIGHT: [a[12], a[13], a[14]],
        }
