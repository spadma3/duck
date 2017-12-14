from pattern import Pattern
from rgb_led import LED, COLORS


class Blinking(Pattern):

    def __init__(self, color=COLORS.WHITE, duration=2):
        self._color = color
        self._duration = float(duration)

    def get_duration(self):
        return self._duration

    def get_identifier(self):
        return "blinking"

    def get_configuration(self, time):
        relative_time = time % self.get_duration()
        if relative_time < self._duration / 2:
            # All on
            return {
                LED.TOP: self._color,
                LED.FRONT_RIGHT: self._color,
                LED.FRONT_LEFT: self._color,
                LED.BACK_RIGHT: self._color,
                LED.BACK_LEFT: self._color,
            }
        else:
            # All off
            return {}
