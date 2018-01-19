from sequence_pattern import SequencePattern
from rgb_led import LED, COLORS


class Blinking3(SequencePattern):
    def get_identifier(self):
        return "blinking3"

    def get_sequence(self):
        GREEN2 = [0, 0.3, 0]
        return [
            (0.25,
             {
                 LED.TOP: COLORS.GREEN,
                 LED.BACK_LEFT: GREEN2,
                 LED.BACK_RIGHT: COLORS.YELLOW,
                 LED.FRONT_LEFT: COLORS.YELLOW,
                 LED.FRONT_RIGHT: COLORS.WHITE
             }),
            (0.25,
             {
                 LED.TOP: COLORS.RED,
                 LED.BACK_LEFT: GREEN2,
                 LED.BACK_RIGHT: COLORS.RED,
                 LED.FRONT_LEFT: COLORS.RED,
                 LED.FRONT_RIGHT: COLORS.RED
             }),
        ]
