from sequence_pattern import SequencePattern
from rgb_led import LED, COLORS


class Blinking2(SequencePattern):
    def get_identifier(self):
        return "blinking2"

    def get_sequence(self):
        WHITE2 = [0.8, 0.8, 0.8]
        GREEN2 = [0, 0.3, 0]
        return [
            (0.25,
             {
                 LED.TOP: GREEN2,
                 LED.BACK_LEFT: COLORS.RED,
                 LED.BACK_RIGHT: COLORS.RED,
                 LED.FRONT_LEFT: WHITE2,
                 LED.FRONT_RIGHT: WHITE2
             }),
            (0.25,
             {
                 LED.TOP: COLORS.OFF,
                 LED.BACK_LEFT: COLORS.OFF,
                 LED.BACK_RIGHT: COLORS.OFF,
                 LED.FRONT_LEFT: WHITE2,
                 LED.FRONT_RIGHT: WHITE2
             }),
        ]
