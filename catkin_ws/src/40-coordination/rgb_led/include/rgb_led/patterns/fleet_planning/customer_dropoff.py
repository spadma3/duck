from rgb_led.patterns.sequence_pattern import SequencePattern
from rgb_led import LED, COLORS

class CustomerDropOff(SequencePattern):
    def get_identifier(self):
        return "fleet_planning/customer_dropoff"

    def get_sequence(self):
        color = COLORS.GREEN
        return [
            (0.1, {LED.FRONT_RIGHT: color}),
            (0.1, {LED.TOP: color}),
            (0.1, {LED.FRONT_LEFT: color}),
            (0.1, {LED.BACK_LEFT: color}),
            (0.1, {LED.BACK_RIGHT: color}),
            (0.1, {LED.FRONT_RIGHT: color}),
            (0.1, {LED.TOP: color}),
            (0.1, {LED.FRONT_LEFT: color}),
            (0.1, {LED.BACK_LEFT: color}),
            (0.1, {LED.BACK_RIGHT: color}),
        ]

