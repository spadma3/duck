from sequence_pattern import SequencePattern
from rgb_led import LED, COLORS

class GoingToCustomer(SequencePattern):
    def get_identifier(self):
        return "fleet_planning/going_to_customer"

    def get_sequence(self):
        return [
            (0.3, {LED.FRONT_RIGHT: COLORS.YELLOW}),
            (0.3, {LED.TOP: COLORS.YELLOW}),
            (0.3, {LED.FRONT_LEFT: COLORS.YELLOW}),
            (0.3, {LED.BACK_LEFT: COLORS.YELLOW}),
            (0.3, {LED.BACK_RIGHT: COLORS.YELLOW}),
        ]

