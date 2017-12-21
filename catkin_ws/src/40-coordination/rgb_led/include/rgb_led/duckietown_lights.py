import random
import time
from rgb_led import RGB_LED, LED, COLORS

# Import all the available patterns
from patterns.sequence_pattern import SequencePattern
from patterns.blinking import Blinking
from patterns.blinking1 import Blinking1
from patterns.blinking2 import Blinking2
from patterns.blinking3 import Blinking3
from patterns.fancy1 import Fancy1
from patterns.fancy2 import Fancy2
from patterns.fleet_planning.going_to_customer import GoingToCustomer
from patterns.fleet_planning.customer_dropoff import CustomerDropOff
from patterns.fleet_planning.customer_pickup import CustomerPickUp


class DuckietownLights():
    """
    Class that manages all available LED patterns.
    """
    # All the available patterns
    patterns = {}

    # All lights on a car
    car_all_lights = [
        LED.TOP,
        LED.BACK_LEFT,
        LED.BACK_RIGHT,
        LED.FRONT_LEFT,
        LED.FRONT_RIGHT
    ]

    @staticmethod
    def add_pattern(pattern):
        """
        Add a pattern to the global dictionary of available patterns.
        :param pattern: An object that is of type Pattern.
        """
        DuckietownLights.patterns[pattern.get_identifier()] = pattern

    @staticmethod
    def load_patterns():
        # Add all patterns we know
        DuckietownLights.add_pattern(Blinking())
        DuckietownLights.add_pattern(Blinking1())
        DuckietownLights.add_pattern(Blinking2())
        DuckietownLights.add_pattern(Blinking3())
        DuckietownLights.add_pattern(Fancy1())
        DuckietownLights.add_pattern(Fancy2())

        # Fleet planning related patterns
        DuckietownLights.add_pattern(GoingToCustomer())
        DuckietownLights.add_pattern(CustomerPickUp())
        DuckietownLights.add_pattern(CustomerDropOff())

# Load the patterns
DuckietownLights.load_patterns()




def create_color_patterns():
    """
    Method that creates patterns for different LEDs at different frequencies with different colors.
    """

    conf_all_off = {
        LED.TOP: COLORS.OFF,
        LED.BACK_LEFT: COLORS.OFF,
        LED.BACK_RIGHT: COLORS.OFF,
        LED.FRONT_LEFT: COLORS.OFF,
        LED.FRONT_RIGHT: COLORS.OFF,
    }

    conf_static_car = {
        LED.TOP: COLORS.OFF,
        LED.BACK_LEFT: COLORS.RED,
        LED.BACK_RIGHT: COLORS.RED,
        LED.FRONT_LEFT: COLORS.WHITE,
        LED.FRONT_RIGHT: COLORS.WHITE,
    }

    def conf_all_on(color):
        x = dict(**conf_all_off)
        for k in x:
            x[k] = color
        return x

    def blink_one(which, color, period, others=conf_all_off):
        r = dict(**others)
        r[which] = color
        return [
            (period / 2, others),
            (period / 2, r),
        ]

    def blink_all(color, period):
        return [
            (period / 2, conf_all_off),
            (period / 2, conf_all_on(color)),
        ]

    colors = {
        'white': COLORS.WHITE,
        'red': COLORS.RED,
        'green': COLORS.GREEN,
        'blue': COLORS.BLUE,
        'yellow': COLORS.YELLOW,
        'orange': COLORS.ORANGE,
    }

    frequencies = [1, 1.1, 1.2, 1.3, 1.4, 1.5,
                   1.6, 1.7, 1.8, 1.9, 2, 2.1, 2.2, 2.3, 2.4,
                   2.5, 3, 3.5, 4.5, 5, 6, 7, 8, 9,
                   10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]

    for name in DuckietownLights.car_all_lights:
        for color, rgb in colors.items():
            for freq in frequencies:
                comb = '%s-%s-%1.1f' % (name, color, freq)
                pattern = SequencePattern(blink_one(name, rgb, 1.0 / freq), comb)
                DuckietownLights.add_pattern(pattern)

    for color, rgb in colors.items():
        for freq in frequencies:
            comb = 'wr-%s-%1.1f' % (color, freq)
            pattern = SequencePattern(blink_one(LED.TOP, rgb, 1.0 / freq, others=conf_static_car), comb)
            DuckietownLights.add_pattern(pattern)

    for color, rgb in colors.items():
        for freq in frequencies:
            comb = 'all-%s-%1.1f' % (color, freq)
            pattern = SequencePattern(blink_all(rgb, 1.0 / freq), comb)
            DuckietownLights.add_pattern(pattern)

# Enable this to create all kinds of single color blinking patterns.
#create_color_patterns()
