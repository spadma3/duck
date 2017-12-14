import random
import time
from rgb_led import RGB_LED, LED, COLORS

from patterns.sequence_pattern import SequencePattern
from patterns.blinking import Blinking
from patterns.blinking1 import Blinking1
from patterns.blinking2 import Blinking2
from patterns.blinking3 import Blinking3


class DuckietownLights():
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


# Add all patterns we know
DuckietownLights.add_pattern(Blinking())
DuckietownLights.add_pattern(Blinking1())
DuckietownLights.add_pattern(Blinking2())
DuckietownLights.add_pattern(Blinking3())
create_color_patterns()


def cycle_LEDs_named(sequence_name):
    if sequence_name not in DuckietownLights.patterns:
        msg = 'Could not find the sequence %r.' % sequence_name
        msg += '\n\nThese are some I know:'
        avail = list(DuckietownLights.patterns)
        random.shuffle(avail)
        N = 20
        if len(avail) > N:
            avail = avail[:N]

        msg += ' ' + ", ".join(avail) + '.'
        raise ValueError(msg)
    sequence = DuckietownLights.patterns[sequence_name]
    cycle_LEDs(sequence)


def get_current_step(t, t0, sequence):
    """ returns i, (period, color) """
    period = sum(s[0] for s in sequence)

    tau = t - t0
    while tau - period > 0:
        tau -= period
    i = 0
    while True:
        current = sequence[i][0]
        if tau < current:
            break
        else:
            i += 1
            tau -= current

    return i, sequence[i]


def cycle_LEDs(sequence):
    led = RGB_LED()

    t0 = time.time()
    # last = all_off
    last = {
        LED.TOP: None,
        LED.BACK_LEFT: None,
        LED.BACK_RIGHT: None,
        LED.FRONT_LEFT: None,
        LED.FRONT_RIGHT: None,
    }
    while True:
        t = time.time()
        _, (_, config) = get_current_step(t, t0, sequence)

        for name, col in config.items():
            k = DuckietownLights.name2port[name]

            if last[name] != col:
                led.setRGB(k, col)
                print(name, k, col)
                last[name] = col

        time.sleep(0.01)
