#!/usr/bin/python
from Adafruit_PWM_Servo_Driver import PWM


class LED():
    """
    Handy names for the available LEDs
    """
    TOP = 'top'
    BACK_LEFT = 'bl'
    BACK_RIGHT = 'br'
    FRONT_LEFT = 'fl'
    FRONT_RIGHT = 'fr'

    DUCKIEBOT_LEDS = [
        TOP,
        BACK_LEFT,
        BACK_RIGHT,
        FRONT_LEFT,
        FRONT_RIGHT
    ]


class COLORS():
    """
    Handy names for colors.
    """
    ORANGE =[1, 0.4, 0] 
    RED = [1, 0, 0]
    GREEN = [0, 1, 0]
    BLUE = [0, 0, 1]
    YELLOW = [1, 1, 0]
    WHITE = [1, 1, 1]
    OFF = [0, 0, 0]
    BLACK = OFF

class RGB_LED():

    # Configuration (calibration) - Optimally in a YAML file.
    PORT_MAPPING = {
        LED.TOP: 2,
        LED.BACK_LEFT: 1,
        LED.BACK_RIGHT: 3,
        LED.FRONT_LEFT: 0,
        LED.FRONT_RIGHT: 4,
    }

    OFFSET_RED = 0
    OFFSET_GREEN = 1
    OFFSET_BLUE = 2

    def __init__(self, debug=False):
        self.pwm = PWM(address=0x40, debug=debug)
        for i in range(15):
            self.pwm.setPWM(i, 0, 4095)

    def setLEDBrightness(self, led, offset, brightness):
        """
        Set the brightness of an LED.
        :param led: The LED of which you want to adjust the brightness.
        :param offset:
        :param brightness: The brightness. An 8-bit integer (i.e. in the range [0,255]
        """
        self.pwm.setPWM(3 * led + offset, brightness << 4, 4095)

    def setRGBint24(self, led, color):
        """
        Set the color of an LED from a 24 bit int where the first 8 bits represent R, the second
        8 represent G and the last 8 represent B.
        :param led: The port number of the led.
        :param color: The integer containing the color information as described above.
        """
        r = color >> 16 & 0xFF
        g = color >> 8 & 0xFF
        b = color >> 0 & 0xFF
        self.setRGBvint8(led, [r, g, b])
        
    def setRGBvint8(self, led, color):
        """
        Set the RGB color of an LED.
        :param led: The port number of an LED:
        :param color: An array with 3 elements each containig an int in the range [0, 255]
        """
        self.setLEDBrightness(led, self.OFFSET_RED, color[0])
        self.setLEDBrightness(led, self.OFFSET_GREEN, color[1])
        self.setLEDBrightness(led, self.OFFSET_BLUE, color[2])

    def setRGB(self, led, color):
        """
        Set the color of a given LED.
        :param led: The port number of the LED. (I.e. use PORT_MAPPING to find it)
        :param color: An array with RGB components in the range [0, 1].
        """
        self.setRGBvint8(led, map(lambda f: int(f * 255), color))

    def __del__(self):
        for i in range(15):
            self.pwm.setPWM(i, 0, 4095)
        del self.pwm
