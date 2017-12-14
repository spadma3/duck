#!/usr/bin/python
import rospy
from rgb_led import RGB_LED, LED, COLORS
from rgb_led.srv import PlayLEDPattern
from rgb_led.duckietown_lights import DuckietownLights


class LEDPatternNode:

    # Number of times the LEDs get updated per second
    LED_update_rate = 20

    def __init__(self):
        self.node = rospy.init_node('LEDPatternNode')

        # The name of the pattern that is currently played.
        self._current_pattern = None
        # How long the current pattern should be played.
        self._current_duration = 0.0
        # Time when the current pattern was started. This is used to find out when
        # to stop again.
        self._current_pattern_start_time = 0.0

        self._update_timer = None

        self._rgb_led = RGB_LED()

    def _play_pattern_service_callback(self, req):
        rospy.loginfo("Playing LED Pattern %s for %f seconds", req.pattern_name, req.duration)
        self.play_pattern(req.pattern_name, req.duration)
        return []

    def play_pattern(self, pattern_name, duration=None):
        """
        Play the pattern with the given time for the given duration.
        If duration is None, then the duration is extracted from the pattern.
        """

        # TODO: Check that the given pattern actually exists.
        if pattern_name not in DuckietownLights.patterns:
            rospy.logwarn("Tried to play pattern %s. But no pattern with that name was found", pattern_name)
            return

        self._current_pattern = DuckietownLights.patterns[pattern_name]
        if duration is not None:
            self._current_duration = duration
        else:
            self._current_duration = self._current_pattern.get_duration()
        self._current_pattern_start_time = rospy.get_rostime()

        # Start the timer that updates the LEDS.
        update_duration = 1 / float(LEDPatternNode.LED_update_rate)
        self._update_timer = rospy.Timer(rospy.Duration.from_sec(update_duration), self._update_leds)

    def _update_leds(self, event):
        rospy.logdebug("Updating LEDs")

        currentTime = rospy.get_rostime()
        elapsedTime = currentTime - self._current_pattern_start_time
        if elapsedTime.to_sec() > self._current_duration:
            # Stop the pattern and turn LEDs off
            self._current_pattern = None
            self._current_duration = 0.0
            self._current_pattern_start_time = 0.0
            self._update_timer.shutdown()

            # Turn off all LEDs
            for led in LED.DUCKIEBOT_LEDS:
                self._rgb_led.setRGB(led, COLORS.OFF)

            rospy.loginfo("Finished playing pattern on LEDs")
            return

        # Perform the actual LED update
        current_configuration = self._current_pattern.get_configuration(elapsedTime.to_sec())
        for led in LED.DUCKIEBOT_LEDS:
            if led in current_configuration:
                self._rgb_led.setRGB(led, COLORS.OFF)
            else:
                self._rgb_led.setRGB(led, current_configuration[led])


if __name__ == '__main__':
    rospy.init_node('LEDPatternNode')
    ledPatternNode = LEDPatternNode()

    # Setup the provided service
    play_pattern_service = rospy.Service(
        'LEDPatternNode/play_pattern',
        PlayLEDPattern,
        ledPatternNode._play_pattern_service_callback
    )

    rospy.loginfo("Finished startup of LEDPatternNode")
    rospy.spin()
