#!/usr/bin/python
import rospy
from rgb_led import RGB_LED, LED, COLORS
from rgb_led.srv import PlayLEDPattern, ListLEDPatterns
from rgb_led.duckietown_lights import DuckietownLights


class LEDPatternNode:

    # Number of times the LEDs get updated per second
    LED_update_rate = 40

    def __init__(self):
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
        """
        Method that is called when a call to the service "play_pattern" is made
        """
        self.play_pattern(req.pattern_name, req.duration)
        return []

    def _list_pattern_service_callback(self, req):
        """
        Method that is called when a call to the service "list_patterns" is made
        """
        pattern_names = DuckietownLights.patterns.keys()
        rospy.loginfo("Returning all available patterns: %s", pattern_names)
        return {'patterns': pattern_names}

    def play_pattern(self, pattern_name, duration=-1):
        """
        Play the pattern with the given time for the given duration.
        If duration is negative, then the duration is extracted from the pattern.
        """

        # Check that the given pattern actually exists.
        if pattern_name not in DuckietownLights.patterns:
            rospy.logwarn("Tried to play pattern %s. But no pattern with that name was found", pattern_name)
            return

        self._current_pattern = DuckietownLights.patterns[pattern_name]
        if duration > 0:
            self._current_duration = duration
        else:
            self._current_duration = self._current_pattern.get_duration()
        self._current_pattern_start_time = rospy.get_rostime()

        # Start the timer that updates the LEDS.
        if self._update_timer is not None:
            self._update_timer.shutdown()
            self._update_timer = None
        update_duration = 1 / float(LEDPatternNode.LED_update_rate)
        self._update_timer = rospy.Timer(rospy.Duration.from_sec(update_duration), self._update_leds)

        rospy.loginfo("Playing LED Pattern %s for %f seconds", pattern_name, self._current_duration)

    def _update_leds(self, event):
        """
        Method that is called periodiclly by a timer if the LEDs need to be updated.
        """
        rospy.logdebug("Updating LEDs")

        current_time = rospy.get_rostime()
        elapsed_time = current_time - self._current_pattern_start_time
        if elapsed_time.to_sec() > self._current_duration:
            # Stop the pattern and turn LEDs off
            self._current_pattern = None
            self._current_duration = 0.0
            self._current_pattern_start_time = 0.0
            self._update_timer.shutdown()
            self._update_timer = None

            # Turn off all LEDs
            for led in LED.DUCKIEBOT_LEDS:
                led_port = RGB_LED.PORT_MAPPING[led]
                self._rgb_led.setRGB(led_port, COLORS.OFF)

            rospy.loginfo("Finished playing pattern on LEDs")
            return

        # Perform the actual LED update
        current_configuration = self._current_pattern.get_configuration(elapsed_time.to_sec())
        for led in LED.DUCKIEBOT_LEDS:
            led_port = RGB_LED.PORT_MAPPING[led]
            if led not in current_configuration:
                self._rgb_led.setRGB(led_port, COLORS.OFF)
            else:
                self._rgb_led.setRGB(led_port, current_configuration[led])

    def on_shutdown(self):
        """
        Method that is run by ROS when this node is shutdown.
        """
        # Ensure all timers are invalidated.
        if self._update_timer is not None:
            self._update_timer.shutdown()
            self._update_timer = None


if __name__ == '__main__':
    rospy.init_node('LEDPatternNode')
    ledPatternNode = LEDPatternNode()
    rospy.on_shutdown(ledPatternNode.on_shutdown)

    # Setup the provided service
    play_pattern_service = rospy.Service(
        'LEDPatternNode/play_pattern',
        PlayLEDPattern,
        ledPatternNode._play_pattern_service_callback
    )
    list_pattern_service = rospy.Service(
        'LEDPatternNode/list_patterns',
        ListLEDPatterns,
        ledPatternNode._list_pattern_service_callback
    )

    # If a pattern was provided via parameters, then just play it
    pattern = rospy.get_param('led_pattern', None)
    if pattern is not None:
        ledPatternNode.play_pattern(pattern, 10000)

    rospy.loginfo("Finished startup of LEDPatternNode")
    rospy.spin()
