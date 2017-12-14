#!/usr/bin/python
import rospy
from rgb_led import RGB_LED
from rgb_led.srv import PlayLEDPattern


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

        self._rgb = RGB_LED()

    def _play_pattern_service_callback(self, req):
        rospy.loginfo("Playing LED Pattern %s for %f seconds", req.pattern_name, req.duration)
        self.play_pattern(req.pattern_name, req.duration)

        self._rgb.setRGB(2, [1, 0, 0])

        return []

    def play_pattern(self, pattern_name, duration):
        """
        Play the pattern with the given time for the given duration.
        """

        # TODO: Check that the given pattern actually exists.

        self._current_pattern = pattern_name
        self._current_duration = duration
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
            # TODO: Turn off all LEDs
            rospy.loginfo("Finished playing pattern on LEDs")
            return

        # TODO: Perform the actual LED update


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
