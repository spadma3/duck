

class Pattern():
    """
    An LED pattern.
    """
    def get_duration(self):
        """
        Get the duration of this pattern.
        :return:
        """
        return 0

    def get_identifier(self):
        """
        An identifier for this pattern.
        """
        return None

    def get_configuration(self, time):
        """
        Get the light configuration at that the given time.
        :param time: The time at which you want the configuration.
        :return: A dictionary containing the color for the LEDs that are on. All LEDs
        that have no entry in the returned dictionary are turned off.
        """
        return {}
