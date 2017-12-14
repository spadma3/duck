
from pattern import Pattern

class SequencePattern(Pattern):
    """
    A pattern that is defined as a sequence of steady states.
    This class is meant to be subclasses.
    """

    def __init__(self):
        self._duration = None

    def get_sequence(self):
        """
        Returns the sequence of steady states as an array of 2-tuples containing the duration
        of that state and a dictionary with the colors for the LEDs.
        This method is meant to be overriden in subclasses.
        """
        return []

    def get_configuration(self, time):
        # Let's first ensure that time is within the duration of the sequence.
        relative_time = time % self._duration

        # Find the current state
        current_end_time = 0
        for duration, configuration in self.get_sequence():
            current_end_time += duration

            if relative_time < current_end_time:
                return configuration

    def get_duration(self):
        if self._duration is None:
            self._compute_duration()

        return self._duration

    def _compute_duration(self):
        """
        Computes the duration of this sequence.
        :return:
        """
        self._duration = sum(map(lambda s: s[0], self.get_sequence()))
