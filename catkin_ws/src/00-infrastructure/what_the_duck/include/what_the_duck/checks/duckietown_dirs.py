import duckietown_utils as dtu

from what_the_duck.check import Check, CheckFailed

__all__ = [
    'FindingDuckiefleet',
    'UptodateDuckiefleet',
]

class FindingDuckiefleet(Check):
    
    def __init__(self):
        pass
        
    def check(self):
        try:
            d = dtu.get_duckiefleet_root()
            return "Duckiefleet root is detected as %r"  % d
        except dtu.DTConfigException as e:
            msg = 'Could not get the duckiefleet root in any way.'
            dtu.raise_wrapped(CheckFailed, e, msg)

class UptodateDuckiefleet(Check):
    
    def __init__(self):
        pass
        
    def check(self):
        pass
