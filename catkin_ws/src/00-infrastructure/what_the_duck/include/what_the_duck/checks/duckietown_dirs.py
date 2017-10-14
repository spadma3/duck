from duckietown_utils.constants import get_duckiefleet_root
from duckietown_utils.exception_utils import raise_wrapped
from duckietown_utils.exceptions import DTConfigException
from what_the_duck.check import Check, CheckFailed


class FindingDuckiefleet(Check):
    
    def __init__(self):
        pass
        
    def check(self):
        try:
            d = get_duckiefleet_root()
            return "Duckiefleet root is detected as %r"  % d
        except DTConfigException as e:
            msg = 'Could not get the duckiefleet root in any way.'
            raise_wrapped(CheckFailed, e, msg)

class UptodateDuckiefleet(Check):
    
    def __init__(self):
        pass
        
    def check(self):
        pass
