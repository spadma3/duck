from duckietown_utils.cli import D8App
from abc import abstractmethod


class RunLineDetectionTests(D8App): 
    """ Runs the line detection tests programmatically. """

    def define_options(self, params):
        params.add_flag('cache', help="Use cache") 
        params.add_flag('cloud', help="Use cloud cache")
        params.accept_extra()
        
        
    @abstractmethod
    def show_info(self, logs):
        pass
    
    
    def go(self):
        pass