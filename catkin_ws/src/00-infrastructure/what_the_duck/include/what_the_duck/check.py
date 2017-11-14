from abc import abstractmethod, ABCMeta

class CheckException(Exception):
    """ The check failed """
    def __init__(self, compact, long_explanation=None):
        if long_explanation is None:
            long_explanation = ''

        self.compact = compact.strip()
        self.long_explanation = long_explanation.strip()
        both = compact + '\n\n' + long_explanation
        Exception.__init__(self, both)


class CheckFailed(CheckException):
    pass

class CheckError(CheckException):
    """ An error while checking; the test itself failed """

class Check(object):
    __metaclass__ = ABCMeta
    
    @abstractmethod
    def check(self):
        """ 
            Returns None on success.
            
            Raises CheckFailed if the check failed.
            
            Raises CheckError if a precondition failed.
        """
        pass
    
    def get_suggestion(self):
        """ If the check can generate a resolution, return
            a Suggestion() object. """
        return None
    
        
        