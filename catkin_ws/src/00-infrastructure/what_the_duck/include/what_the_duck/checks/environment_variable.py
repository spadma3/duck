import os
from what_the_duck.check import Check, CheckFailed, CheckError

class EnvironmentVariableExists(Check):
    """ Checks that the environment variable is set. """
    def __init__(self, name):
        self.name = name
        
    def check(self):
        if self.name not in os.environ:
            msg = 'Could not find environment variable %r.' % self.name
            raise CheckFailed(msg)
        return 'The value is %r' % os.environ[self.name]
        
        

class EnvironmentVariableIsEqualTo(Check):
    """ Checks that the environment variable is set to a specific value. """
    def __init__(self, name, expected):
        self.name = name
        self.expected = expected
        
    def check(self):
        if self.name not in os.environ:
            msg = 'Could not find environment variable %r.' % self.name
            raise CheckError(msg)
        
        value = os.environ[self.name]
        
        if value != self.expected:
            msg = 'Value of $%s is %r instead of %r.' % (self.name, value, self.expected)
            raise CheckFailed(msg)
        
        return 'The value is %r' % os.environ[self.name]