import os
import stat

import duckietown_utils as dtu
from what_the_duck.check import CheckFailed, CheckError, Check
from what_the_duck.resolution import Suggestion


__all__ = ['CheckPermissions']

class CheckPermissions(Check):
    def __init__(self, filename, expected):
        if not isinstance(expected, str) or len(expected) != 4:
            msg = 'Expected "expected" to be a 4-digit string octal ("0700")'
            raise ValueError(msg)
        
        self.filename = filename
        self.expected = expected
        
    def check(self):
        fn = dtu.expand_all(self.filename)
        
        if not os.path.exists(fn):
            msg = 'Cannot check permissions if file or dir does not exist.'
            raise CheckError(msg)
        
        fstats = os.stat(fn)
        filemode = oct(stat.S_IMODE(fstats.st_mode))
        if len(filemode) > 4:
            filemode = filemode[-4:]
        if filemode != self.expected:
            msg = ('Expected mode %r, obtained %r.' % 
                   (self.expected, filemode))
            raise CheckFailed(msg)
            
    def get_suggestion(self):
        msg = """
You can fix the permissions using:

    $ chmod %s %s
        """ % (self.expected, self.filename)
        return Suggestion(msg)
        