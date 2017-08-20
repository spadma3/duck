import os
from duckietown_utils.instantiate_utils import indent
from what_the_duck.check import Check, CheckFailed, CheckError
from duckietown_utils import expand_all

class FileContains(Check):
    
    """ Check that the file contains a certain string """
    
    def __init__(self, filename, string):
        self.filename = filename
        self.string = string
    
    def check(self):
        fn = expand_all(self.filename)
        
        if not os.path.exists(fn):
            msg = 'File does not exist: %s' % fn
            raise CheckError(msg)
        
        contents = open(fn).read()
                
        l = 'Entire contents of %s:\n' % fn + indent(contents, '  > ')
        
        if not self.string in contents:
            msg = 'String %r not contained in %s' % (self.string, fn)
            raise CheckFailed(msg, l)
        