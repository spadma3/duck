import os

from duckietown_utils import expand_all
from what_the_duck.check import Check, CheckFailed


class FileExists(Check):
    
    def __init__(self, filename):
        self.filename = filename
        
    def check(self):
        fn = expand_all(self.filename)
        
        short = os.path.basename(self.filename)
        if not os.path.exists(fn):
            msg = 'Path does not exist: %s' %  short
            l = 'Complete path:\n  %s' % fn
            raise CheckFailed(msg, l)

        if os.path.isdir(fn):
            msg = 'Expect this to be a file, not a directory: %s' % short
            l = 'Complete path:\n  %s' % fn
            raise CheckFailed(msg)
        
        data = open(fn).read()
        lines = data.split('\n')
        maxlen = max(map(len, lines))
        if len(lines) < 100 and maxlen < 80:
            return data
    

class DeviceExists(Check):
    
    def __init__(self, filename):
        self.filename = filename
        
    def check(self):
        fn = expand_all(self.filename)
        
        if not os.path.exists(fn):
            msg = 'Path does not exist: %s' % fn
            raise CheckFailed(msg)

        if os.path.isdir(fn):
            msg = 'Expect this to be a device, not a directory: %s' % fn
            raise CheckFailed(msg)
        
        # XXX: how to check for device?
        
class DirExists(Check):
    
    def __init__(self, filename):
        self.filename = filename
        
    def check(self):
        fn = expand_all(self.filename)
        
        if not os.path.exists(fn):
            msg = 'Dir does not exist: %s' % fn
            raise CheckFailed(msg)
        
        if not os.path.isdir(fn):
            msg = 'Expect this to be a directory: %s' % fn
            raise CheckFailed(msg)
        