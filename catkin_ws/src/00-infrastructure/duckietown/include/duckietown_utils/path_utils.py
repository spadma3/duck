import os

def expand_all(filename):
    fn = filename
    fn = os.path.expanduser(fn)
    fn = os.path.expandvars(fn)
    return fn