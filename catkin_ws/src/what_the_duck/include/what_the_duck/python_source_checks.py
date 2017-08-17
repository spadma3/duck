# -*- coding: utf-8 -*-
from what_the_duck.check import CheckFailed, Check
from duckietown_utils.locate_files_impl import locate_files
from duckietown_utils.exception_utils import raise_wrapped

class PythonPackageCheck(Check):
    ''' Checks that a package is well formed. '''
    def __init__(self, package_name, dirname):
        self.package_name = package_name
        self.dirname = dirname
        
    def check(self):
        # find a
        try:
            python_files = locate_files(self.dirname, '*.py')
            for filename in python_files:
                try:
                    check_no_tabs(filename)
                    check_no_half_merges(filename)
                except CheckFailed as e:
                    msg = 'Check failed for file %s:' % filename
                    raise_wrapped(CheckFailed, e, msg, compact=True)
        except CheckFailed as e:
            msg = 'Checks failed for package %s.' % self.package_name
            l = str(e)
            raise CheckFailed(msg, l)
            
def check_no_half_merges(filename):
    contents = open(filename).read()
    if ('<' * 4 in contents) or ('>'*4 in contents):
        msg = 'It loooks like this file has been half-merged.' 
        raise CheckFailed(msg)


def check_no_tabs(filename):
    # Things to check:
    
    # there is an "encoding" file line, and the encoding is utf-8
    
    contents = open(filename).read()
    if '\t' in contents:
        n = 0
        for c in contents:
            if c == '\t':
                n += 1
        msg = 'The file contains %d tab characters. The tab characters are evil!' % n
        raise CheckFailed(msg)
