# -*- coding: utf-8 -*-
from what_the_duck.check import CheckFailed, Check
from duckietown_utils.locate_files_impl import locate_files
from duckietown_utils.exception_utils import raise_wrapped
import os
from duckietown_utils.constants import DuckietownConstants
from duckietown_utils.instantiate_utils import indent

class PythonPackageCheck(Check):
    ''' Checks that a package is well formed. '''
    def __init__(self, package_name, dirname):
        self.package_name = package_name
        self.dirname = dirname
        
    def check(self):
        
        if self.package_name in DuckietownConstants.good_readme_exceptions:
            pass # 
        else:
            check_good_readme(self.dirname, self.package_name)
        
        python_files = locate_files(self.dirname, '*.py')
        
        try:
            for filename in python_files:
                try:
                    check_no_half_merges(filename)
                    if DuckietownConstants.enforce_no_tabs:
                        check_no_tabs(filename)
                    if DuckietownConstants.enforce_naming_conventions:
                        check_good_name(filename)
                    
                except CheckFailed as e:
                    l = 'Check failed for file:\n    %s' % filename + '\n' + str(e.long_explanation)
                    raise CheckFailed(e.compact, l)
                    raise
#                     fn = os.path.relpath(filename, self.dirname)
#                     msg = 'Check failed for file %s:' % fn
#                     raise_wrapped(CheckFailed, e, msg, compact=True)

        except CheckFailed as e:
            raise
#             msg = 'Checks failed for package %s.' % self.package_name
#             l = str(e)
#             raise CheckFailed(msg, l)

def looks_camel_case(x):
    """ checks if there is lower UPPER sequence """
    for i in range(len(x)-1):
        a = x[i]
        b = x[i+1]
        if a.isalpha() and b.isalpha():
            if a.islower() and b.isupper():
                return True
    return False
            
def check_good_name(filename):
    bn = os.path.basename(filename)
    
    if looks_camel_case(bn):
        msg = 'Invalid filename %r. Python files should not use CamelCase; we use underscored_file_names.' %bn
        raise CheckFailed(msg)
    
    
def check_no_half_merges(filename):
    contents = open(filename).read()
    if ('<' * 4 in contents) or ('>'*4 in contents):
        msg = 'It loooks like the file %r has been half-merged.' % os.path.basename(filename) 
        raise CheckFailed(msg)

def check_good_readme(dirname, package_name):
    fn = os.path.join(dirname, 'README.md')
    if not os.path.exists(fn):
        msg = 'The README file for %r does not exist.' % (package_name)
        l = 'File does not exist: %s' % fn
        raise CheckFailed(msg, l)
    
    
    # Check that the first line is a H1 in the format 
    #     # blah blah   {#package_name}
    contents = open(fn).read()
    lines0 = contents.split('\n')
    # remove all empty lines
    lines = [_ for _ in lines0 if _.strip()]
    first = lines[0]
    try:
        if not first.startswith('# '):
            msg = 'I expect the first line of the README to be a valid topic declaration.'
            raise ValueError(msg)
        frag = '{#%s' % package_name
        if not frag in first:
            msg = "Could not find %r in first line." % frag
            raise ValueError(msg)
    except ValueError as e:
        msg = 'README problem: ' + str(e)
        l = 'First 5 lines are:\n' + indent("\n".join(lines0[:5]), '> ')
        raise CheckFailed(msg, l)

def check_no_tabs(filename):
    # Things to check:
    
    # there is an "encoding" file line, and the encoding is utf-8
    
    contents = open(filename).read()
    if '\t' in contents:
        n = 0
        for c in contents:
            if c == '\t':
                n += 1
                
        short = os.path.basename(filename)
        msg = 'The file %r contains %d tab characters.' % (short, n)
        l = 'The tab characters are evil in Python code.'
        l += '\nPlease be *very* careful in changing them.'
        l += '\nDo *not* use a tool to do it (e.g. "Convert tabs to spaces"); it will get it wrong!' 
        raise CheckFailed(msg, l)
