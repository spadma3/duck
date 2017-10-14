from what_the_duck.check import Check, CheckFailed


class CanImportPackages(Check):
    
    def __init__(self, packages):
        self.packages = packages
        
    def check(self):
        out = ""
        for package in self.packages:
            
            try:
                p = __import__(package, fromlist=['dummy'])
                
                out += '%s: %s\n' % (package, package_info(p))
                
            except ImportError as e:
                msg = 'Cannot import package %r: %s\n' % (package, e)
                raise CheckFailed(msg)

        return out
    
def package_info(p):
    if hasattr(p, '__version__'):
        v = str(getattr(p, '__version__'))
    else:
        v = '?'
    d = v 
    d += '  ' + p.__file__
    return d