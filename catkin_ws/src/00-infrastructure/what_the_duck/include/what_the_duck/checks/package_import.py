from what_the_duck.check import Check, CheckFailed


class CanImportPackages(Check):
    
    def __init__(self, packages):
        self.packages = packages
        
    def check(self):
        for package in self.packages:
            try:
                __import__(package, fromlist=['dummy'])
            except ImportError as e:
                msg = 'Cannot import package %r: %s\n' % (package, e)
                raise CheckFailed(msg)
