import urllib2
from what_the_duck.check import Check, CheckFailed

class InternetConnected(Check):
    """ Check that we are connected to the internet """
    
    def __init__(self, url='https://www.google.com'):
        """ Use an https server so we know that we are not fooled by
            over-reaching academic network admins """
        self.url = url
        
    def check(self):
        try:
            urllib2.urlopen(self.url, timeout=3)
#             print "Connected"
        except IOError as e:
            msg = 'Cannot connect to %s' % self.url
            l = str(e)
            raise CheckFailed(msg, l)
            
