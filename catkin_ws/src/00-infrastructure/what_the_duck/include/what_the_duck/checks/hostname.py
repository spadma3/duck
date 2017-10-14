from what_the_duck.check import Check, CheckFailed
import socket
from duckietown_utils.instantiate_utils import indent


class CheckHostnameConfigured(Check):
    
    def check(self):
        hostname = socket.gethostname()
        
        initial = 'duckiebot-not-configured'
        if hostname == initial:
            msg = 'The hostname is not configured; it is still %r.' % initial
            raise CheckFailed(msg)
        
        fn = '/etc/hostname'
        contents = open(fn).read().strip()
        if contents != hostname:
            msg = 'The hostname is %r but it does not match %s.' % (hostname, fn)
            l = 'Entire contents of %s:\n' % fn + indent(contents, '  > ')
            raise CheckFailed(msg, l)
        
        return contents
         
class CheckGoodHostsFile(Check):
    
    def check(self):
        # read hosts file
        fn = '/etc/hosts'
        contents = open(fn).read()
        
        l = 'Entire contents of %s:\n' % fn + indent(contents, '  > ')
        
        if '10.' in contents or '192.' in contents:
            msg = 'The %s file contains hard-wired IPs.' % fn
            raise CheckFailed(msg, l)

        if '.local' in contents:
            msg = 'The %s file contains hard-wired host names.' % fn
            raise CheckFailed(msg, l)
        
        hostname = socket.gethostname()
        
        if not hostname in contents:
            msg = 'The %s file does not contain an entry for your hostname %r.' % (fn, hostname)
            raise CheckFailed(msg, l)
        
        return contents
        