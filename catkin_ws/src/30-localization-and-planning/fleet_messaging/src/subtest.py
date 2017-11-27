# Testing
import commlibs as commlibs

testtype = 'sub'
sub = commlibs.duckie0mq(type = 'sub')
sub.connect("tcp://127.0.0.1:2233")
while True:
    print(sub.rcv_string())
