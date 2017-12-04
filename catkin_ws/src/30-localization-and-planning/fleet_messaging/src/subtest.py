# Testing
import commlibs
import commlibs2
import thread

sub = []

sub1 = commlibs.duckie0mq(type = 'sub', port = "5555")
sub2 = commlibs2.duckie0mq(interface = "wlp1s0", type = 'sub')

def recieve(sub):
    while True:
        print(sub.rcv_string())
    return

try:
    thread.start_new_thread(recieve, (sub1,))
    thread.start_new_thread(recieve, (sub2,))
except:
    print("Error on spawning threads")

while True:
    pass
