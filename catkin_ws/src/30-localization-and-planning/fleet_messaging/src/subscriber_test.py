# Testing
import commlibs2
import thread

sub1 = commlibs2.duckiemq(interface="wlp1s0", socktype='sub')
sub2 = commlibs2.duckiemq(interface="wlp1s0", socktype='sub')
sub2.setfilter('ERROR')

def recieve(sub, name):
    while True:
        print(name + ": " + sub.rcv_string())
    return

try:
    thread.start_new_thread(recieve, (sub1, "sub1", ))
    thread.start_new_thread(recieve, (sub2, "sub2", ))
except:
    print("Error on spawning threads")

while True:
    pass
