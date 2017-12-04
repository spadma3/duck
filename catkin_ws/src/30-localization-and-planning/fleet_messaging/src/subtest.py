# Testing
import commlibs
import commlibs2
import multiprocessing

sub = []

sub.append(commlibs.duckie0mq(type = 'sub', port = "5555"))
sub.append(commlibs2.duckie0mq(interface = "wlp1s0", type = 'sub'))

print(sub[0].rcv_string())
print(sub[1].rcv_string())

print(sub[0].rcv_string())
print(sub[1].rcv_string())

def worker(sub,i):
    while True:
        print("Process" + str(i) + ": " + sub[i].rcv_string())
    return

jobs = []
for i in range(2):
    p = multiprocessing.Process(target=worker, args=(sub,i))
    jobs.append(p)
    p.start()
