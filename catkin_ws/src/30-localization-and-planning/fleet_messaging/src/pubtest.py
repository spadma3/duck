# Testing
import commlibs
import commlibs2

pub0 = commlibs.duckie0mq(subip = "192.168.40.10", port = "5555", type = 'pub')
pub0.send_string('hi from yoga to omon')

pub1 = commlibs.duckie0mq(subip = "192.168.40.2", port = "5555", type = 'pub')
pub1.send_string('yo from yoga to yoga')

pub2 = commlibs2.duckie0mq(interface = "wlp1s0", type = 'pub')
pub2.send_string('yeah from yoga to everyone but yoga')
