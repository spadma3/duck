# Testing
from include.fleet_messaging import commlibs2

pub = commlibs2.duckiemq(interface="wlp1s0", socktype='pub')
pub.send_string('yeah from yoga@' + str(pub2.ownip) + ' to everyone but yoga')
pub.send_string('ERROR from yoga@' + str(pub2.ownip) + ' to everyone but yoga')
