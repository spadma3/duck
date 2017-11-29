# Testing
import commlibs as commlibs

pub = commlibs.duckie0mq(type = 'pub')
pub.send_string('yoyoyoyo')

pub2 = commlibs.duckie0mq(port = "tcp://127.0.0.1:2233", type = 'pub')
pub2.send_string('yoyoyoyi')

#pub2 = commlibs.duckie0mq(port = "pgm://192.168.1.1;239.192.1.1:5555", type = 'pub')
#pub2.send_string('yoyoyoyi')
