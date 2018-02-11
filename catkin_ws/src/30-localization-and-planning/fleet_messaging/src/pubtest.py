# Testing
import commlibs
import commlibs2

# get ips and interface name with ifconfig

# subip is ip of the bot in your network
pub0 = commlibs.duckie0mq(subip = "192.168.40.10", port = "5555", type = 'pub')
pub0.send_string('hi from laptop to bot')

# subip is ip of laptop in your network
pub1 = commlibs.duckie0mq(subip = "192.168.40.2", port = "5555", type = 'pub')
pub1.send_string('yo from laptop to laptop')

# interface is name of the network interface of the network you use, if left empty it's wlan0 (should work for the duckiebot)
pub2 = commlibs2.duckie0mq(interface = "wlp1s0", type = 'pub')
pub2.send_string('yeah from laptop to everyone but laptop')
