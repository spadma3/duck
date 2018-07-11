from phue import Bridge
import time
import os

# Connect to bridge
b = Bridge('192.168.1.5')
b.connect()
# Turn on lights
b.set_group('Megacity', 'on', True)
# Set brightness to max
b.set_group('Megacity', 'bri', 254)

# Get bot name
veh = raw_input("Vehicle name: ")
os.system("./set_ros_master.sh "+str(veh))

# Loop through colorspace
h = 0 # Range: [0,65535]
s = 4 # Range: [0,254]

# Loop through hue range
for h in range(0,65520,720):
    b.set_group('Megacity', 'hue', h)
    # Loop through saturation range
    for s in range(4,254,10):
        b.set_group('Megacity', 'sat', s)
        # Record bag at each point in colorspace
        os.system("./bag_write_line_detect.sh "+str(veh)+" "+str(h)+" "+str(s))
