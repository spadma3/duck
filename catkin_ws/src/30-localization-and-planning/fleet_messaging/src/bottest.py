# Testing
import commlibs
import botlistlib
import numpy as np


listport = '232r3'
mapport = '34342'

mylist = botlistlib.botlist()

listpub = commlibs.duckie0mq(port = listport, type = 'pub')
listsub = commlibs.duckie0mq(port = listport, type = 'sub')

mappub = commlibs.duckie0mq(port = mapport, type = 'pub')
mapsub = commlibs.duckie0mq(port = mapport, type = 'sub')

map = mapsub.recv_serialized()
#do sth with map
mappub.send_serialized(mymap)

newlist = listsub.recv_serialized()
mylist.upd_botlist(newlist)

#generate random port
#if port not in mylist.botlist[:,1]
#mylist.port = port


mysub = commlibs.duckie0mq(port = mylist.port, type = 'sub')

mypub = commlibs.duckie0mq(port = "whatever", type = 'pub')
