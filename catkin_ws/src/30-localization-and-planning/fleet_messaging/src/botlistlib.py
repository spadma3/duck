import time
import numpy as np

timestr = str(int(time.time()))
oldtimestr = str(int(time.time())-100)

oldbotlist = np.array([['alice','192.168.0.1','a2r', timestr],['bob','192.168.0.2','j5', timestr],['chad','192.168.0.3','q3l', oldtimestr]], dtype = object)



class botlist(object):
    # Constructor
    def __init__(self, bot = "myname", port = "myport", pos = "a1r"):
        self.bot = bot
        self.port = port
        self.pos = pos
        self.time = int(time.time())
        self.decaytime = 30

        self.me = np.empty([1,4],dtype = object)
        self.me[0,0] = self.bot
        self.me[0,1] = self.port
        self.me[0,2] = self.pos
        self.me[0,3] = self.time

        self.botlist = self.me

    def upd_botlist(self, botlist):

        self.time = int(time.time())
        for i in range(np.shape(botlist)[0]):
            match = np.where(self.botlist[:,0]==botlist[i,0])
            if np.size(match):
                self.botlist[match[0][0],:] = botlist[i,:]
            else:
                addarray = np.empty([1,4],dtype = object)
                addarray[0,0] = botlist[i,0]
                addarray[0,1] = botlist[i,1]
                addarray[0,2] = botlist[i,2]
                addarray[0,3] = botlist[i,3]
                self.botlist = np.append(self.botlist, addarray, axis=0)

        orglen = np.shape(self.botlist)[0]
        for ii in range(orglen):
            i = orglen-ii-1
            if int(self.botlist[i,3])+self.decaytime < int(self.time):
                self.botlist = np.delete(self.botlist, i, 0)

        self.time = int(time.time())
        self.me[0,3] = self.time


        if np.size(self.botlist)>4:
            delindex = np.where(self.botlist[:,0]==self.bot)
            self.botlist = np.delete(self.botlist, delindex, 0)
            self.botlist = np.append(self.me, self.botlist, axis=0)

        else:
            self.botlist = self.me


        return self.botlist


mylist = botlist()

# %%

newbotlist = mylist.upd_botlist(oldbotlist)
print(newbotlist)
