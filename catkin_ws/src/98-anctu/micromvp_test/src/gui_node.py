#!/usr/bin/env python

import rospy
import rospkg
from micromvp_test.msg import micromvp_carspeed
from micromvp_test.msg import micromvp_carspeedArray
from micromvp_test.msg import micromvp_pose
from micromvp_test.msg import micromvp_poseArray
import pygame
from pygame.locals import *
from pgu import gui
from random import random
import math
import os
import sys
import argparse
import threading
import time
import imp
from os import listdir
from os.path import isfile, join
from munkres import Munkres
from Queue import Queue
from micromvp_test import utils
from micromvp_test import positionZMQSub
from micromvp_test import DDR
#import utils
#import DDR
#import positionZMQSub
 
lock = threading.Lock()

# Simulation screen
class Painter(gui.Widget):
    def __init__(self,**params):
        gui.Widget.__init__(self,**params)       
        self.package_path = rospkg.RosPack().get_path('micromvp_test') #setup path for load image etc.
        self.surface = None
        self.font = pygame.font.SysFont("comicsansms", int(utils.wheelBase))
        self.carImage = pygame.image.load(self.package_path + "/src/images/carImage.png")
        self.carImage = pygame.transform.smoothscale(self.carImage, (int(utils.wheelBase * 2.0 / (9.0 / 8.0)), int(utils.wheelBase * 2.0)))
        self.bound = None
        self.pointPool = Queue()
        self.record = False

    def init(self, v):
        # thread lock for screen 2
        self.lock2 = threading.Lock()
        self.surface = pygame.Surface((int(v['width']),int(v['height'])))
        self.surface.fill(utils.RGB_WHITE)
        self.repaint()
        self.t1 = threading.Thread(target = self.recordMouse)
        self.t1.setDaemon(True)
        self.showMode = 0
    
    def paint(self, s):
        # Update the screen
        s.blit(self.surface,(0,0))

    def event(self, e):
        # Screen 2 event handler (basically the mouse movement)
        if not self.surface: return
        
        if e.type == gui.MOUSEBUTTONDOWN:
            with self.lock2:
                self.record = True
        elif e.type == gui.MOUSEBUTTONUP:
            with self.lock2:
                self.record = False

    def recordMouse(self):
        # Record the position of mouse, if leftbutton is down
        rec = False
        while True:
            with self.lock2:
                rec = self.record
            if rec:
                self.pointPool.put(pygame.mouse.get_pos())
            time.sleep(0.01)

    def draw(self, locs, paths):
        # Draw screen 2
        self.surface.fill(utils.RGB_WHITE)

        with self.lock2:
            sm = self.showMode

        # Draw Boundary
        pygame.draw.rect(self.surface, utils.RGB_GREY, self.bound, 1)

        # Show the grid for mrpp (can be deleted if you dont use mrpp)
        if sm == 1:
            for i in range(len(utils.gridCopy)):
                for j in range(len(utils.gridCopy[0]) - 1):
                    if i % 2 != j % 2:
                        continue 
                    pygame.draw.line(self.surface, utils.RGB_BLACK, utils.gridCopy[i][j], utils.gridCopy[i][j + 1])
            for i in range(len(utils.gridCopy) - 1):
                for j in range(len(utils.gridCopy[0])):
                    pygame.draw.line(self.surface, utils.RGB_BLACK, utils.gridCopy[i][j], utils.gridCopy[i + 1][j])
            for i in range(len(utils.gridCopy)):
                for j in range(len(utils.gridCopy[0])):
                    pygame.draw.circle(self.surface, utils.RGB_BLACK, (int(utils.gridCopy[i][j][0]), int(utils.gridCopy[i][j][1])), 3, 0)

        # Draw Path
        for index, path in enumerate(paths):
            if len(path) <= 1:
                continue
            if sm != 2:
                pygame.draw.lines(self.surface, utils.RGB_PATH_COLORS[index], False, path, 5)
            text = self.font.render("Goal"+ str(locs[index][3]), 1, utils.RGB_PATH_COLORS[index])
            self.surface.blit(text, path[-1])
            pygame.draw.circle(self.surface, utils.RGB_PATH_COLORS[index], (int(path[-1][0]), int(path[-1][1])), 5, 0)

        # Draw Car
        for index, stat in enumerate(locs):
            car = pygame.transform.rotate(self.carImage, - 180.0 * stat[2] / math.pi - 90)
            text = self.font.render(str(stat[3]), 1, utils.RGB_PATH_COLORS[index])
            self.surface.blit(car, (stat[0] - utils.wheelBase, stat[1] - utils.wheelBase)) 
            self.surface.blit(text, (stat[0] - utils.wheelBase / 2, stat[1] - utils.wheelBase / 2))    

        # Collosion Detection
        for ind1, i in enumerate(locs):
            for ind2, j in enumerate(locs):
                if ind1 == ind2:
                    continue
                else:
                    if utils.CheckCollosion(utils.wheelBase * 1.5, i[0], i[1], j[0], j[1]):
                        text = self.font.render("TOO CLOSE!", 1, utils.RGB_BLACK)
                        text_width, text_height = self.font.size("TOO CLOSE!")
                        self.surface.blit(text, ((i[0] + j[0]) / 2 - text_width / 2, (i[1] + j[1]) / 2 - text_height / 2))

        self.repaint()  


# Main screen
class App(gui.Desktop):
    def __init__(self, **params):
        gui.Desktop.__init__(self, **params)
        #set up publisher
        self.carpub=rospy.Publisher("~carspeeds", micromvp_carspeedArray, queue_size=1)
        #set up subscriber for car location (Apriltags)
        self.carloc = rospy.Subscriber("erickiemocap/tag_detections_poses", micromvp_poseArray, self.ros_GetLocation, queue_size=1)
        #setup path for load image etc.
        self.package_path = rospkg.RosPack().get_path('micromvp_test')
        #self.connect(gui.QUIT, self.FlushQuit, None)
        self.connect(gui.QUIT, self.shutdown, None)
        # Setup everything
        self.SetupArgv()
        self.SetupCars()
        self.SetupGUI()
        # test location
        #self.test_Location()
    '''
    def FlushQuit(self, garbage):
        with lock:
            if not self.sim:
                self.xBee.xBeeFlush7()
        self.quit()
    '''

    def SetupArgv(self):
        #parser = argparse.ArgumentParser(description = "microMVP")
        #parser.add_argument("-s", dest = "sim", action = "store_true", default = False, help = "Simulation Mode")
        #args = parser.parse_args()
        #self.sim = bool(os.environ.get("sim", args.sim))
        self.sim = False
        self.vMax = 1.0
        #self.simSpeed = utils.simSpeed
        #if not self.sim:
        self.simSpeed = 1
        self.runCar = False
        #if not self.sim:
        #    self.xBee = xBee.Bee(utils.xBeeSocket)
        #    self.xBee.xBeeFlush7()
        self.syn = False
        #for first get location
        self.firstgetlocation = True

    def SetupCars(self):
        self.cars = dict()
        #print 'Car dict: ', self.cars
        #print 'CarInfo: ', utils.carInfo
        for item in utils.carInfo:
            self.cars[item[1]] = utils.UnitCar(tag = item[1], ID = item[0])
        '''
        if self.sim:
            self.SetupWB()
            #self.GetRandomArrangement()
            self.test_Location()
        else:
            self.test_Location()
            self.SetupWB()
        '''
        self.SetupWB()
        self.test_Location()

    def SetupWB(self):
        # Auto-detect the size of cars
        # new version: set WB in utils.py
        '''
        if self.sim:
            pass
        else:
            utils.wheelBase -= 30
            data = positionZMQSub._get_all_car_position_data()
            read = 0
            for garbage, key in utils.carInfo:
                if data[key] == "":
                    continue
                else:
                    read += 1
                    x0, y0, x1, y1, x2, y2, x3, y3 = data[key].split()
                    tagSize = math.sqrt(math.pow(float(x0) - float(x1), 2) + math.pow(float(y0) - float(y1), 2)) 
                    utils.wheelBase += tagSize / utils.tagRatio
            utils.wheelBase = utils.wheelBase / read  
            #print utils.wheelBase
        '''
        tag_size = 20
        utils.wheelBase = tag_size / utils.tagRatio
        self.bound = utils.Boundary()
        #self.firstgetlocation = False

    def SetupGUI(self):
        c = gui.Container(width = utils.container_width, height = utils.container_height)

        # Dialogs
        self.quit_d = utils.QuitDialog()
        self.quit_d.connect(QUIT, self.quit, None)
        self.help_d = utils.HelpDialog()
        self.about_d = utils.AboutDialog()
        
        # Main menu
        menus = self.SetupMenus()        
        c.add(menus, 0, 0)
        menus.rect.w, menus.rect.h = menus.resize()

        # Toolbox
        ctrls = self.SetupToolbox()
        c.add(ctrls, 0, menus.rect.bottom + utils.spacer)

        ctrls.rect.x, ctrls.rect.y = ctrls.style.x, ctrls.style.y
        ctrls.rect.w, ctrls.rect.h = ctrls.resize()

        self.painter = Painter(width = utils.painter_width, height = utils.painter_height, style={'border': 2})
        self.painter.bound = pygame.Rect(self.bound.l, self.bound.u, self.bound.width, self.bound.height)
        self.x_offset = ctrls.rect.w + utils.spacer
        self.y_offset = menus.rect.h + utils.spacer
        c.add(self.painter, self.x_offset, self.y_offset)
        self.painter.init({'width': utils.painter_width,'height': utils.painter_height})
        self.painter.rect.w, self.painter.rect.h = self.painter.resize()

        self.widget = c
        self.painter.t1.start()
        self.t1 = threading.Thread(target = self.ReadMouse)
        self.t1.setDaemon(True)
        self.t1.start()

    def SetupMenus(self):
        self.menus = menus = gui.Menus([
            ('File/Exit',self.quit_d.open,None),
            ('Help/Help',self.help_d.open,None),
            ('Help/About',self.about_d.open,None),
            ])
        return menus

    def SetupToolbox(self):
        self.ctrls = ctrls = gui.Table(width = 125)

        ctrls.tr()
        ctrls.td(gui.Label(" Car Control: "), align = 0)

        ctrls.tr()
        btn_run = gui.Button("Run", width = 90)
        btn_run.connect(gui.CLICK, self.B_run)
        ctrls.td(btn_run)

        ctrls.tr()
        btn_stop = gui.Button("Stop", width = 90)
        btn_stop.connect(gui.CLICK, self.B_stop)
        ctrls.td(btn_stop)

        ctrls.tr()
        btn_clear = gui.Button("Clear", width = 90)
        btn_clear.connect(gui.CLICK, self.B_clear)
        ctrls.td(btn_clear)

        ctrls.tr()
        self.sli_v = gui.HSlider(value= 100,min=0,max = 100,size=20,width=120)
        ctrls.td(self.sli_v, colspan=3)

        ctrls.tr()
        ctrls.td(gui.Label(""), align = 0)

        ctrls.tr()
        ctrls.td(gui.Label(" Draw Path: "))

        ctrls.tr()
        self.sel_car = sel_car = gui.Select()
        for item in utils.carInfo:
            sel_car.add("#" + str(item[0]) + ", Tag" + str(item[1]), item[1])
        ctrls.td(sel_car)

        ctrls.tr()
        ctrls.td(gui.Label(""), align = 0)
        
        ctrls.tr()
        ctrls.td(gui.Label(" Patterns: "), align = 0)

        files = [f for f in listdir(self.package_path + "/src/patterns/") if isfile(join(self.package_path + "/src/patterns/", f))]
        ctrls.tr()
        self.sel_ptn = sel_ptn = gui.Select()
        for f in files:
            if ".py" in f:
                if ".pyc" not in f:
                    sel_ptn.add(f.split(".")[0], f)
        ctrls.td(sel_ptn)

        ctrls.tr()
        btn_pattern = gui.Button("Get Pattern", width = 90)
        btn_pattern.connect(gui.CLICK, self.B_pattern)
        ctrls.td(btn_pattern)

        ctrls.tr()
        ctrls.td(gui.Label(""), align = 0)

        ctrls.tr()
        ctrls.td(gui.Label("Path Planning:"), align = 0)

        files = [f for f in listdir(self.package_path + "/src/algorithms/") if isfile(join(self.package_path + "/src/algorithms/", f))]
        ctrls.tr()
        self.sel_alg = sel_alg = gui.Select()
        for f in files:
            if ".py" in f:
                if ".pyc" not in f:
                    sel_alg.add(f.split(".")[0], f)
        ctrls.td(sel_alg)
        
        ctrls.tr()
        btn_plan = gui.Button("Run ALG", width = 90)
        btn_plan.connect(gui.CLICK, self.B_plan)
        ctrls.td(btn_plan)

        return ctrls

    def B_run(self):
        with lock:
            self.runCar = True

    def B_stop(self):
        with lock:
            self.runCar = False

    def B_clear(self):
        for i in range(3):
            with lock:
                for key in self.cars.keys():
                    self.cars[key].path = []
            #    if not self.sim:
            #        self.xBee.xBeeFlush7()
            time.sleep(0.03)
        with self.painter.lock2:
                self.painter.showMode = 0

    def B_pattern(self):
        if self.sel_ptn.value == None:
            return
        self.B_stop() 
        mod = imp.load_source("", self.package_path + "/src/patterns/" + self.sel_ptn.value)
        paths = mod.GetPath(len(self.cars.keys()), self.bound)
        locs = [0 for x in range(len(self.cars.keys()))]
        with lock:
            for i, j in enumerate(self.cars.keys()):
                locs[i] = (self.cars[j].x, self.cars[j].y)
        paths2 = self.Shuffle(locs, paths)
        paths2 = self.Refinement(paths2)  

        mod = imp.load_source("", self.package_path + "/src/algorithms/rvo2.py")
        paths1 = mod.GetPath(locs, [p[0] for p in paths2], utils.wheelBase, self.bound)
        paths1 = self.Refinement(paths1)

        pathsm = [[paths1[i][-1], paths2[i][0]] for i in range(len(paths2))]
        pathsm = self.Refinement(pathsm)

        paths = list()
        for i in range(len(paths2)):
            paths.append(paths1[i] + pathsm[i] + paths2[i])
        with self.painter.lock2:
            self.painter.showMode = 0
        with lock:
            self.syn = True
            for i, j in enumerate(self.cars.keys()):
                self.cars[j].path = paths[i]

    def B_plan(self):
        if self.sel_alg.value == None:
            return
        self.B_stop()
        mod = imp.load_source("", self.package_path + "/src/algorithms/" + self.sel_alg.value)
        locs = [0 for x in range(len(self.cars.keys()))]
        with lock:
            for i, j in enumerate(self.cars.keys()):
                locs[i] = (self.cars[j].x, self.cars[j].y)
        #print 'Get algorithm path'
        #self.publisher.publish("Get the path")
        paths = mod.GetPath(locs, self.GetRandomGoals(), utils.wheelBase,self.bound)
        ''' print path
        for i in range(0, len(paths[0]), len(paths[0])/100):
            print paths[0][i]
        print 'path[0]: ', len(paths[0])
        for i in range(0, 100):
            print paths[0][i]
        print 'path[2]: ', len(paths[2])
        for i in range(0, len(paths[2]), len(paths[2])/100):
            print paths[2][i]
        paths = self.Refinement(paths)
        print 'path[0]: ', len(paths[0])
        for i in range(0, len(paths[0]), len(paths[0])/50):
            print paths[0][i]
        print ''
        print 'path[2]: ', len(paths[2])
        for i in range(0, len(paths[2]), len(paths[2])/50):
            print paths[2][i]
        '''
        with self.painter.lock2:
            if self.sel_alg.value == "mrpp.py":
                self.painter.showMode = 1
            elif self.sel_alg.value == "rvo2.py":
                self.painter.showMode = 2
        with lock:
            self.syn = True
            for i, j in enumerate(self.cars.keys()):
                self.cars[j].path = paths[i]

    def Shuffle(self, locs, paths):
        matrix = list()
        for index, loc in enumerate(locs):
            matrix.append(list())
            for path in paths:
                matrix[-1].append(math.sqrt(math.pow(loc[0] - path[0][0], 2) + math.pow(loc[1] - path[0][1], 2)))
        m = Munkres()
        indexes = m.compute(matrix)
        newPath = list()
        for row, column in indexes:
            newPath.append(paths[column])
        return newPath

    def Refinement(self, paths):
        """ Make the paths more detailed """
        #print 'path Refinement'
        length = 0
        for path in paths:
            if len(path) > length:
                length = len(path)
        for path in paths:
            while len(path) < length:
                print 'path[-1]: '. path[-1]
                path.append(path[-1])
        total = 0.0
        num = 0
        for path in paths:
            for i in range(len(path) - 1):
                if path[i] != path[i + 1]:
                    total += math.sqrt(math.pow(path[i][0] - path[i + 1][0], 2) + math.pow(path[i][1] - path[i + 1][1], 2))
                    num += 1
        if num == 0:
            return paths
        pts = (int(total / num) + 1) / 4
        if pts == 0:
            return paths
        newPath = [list() for path in paths]
        for index, path in enumerate(paths):
            for i in range(len(path) - 1):
                newPath[index].append(path[i])
                stepX = (path[i + 1][0] - path[i][0]) / pts
                stepY = (path[i + 1][1] - path[i][1]) / pts
                for j in range(pts):
                    newPath[index].append((newPath[index][-1][0] + stepX, newPath[index][-1][1] + stepY))
            newPath[index].append(path[-1])
        return newPath

    def GetRandomArrangement(self):
        # Random locations without colliding, only in simulation mode
        starts = list()
        #print 'len of sefcarskeys: ', len(self.cars.keys())
        for i in range(len(self.cars.keys())):
            inserted = False
            while not inserted:
                newX = self.bound.width * random() + self.bound.l
                newY = self.bound.height * random() + self.bound.u
                if self.NoCollision(starts, newX, newY):
                    starts.append((newX, newY, random() * math.pi))
                    inserted = True
                else:
                    pass
        for index, item in enumerate(self.cars.keys()):
            (
                self.cars[item].x, self.cars[item].y, self.cars[item].theta) = starts[index]

    def GetRandomGoals(self):
        # Random goals without colliding
        goals = list() 
        for i in range(len(self.cars.keys())):
            inserted = False
            while not inserted:
                newX = self.bound.width * random() + self.bound.l
                newY = self.bound.height * random() + self.bound.u
                if self.NoCollision(goals, newX, newY):
                    goals.append((newX, newY))
                    inserted = True
                else:
                    pass
        return goals

    def NoCollision(self, l, x, y):
        # Check if collision occurs
        for obj in l:
            if utils.CheckCollosion(2 * utils.wheelBase, x, y, obj[0], obj[1]):
                return False
        return True

    def Synchronize(self, speeds, paths):
        # Make the overall speed equal to each other
        length = 0
        for i, j in enumerate(paths):
            if len(j) > length:
                length = len(j)
        for i, j in enumerate(paths):
            diff = length - len(j)
            diff = float(16 - diff) / 16.0
            if diff < 0.0:
                diff = 0.0
            vL = speeds[i][0] * math.sqrt(diff)
            vR = speeds[i][1] * math.sqrt(diff)
            speeds[i] = (vL, vR)

    def ReadMouse(self):
        # Get the mouse trajectory
        pt = (0.0, 0.0)
        prev = None
        while True:
            if self.sel_car.value == None:
                pass
            elif self.sel_car.value != prev:
                prev = self.sel_car.value
            else:
                try:
                    pt = self.painter.pointPool.get_nowait()
                    with lock:
                        self.cars[self.sel_car.value].path.append((pt[0] - self.x_offset, pt[1] - self.y_offset))
                        self.syn = False
                        with self.painter.lock2:
                            self.painter.showMode = 0
                except:
                    pass
            time.sleep(0.01)

    def Draw(self):
        locs = [0 for x in range(len(self.cars.keys()))]
        paths = [0 for x in range(len(self.cars.keys()))]
        #print 'Draw locs: ', locs
        #print 'Draw paths: ', paths
        while True:           
            with lock:
                for i, j in enumerate(self.cars.keys()):
                    locs[i] = (self.cars[j].x, self.cars[j].y, self.cars[j].theta, self.cars[j].ID)
                    paths[i] = list(self.cars[j].path)
            self.painter.draw(locs, paths)
            time.sleep(0.015)

    def Follow(self):
        locs = [0 for x in range(len(self.cars.keys()))]
        paths = [0 for x in range(len(self.cars.keys()))]
        speeds = [(0.0, 0.0) for x in range(len(self.cars.keys()))]
        vM = 1.0
        while True:            
            with lock:
                syn = self.syn
                for i, j in enumerate(self.cars.keys()):
                    locs[i] = (self.cars[j].x, self.cars[j].y, self.cars[j].theta)
                    paths[i] = list(self.cars[j].path)
                    vM = self.sli_v.value / 100.0 * self.vMax
            for i, j in enumerate(locs):
                speeds[i] = DDR.Calculate(j[0], j[1], j[2], paths[i], vM, utils.wheelBase)
            if syn:
                self.Synchronize(speeds, paths)
            with lock:
                for i, j in enumerate(self.cars.keys()):
                    self.cars[j].lSpeed = speeds[i][0]
                    self.cars[j].rSpeed = speeds[i][1]
                    self.cars[j].path = paths[i]
            time.sleep(0.02)
    '''
    def Follow(self):
        locs = [0 for x in range(len(self.cars.keys()))]
        paths = [0 for x in range(len(self.cars.keys()))]
        speeds = [(0.0, 0.0) for x in range(len(self.cars.keys()))]
        vM = 1.0
        while True:            
            with lock:
                syn = self.syn
                for i, j in enumerate(self.cars.keys()):
                    locs[i] = (self.cars[j].x, self.cars[j].y, self.cars[j].theta)
                    paths[i] = list(self.cars[j].path)
                    vM = self.sli_v.value / 100.0 * self.vMax
            for i, j in enumerate(locs):
                speeds[i] = DDR.Calculate(j[0], j[1], j[2], paths[i], vM, utils.wheelBase)
            if syn:
                self.Synchronize(speeds, paths)
            with lock:
                for i, j in enumerate(self.cars.keys()):
                    self.cars[j].lSpeed = speeds[i][0]
                    self.cars[j].rSpeed = speeds[i][1]
                    self.cars[j].path = paths[i]
            time.sleep(0.02)
    '''

    def ros_GetLocation(self, msg):
        #ros subscriber get location
        #msg
        # setWB and GUI
        compensation = -1*math.pi/2
        KEY = self.cars.keys()
        #locs = [(0, 0, 0) for x in range(len(self.cars.keys()))]
        #make every xyo become origin and than allocate xyo
        for i, j in enumerate(self.cars.keys()):
            self.cars[j].x = 0
            self.cars[j].y = 0
            self.cars[j].theta = 0 + compensation
        for index in range(len(msg.poses)):
            for carID, tagID in enumerate(self.cars.keys()):
                if msg.poses[index].id == self.cars[tagID].tag:
                    self.cars[tagID].x = msg.poses[index].x
                    self.cars[tagID].y = msg.poses[index].y
                    self.cars[tagID].theta = msg.poses[index].o + compensation
                    break
        #for i, j in enumerate(self.cars.keys()):
        #    self.cars[j].x, self.cars[j].y, self.cars[j].theta = locs[i]
        self.SendSpeed()

    def test_Location(self):
        compensation = -1*math.pi/2
        for i, j in enumerate(self.cars.keys()):
            self.cars[j].x, self.cars[j].y, self.cars[j].theta = (160, 120, compensation+math.pi/2)

    '''    
    def GetLocation(self):
        with lock:
            #for i, j in enumerate(self.cars.keys()):
                #self.cars[j].x, self.cars[j].y, self.cars[j].theta = locs[i]
        
            KEY = self.cars.keys()
        if self.sim:
            locs = [0 for x in range(len(self.cars.keys()))]
            speeds = [(0.0, 0.0) for x in range(len(self.cars.keys()))]
            run = True
            while True:
                with lock:
                    run = self.runCar
                    #print 'GetLocation lock'
                    for i, j in enumerate(self.cars.keys()):
                        locs[i] = (self.cars[j].x, self.cars[j].y, self.cars[j].theta)
                        speeds[i] = (self.cars[j].lSpeed * self.simSpeed, self.cars[j].rSpeed * self.simSpeed)
                    #print 'GetLocation lock end'
                if run:
                    for i, j in enumerate(locs):
                        locs[i] = DDR.Simulate(j[0], j[1], j[2], speeds[i][0], speeds[i][1], utils.wheelBase)
                    with lock:
                        for i, j in enumerate(self.cars.keys()):
                            (self.cars[j].x, self.cars[j].y, self.cars[j].theta) = locs[i]
                else:
                    time.sleep(0.05)        
                time.sleep(0.02)
        else:
            locs = [(0, 0, 0) for x in range(len(self.cars.keys()))]
            while True:
                data = positionZMQSub._get_all_car_position_data()
                for index, key in enumerate(KEY):
                    if len(data[key]) == 0:
                        continue
                    x0, y0, x1, y1, x2, y2, x3, y3 = data[key].split()
                    x0 = float(x0)
                    x1 = float(x1)
                    x2 = float(x2)
                    x3 = float(x3)
                    y0 = float(y0)
                    y1 = float(y1)
                    y2 = float(y2)
                    y3 = float(y3)
                    # Middle
                    x = (x0 + x1 + x2 + x3) / 4.0
                    y = (y0 + y1 + y2 + y3) / 4.0
                    # Front
                    frontMid_x = (x0 + x1) / 2.0
                    frontMid_y = (y0 + y1) / 2.0
                    # Rare
                    rareMid_x = (x2 + x3) / 2.0
                    rareMid_y = (y2 + y3) / 2.0 
                    theta = DDR.calculateATan(frontMid_x - rareMid_x, frontMid_y - rareMid_y)
                    locs[index] = (x, y, theta)
                with lock:
                    for i, j in enumerate(self.cars.keys()):
                        self.cars[j].x, self.cars[j].y, self.cars[j].theta = locs[i]
                time.sleep(0.02)    
        ''' 
    '''     
    def SendSpeed(self):
        car_msg = []
        for i in range(len(utils.carInfo)):
            car_msg.append(micromvp_carspeed())
        
        newList1 = []
        with lock:
            for carID, garbage in utils.carInfo:
                newList1.append(carID)
                car_msg[i].carID = newList1[i]
        speeds = [(0, 0) for x in range(len(self.cars.keys()))]
        run = True
        while True:
            with lock:
                run = self.runCar
                for i, (carID, tagID) in enumerate(utils.carInfo):
                    speeds[i] = (self.cars[tagID].lSpeed * self.simSpeed, self.cars[tagID].rSpeed * self.simSpeed)
            if run:
                print "send speed"
                newList2 = [] #leftspeed
                newList3 = [] #rightspeed
                for i in range(len(utils.carInfo)):
                    newList2.append(speeds[i][0])
                    newList3.append(speeds[i][1])
                    car_msg[i].lspeed = newList3[i]
                    car_msg[i].rspeed = newList2[i]
                    self.carpub[i].publish(car_msg[i])
            else:
                for i in range(len(utils.carInfo)):
                    car_msg[i].lspeed = 0
                    car_msg[i].rspeed = 0
                    self.carpub[i].publish(car_msg[i])
            time.sleep(0.02)
    '''
    def SendSpeed(self):
        car_msg = []
        for i in range(len(utils.carInfo)):
            car_msg.append(micromvp_carspeed())
        
        newList1 = []
        for carID, tagID in utils.carInfo:
            newList1.append(tagID)
            car_msg[carID].tagID = newList1[carID]
        speeds = [(0, 0) for x in range(len(self.cars.keys()))]

        run = self.runCar
        for i, (carID, tagID) in enumerate(utils.carInfo):
            speeds[i] = (self.cars[tagID].lSpeed * self.simSpeed, self.cars[tagID].rSpeed * self.simSpeed)
        if run:
            newList2 = [] #leftspeed
            newList3 = [] #rightspeed
            for i, (carID, tagID) in enumerate(utils.carInfo):
                #newList2.append(speeds[carID][0])
                #newList3.append(speeds[carID][1])
                #car_msg[carID].lspeed = newList3[carID]
                #car_msg[carID].rspeed = newList2[carID]
                car_msg[carID].lspeed = speeds[carID][1]
                car_msg[carID].rspeed = speeds[carID][0]
                if self.cars[tagID].x == 0:
                    car_msg[carID].lspeed = 0
                    car_msg[carID].rspeed = 0
        else:
            for i in range(len(utils.carInfo)):
                car_msg[i].lspeed = 0
                car_msg[i].rspeed = 0
        self.carpub.publish(car_msg)
    #'''

    def shutdown():
        '''
        car_msg = []
        for i in range(len(utils.carInfo)):
            car_msg.append(micromvp_carspeed())
        for i in range(len(utils.carInfo)):
            car_msg[i].lspeed = 0
            car_msg[i].rspeed = 0
            self.carpub[i].publish(car_msg[i])
        '''
        print "microMVP shutdown"
        self.quit()

if __name__ == "__main__":
    rospy.init_node('gui_node')
    app = App()

    t1 = threading.Thread(target = app.Draw)
    t2 = threading.Thread(target = app.Follow)
    #t3 = threading.Thread(target = app.GetLocation)
    #t4 = threading.Thread(target = app.SendSpeed)

    t1.setDaemon(True)
    t2.setDaemon(True)
    #t3.setDaemon(True)
    #t4.setDaemon(True)

    #t3.start()
    t1.start()
    t2.start()
    #t4.start()
    app.run()
    rospy.loginfo("Start Run")

    rospy.on_shutdown(shutdown)
    #rospy.on_shutdown()
    rospy.spin()
