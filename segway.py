# -*- coding: utf-8 -*-
#A little program to use the Cue bot as a segway, very much in progress....
from threading import Thread
import WonderPy.core.wwMain
import numpy as np
import time
from WonderPy.core.wwConstants import WWRobotConstants


class MyClass(object):
    def __init__(self):
        #PID controller constants
        self.kp            = 2.1
        self.ki            = 1
        self.kd            = 0#-0.01

        self.fric = 1. #Helps to overcome static friction

        #The rest are variables to help with calculations
        self.loopTimeSec= 0.04 #length of PID loop
        self.speed = 0
        self.gyroAngleBuffLen = 5
        self.gyroAngleBuff = np.zeros(self.gyroAngleBuffLen)
        self.gyroAngleErrorAccumulated=0

        self.setPoint = 0
        self.err = 0
        self.lastLoopStart=0
        self.lastLoopDur=0

        self.senseLoopDur=0
        self.senseLoopLast=0

    def on_sensors(self, robot):
        #Timer for determineing how long between sensor checks
        self.senseLoopDur=time.time()-self.senseLoopLast
        self.senseLoopLast=time.time()

    def on_connect(self, robot):
        print("Starting a thread for %s." % (robot.name))
        Thread(target=self.thread_mover, args=(robot,)).start()
        # Thread(target=self.thread_print, args=(robot,)).start()

    def thread_print(self, robot):
        while True:
            print round(self.speed,1)


    def thread_mover(self, robot):
        #Set up: LEDs turn green, user balances bot, then lights turn off and control begins
        setUp=True
        robot.cmds.RGB.stage_all(0, 100, 200)
        setupTime=time.time()
        while setUp:
            self.setPoint = robot.sensors.accelerometer.degrees_z_yz()
            if time.time()-setupTime >5:
                setUp = False
        robot.cmds.RGB.stage_all(0, 0, 0)

        # PID Control loop
        while True:
            self.lastLoopStart = time.time()
            self.gyroAngleBuff[:-1] = self.gyroAngleBuff[1:]
            self.gyroAngleBuff[-1] = robot.sensors.accelerometer.degrees_z_yz()

            gyroRate = (self.gyroAngleBuff[-1] - self.gyroAngleBuff[-5])/(self.loopTimeSec*5)
            gyroAngleErr = self.setPoint - self.gyroAngleBuff[-1]

            #calculate speed from sensor values
            self.speed = gyroAngleErr*self.kp + gyroRate*self.kd + np.sum(self.setPoint-self.gyroAngleBuff)/self.gyroAngleBuffLen*self.ki

            #Add friction helper and bound max speeds
            self.speed = min(50, self.speed + self.fric) if self.speed > 0 else  max(-50,self.speed- self.fric)

            robot.cmds.body.stage_wheel_speeds_naive(self.speed, self.speed)

            #wait so that every loop is same duratio
            while(time.time()-self.lastLoopStart <  self.loopTimeSec):
                time.sleep(0.0001)
            self.lastLoopDur=time.time()-self.lastLoopStart

if __name__ == "__main__":
    WonderPy.core.wwMain.start(MyClass())
