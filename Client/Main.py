#!/usr/bin/python
# -*- coding: utf-8 -*-
import threading
import cv2
import socket
import time
import numpy as np
import signal
import atexit
from networktables import NetworkTables
from enum import Enum

# Only keep the threads running barring a halt
isOn = False
lock = threading.Lock()
class Main:

    """
    Executive class for controlling flow
    """

    SOCK_IP = '10.2.63.5'  # Static IP for Driver's Station on FMS
    SOCK_PORT = 5810  # TCIP Port to use for communication with Driver's Station
    NT_IP = 'roboRIO-263-FRC.local'  # RIO server IP for NetworkTables

    def __init__(self):

    #register exit methods
    atexit.register(Sender.sender_exit)
    atexit.register(ImageRetriever.image_exit)

        """initializes all threads and class instances
        """
        self.image_retrieval = ImageRetriever()
        self.image_processor = FindTape()
        self.image_sender = Sender(self.SOCK_IP, self.SOCK_PORT, self.NT_IP)
        #Create continual threads to run
        self.processor_t = threading.Thread(name='image_processor',
                target=self.image_processor.update,
                args=(self.image_retrieval,self.image_sender))
        self.sender_t = threading.Thread(name='image_sender',
                target=self.image_sender.update_camera_feed,
                args=(self.image_retrieval, ))
       # self.toggle_t = threading.Thread(name = 'image_toggle',
          #      target = self.image_sender.toggleThread,)


    def start(self):
        """responsible for starting threads that will continue until bot is in off state
        """
        self.processor_t.start()
        self.sender_t.start()
        #self.toggle_t.start()


class FindTape:

    filtered = False
    def __init__(self):
        pass

    def update(self, retriever, nt_manager):
        """Continually updates both camera feeds' data through NetworkTables
        """
        global isOn
        while isOn:
        try:
            pGear = self.process(retriever.get_latest_gear())
            pShooter = self.process(retriever.get_latest_shooter())
        except:
            continue
        #needs to be fixed
        if True:
            nt_manager.update_gear_coords(pGear)
    
        if(len(pShooter) == 2):
                nt_manager.update_shooter_coords(pShooter[0],pShooter[1])

    def process(frame): #needs to be fixed, processes given image
        c = 0
        contourL = []
        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

        upper = np.array([45,0,225])
        lower = np.array([46,30,255])

        mask = cv2.inRange(hsv,upper,lower)
        mask = cv2.bitwise_and(frame, frame, mask = mask)
        erode = cv2.erode(mask,(15,15),iterations = 4)
        dilate = cv2.dilate(erode,(15,15 ), iterations = 1)
        self.tape = cv2.erode(dilate,(15,15),iterations = 4)

        gray = cv2.cvtColor(self.tape,cv2.COLOR_BGR2GRAY)
        _,thresh = cv2.threshold(gray,127,255,0)

        x, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            epsilon = 0.02*cv2.arcLength(contour,True)
            approx = cv2.approxPolyDP(contour,epsilon,True)

            if len(approx) > 0 and cv2.contourArea(contour) >  cv2.getTrackbarPos('Min Area','Final'):
                cv2.drawContours(self.tape,[contour],-1,(127,58,254), 3)
                M = cv2.moments(contour)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                centroid = (cx,cy)
                contourL.append(centroid)
                c += 1
                cv2.circle(self.tape,centroid,5,(0,255,0),thickness = -1)
                cv2.putText(self.tape,'Center',(cx+5,cy+5),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),1,cv2.LINE_AA)
            else:
                cv2.drawContours(self.tape,[contour],-1,0,-1)
            if c % 2 == 0:
                return contours 
            self.filtered = True

    def getCoords(self):
        return self.name


class Sender:
    """
    Class responsible for all data sent from Raspberry Pi to both NT server and DS server
    """

    NetworkTables.initialize(server='roboRIO-263-FRC.local')
    gear = NetworkTables.getTable('cameraData/gear')
    shooter = NetworkTables.getTable('cameraData/shooter')
    camera_data = NetworkTables.getTable('cameraData/clientMode')
    #print('tables are initialized')
    FRAMERATE_PERIOD = 1 / 15 #its 0 unless 15 is 15.0
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #toggle = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    def __init__(self,serverIP, serverPort, networktableIP):

        self.mode_gear = True

        while True:
            try:
                #print('Trying to connect to ' + serverIP + ' on port ' + str(serverPort))
                self.sock.connect((serverIP, serverPort))
                #self.toggle.connect(('roboRIO-263-FRC.local',5800))
                break
            except Exception as e:
        pass
                #print('Failed to connect: ' + str(e))
        print('Connected')

    def update_camera_feed(self, retriever):
        global isOn
        while isOn:   
            self.mode_gear = self.camera_data.getBoolean('gearMode', defaultValue=True)
            if self.mode_gear:
                frame = retriever.get_latest_gear()
        else:   
                frame = retriever.get_latest_shooter()
            if frame is None: return
            im = cv2.imencode('.jpg', frame)[1].tostring()
            self.sock.send(im)
            time.sleep(self.FRAMERATE_PERIOD)
        
    @classmethod
    def update_gear_coords(self, contours):
    #print('updating gear coords')
    if (len(contours) is not 2):
       #print('sending -1s')
       self.gear.putNumber('pointOneX', -1)
       self.gear.putNumber('pointOneY', -1)
       self.gear.putNumber('pointTwoX', -1)
       self.gear.putNumber('pointTwoY', -1)
    else: 
        #print('sending other values')
        (p1x,p1y,p1w,p1h) = cv2.boundingRect(contours[0])
        p1x += p1w/2.0
        p1y += p1h/2.0
        (p2x,p2y,p2w,p2h) = cv2.boundingRect(contours[1])
        p2x += p2w/2.0
        p2y += p2h/2.0  

            self.gear.putNumber('pointOneX', p1x)
            self.gear.putNumber('pointOneY', p1y)
            self.gear.putNumber('pointTwoX', p2x)
            self.gear.putNumber('pointTwoY', p2y)
    #print('updated gear coords')
    @classmethod
    def update_shooter_coords(self, pointOne, pointTwo):
    #print('updating shooter coords')
    (p1x,p1y,p1w,p1h) = cv2.boundingRect(pointOne)
        p1x += p1w/2.0
        p1y += p1h/2.0
        (p2x,p2y,p2w,p2h) = cv2.boundingRect(pointTwo)
        p2x += p2w/2.0
        p2y += p2h/2.0
        self.shooter.putNumber('pointOneX', p1x)
        self.shooter.putNumber('pointOneY', p1y)
        self.shooter.putNumber('pointTwoX', p2x)
        self.shooter.putNumber('pointTwoY', p2y)
    #print('updated shooter coords')
    @classmethod
    def sender_exit(self):
       self.sock.close()
       #print('sender closed')

    @classmethod
    def is_match_over():
        return NetworkTables.getTable('cameraData').getBoolean("end", False)
class ImageRetriever:

    
    gearCam = cv2.VideoCapture(-1)
    shooterCam = cv2.VideoCapture(1)
    def __init__(self):
        for x in range(10):
            #print('Connecting to camera in ' + str(x) + '/10 seconds')
        time.sleep(1)
        self.gearCam.set(cv2.CAP_PROP_FRAME_WIDTH, 360)
        self.gearCam.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.gearCam.set(cv2.CAP_PROP_FPS, 15)
        self.gearCam.set(cv2.CAP_PROP_EXPOSURE, 20)

        self.shooterCam.set(cv2.CAP_PROP_FRAME_WIDTH, 360)
        self.shooterCam.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.shooterCam.set(cv2.CAP_PROP_FPS, 15)
        self.shooterCam.set(cv2.CAP_PROP_EXPOSURE, 20)

        #self.vOut = cv2.VideoWriter('output.avi', -1, 15.0, (360, 240))

    @classmethod
    def get_latest_gear(self):
        if Sender.is_match_over():
            #vOut.release()
    lock.acquire()
        frame = self.gearCam.read()[1]
    time.sleep(0.05)
    #print('got latest gear: ' + str(len(frame)))
    lock.release()
    return frame
        
    @classmethod
    def get_latest_shooter(self):
    lock.acquire()
    frame = self.shooterCam.read()[1]
    time.sleep(0.05)
    #print('got latest shooter: ' + str(len(frame)))
    lock.release()
        return frame
    
    @classmethod
    def image_exit(self):
    self.shooterCam.release()
    self.gearCam.release()
    #print('image closed')


if __name__ == '__main__':
    global isOn
    isOn = True
    executive = Main()
    executive.start()

            
        
        
        
