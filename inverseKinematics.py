
from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime

import ctypes
import _ctypes
import pygame
import sys

if sys.hexversion >= 0x03000000:
    import _thread as thread
else:
    import thread
# colors for drawing different bodies 
from pygame.locals import *

from OpenGL.GL import *
from OpenGL.GLU import *

import numpy as np
import math
import serial
import struct
import time
import socket



'''This program allows for the control logic of the robot arm and its visualization.
The robot takes in input from the kinect about the joint angles of a person, converts them 
into coordinates in the robots workspace, implements an inverse kinematics algorithm 
to figure out which configuration of motor angles will allow the robot to reach that position.
Finally that configuration is visualized using pyopengl and forward kinematics. Lastly, the 
motor angles are sent to a processing program via sockets that receives this data and 
feeds it to the arduino.'''

#draw text function taken from https://www.pygame.org/wiki/CrossPlatformTextOpengl
def drawText(position, textString):     
    font = pygame.font.Font (None, 64)
    textSurface = font.render(textString, True, (255,255,255,255), (0,0,0,255))     
    textData = pygame.image.tostring(textSurface, "RGBA", True)     
    glRasterPos3d(*position)     
    glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)


def rM1(x1,L1):
    '''x1 is the angle of the rotation joint for the chest and L is the displacement from the chest till shoulder,
    basically like a color bone'''
    x = math.radians(x1)
    c1 = math.cos(x)
    s1 = math.sin(x)
    return np.array([[c1,-s1,0,0],
                      [s1,c1,0,0],
                      [0,0,1,L1],
                      [0,0,0,1]])

def rM2Right(x2):
    '''x2 is the angle of the rotation joint for the shoulder'''
    x = math.radians(x2)
    c2 = math.cos(x)
    s2 = math.sin(x)
    return np.array([[s2,c2,0,0],
                      [0,0,1,0],
                      [c2,-s2,0,0],
                      [0,0,0,1]])
def rM4(x4):
    '''x3 is the angle of elbow joint and L2 is the distance from the shoulder till the elbow'''
    x = math.radians(x4)
    c4 = math.cos(x)    
    s4 = math.sin(x)
    return np.array([[c4,-s4,0,0],
                     [0,0,-1,0],
                     [s4,c4,0,0],
                     [0,0,0,1]])

def rM3Right(x3,L3):
    '''x1 is the angle of the rotation joint for the chest and L is the displacement from the chest till shoulder,
    basically like a color bone'''
    x = math.radians(x3)
    c3 = math.cos(x)
    s3 = math.sin(x)
    return np.array([[1,0,0,L3],
                      [0,c3,-s3,0],
                      [0,s3,c3,0],
                      [0,0,0,1]])

def rM5(L5):
    '''L3 is the distance from the elbow till the hand'''
    return np.array([[1,0,0,L5],
                 [0,1,0,0],
                 [0,0,1,0],
                 [0,0,0,1]])
   
def returnVerticesRight(x1,x2,x3,x4):
    '''returns the vertices of the chest(p0), shoulder(p1), elbow(p2) and hand(p3)'''
    #generating transofrmation matrices to use
    m1 = rM1(x1,4.5)
    m2 = rM2Right(x2)
    m3 = rM3Right(x3,5.25)
    m4 = rM4(x4)
    m5 = rM5(7.75)

    #finding 2 using forward kinematics
    p0 = [0,0,0,1]
    p1 = (m1.dot(m2)).dot(p0)
    p2 = ((m1.dot(m2)).dot(m3)).dot(p0)
    p3 = ((((m1.dot(m2)).dot(m3)).dot(m4)).dot(m5)).dot(p0)
   
    return p0,p1,p2,p3
    

def drawRobotRight(x1,x2,x3,x4):
    '''This function draws the robot, code for this and a lot of the pygame drawing code is adapted from here:
        https://pythonprogramming.net/opengl-rotating-cube-example-pyopengl-tutorial/, code sections on which
        edges to draw, the scale of the scene and the rotation of the scene have all been modified'''
    vertices = returnVerticesRight(x1,x2,x3,x4)
    #print(vertices)
    edges = ((0,1),
    (1,2),
    (2,3))
    glBegin(GL_LINES)
    for edge in edges:
        for vertex in edge:
            glVertex3fv(vertices[vertex][:-1])
    glEnd()

def getVectorRight(s,e,h):
    '''takes coordinates from the kinect and scales it down'''
    s,e,h = np.array(s),np.array(e),np.array(h)
    totalLength = abs(np.linalg.norm(e-s)+np.linalg.norm(h-e))
    relativeLength = abs(np.linalg.norm(h-s))
    s[0],s[1],s[2],h[0],h[1],h[2] = -s[1],-s[2],s[0],-h[1],-h[2],h[0]
    #print(h-s)
    v = h - s
    v = (v/np.linalg.norm(v)*13*(relativeLength/totalLength))
    v[2] += 4.5
    return v
    
def inRangeRight(e):
    '''Makes sure that the coordinate received is achievable in the robots workspace, otherwise maps to the closest point'''
    x = e - np.array([0,0,4.5])
    if np.linalg.norm(x) > 13:
        x = (x/np.linalg.norm(x))*13 + np.array([0,0,4.5])
        return x
    else:
        return e
        
def setBoundsRight(o):
    '''Makes sure no motor angle is out of its physical boundaries'''
    if o[0] < -30:
        o[0] = 0
    elif o[0] > 150:
        o[0] = 150
    if o[1] > 90:
        o[1] = 90
    elif o[1] < -90:
        o[1] = -90
    if o[2] > 90:
        o[2] = 90
    elif o[2] < -90:
        o[2] = -90
    if o[3] < 0:
        o[3] = 0
    elif o[3] > 135:
        o[3] = 135
    return o

def forwardKinematicsRight(o):
    '''Solves the forward kinematics of the robot right hand to find coordinates of the end effector'''
    l = [4.5,5.25,7.75]
    ang = [math.radians(o[0]),math.radians(o[1]),math.radians(o[2]),math.radians(o[3])]
    c1,s1,c2,s2,c3,s3,c4,s4 =  math.cos(ang[0]),math.sin(ang[0]),\
    math.cos(ang[1]),math.sin(ang[1]),math.cos(ang[2]),math.sin(ang[2]),\
    math.cos(ang[3]),math.sin(ang[3])
    x = c1*s2*l[1]+c1*s2*c4*l[2]-s1*c3*s4*l[2]-c1*c2*s3*s4*l[2]
    y = s1*s2*l[1]+s1*s2*c4*l[2]+c1*c3*s4*l[2]-s1*c2*s3*s4*l[2]
    z = c2*l[1]+c2*c4*l[2]+s2*s3*s4*l[2]+l[0]
    return np.array([x,y,z])
    
def comparePositions(e,cP,diff):
    '''Checks how far off current position (cp) is from target position (e)'''
    return abs(e[0]-cP[0]) < diff and abs(e[1]-cP[1]) < diff and abs(e[2]-cP[2]) < diff
    
def inverseKinematicsRight(e,o):
    '''Solves inverse kinematics for right arm'''
    didBreak = False
    e = inRangeRight(e)
    cP = forwardKinematicsRight(o)
    i = 0
    while not(comparePositions(e,cP,0.1)):
        i += 1
        dO = getDeltaOrientationRight(e,o)*25
        o[0] += dO[0]
        o[1] += dO[1]
        o[2] += dO[2]
        o[3] += dO[3]
        cP = forwardKinematicsRight(o)
        if i > 100:
            didBreak = True
            break
    o = setBoundsRight(o)
    return o,didBreak
    
def getDeltaOrientationRight(e,o):
    '''Calculates offset for current position to reach desired positon (e)'''
    JT = np.transpose(computeJacobianRight(o))
    V = e - forwardKinematicsRight(o)
    dO = JT.dot(V)
    return dO
    
def computeJacobianRight(o):
    '''Solves for the jacobian matrix, read more here:https://en.wikipedia.org/wiki/Jacobian_matrix_and_determinant'''
    lst = []
    diff = 0.000000001
    cP = forwardKinematicsRight(o)
    for i in range(len(o)):
        o[i] += diff
        dP = forwardKinematicsRight(o)
        o[i] -= diff
        for j in range(3):
            lst += [(dP[j]-cP[j])/diff]
    return np.array([[lst[0],lst[3],lst[6],lst[9]],[lst[1],lst[4],lst[7],lst[10]],[lst[2],lst[5],lst[8],lst[11]]])

def getCoordinatesRight(joints):
    '''Gets the coordinates for the shoulder,elbow and hand of the right arm from the kinect'''
    sX = joints[PyKinectV2.JointType_ShoulderRight].Position.x 
    sY = joints[PyKinectV2.JointType_ShoulderRight].Position.y 
    sZ = joints[PyKinectV2.JointType_ShoulderRight].Position.z
    s = [sX, sY, sZ]
    eX = joints[PyKinectV2.JointType_ElbowRight].Position.x 
    eY = joints[PyKinectV2.JointType_ElbowRight].Position.y 
    eZ = joints[PyKinectV2.JointType_ElbowRight].Position.z
    e = [eX, eY, eZ]
    hX = joints[PyKinectV2.JointType_WristRight].Position.x 
    hY = joints[PyKinectV2.JointType_WristRight].Position.y 
    hZ = joints[PyKinectV2.JointType_WristRight].Position.z
    h = [hX, hY, hZ]
    return s,e,h


#Code below does the same function as the code above, but just for the left arm, so it is flipped over.

def rM2Left(x2):
    x = math.radians(x2)
    c2 = math.cos(x)
    s2 = math.sin(x)
    return np.array([[s2,-c2,0,0],
                      [0,0,1,0],
                      [-c2,-s2,0,0],
                      [0,0,0,1]])

def rM3Left(x3,L3):
    x = math.radians(x3)
    c3 = math.cos(x)
    s3 = math.sin(x)
    return np.array([[1,0,0,L3],
                      [0,c3,s3,0],
                      [0,-s3,c3,0],
                      [0,0,0,1]])

   
def returnVerticesLeft(x1,x2,x3,x4):
    m1 = rM1(x1,-4.5)
    m2 = rM2Left(x2)
    m3 = rM3Left(x3,5.25)
    m4 = rM4(x4)
    m5 = rM5(7.75)

    #finding 2 using forward kinematics
    p0 = [0,0,0,1]
    p1 = (m1.dot(m2)).dot(p0)
    p2 = ((m1.dot(m2)).dot(m3)).dot(p0)
    p3 = ((((m1.dot(m2)).dot(m3)).dot(m4)).dot(m5)).dot(p0)
   
    return p0,p1,p2,p3
    

def drawRobotLeft(x1,x2,x3,x4):
    vertices = returnVerticesLeft(x1,x2,x3,x4)
    edges = ((0,1),
    (1,2),
    (2,3))
    glBegin(GL_LINES)
    for edge in edges:
        for vertex in edge:
            glVertex3fv(vertices[vertex][:-1])
    glEnd()

def getVectorLeft(s,e,h):
    s,e,h = np.array(s),np.array(e),np.array(h)
    totalLength = abs(np.linalg.norm(e-s)+np.linalg.norm(h-e))
    relativeLength = abs(np.linalg.norm(h-s))
    s[0],s[1],s[2],h[0],h[1],h[2] = -s[1],-s[2],s[0],-h[1],-h[2],h[0]
    #print(h-s)
    v = h - s
    v = (v/np.linalg.norm(v)*13*(relativeLength/totalLength))
    v[2] -= 4.5
    return v

def inRangeLeft(e):
    x = e - np.array([0,0,-4.5])
    if np.linalg.norm(x) > 13:
        x = (x/np.linalg.norm(x))*13 + np.array([0,0,-4.5])
        return x
    else:
        return e
        
def setBoundsLeft(o):
    if o[0] < -30:
        o[0] = 0
    elif o[0] > 150:
        o[0] = 150
    if o[1] > 90:
        o[1] = 90
    elif o[1] < -90:
        o[1] = -90
    if o[2] > 90:
        o[2] = 90
    elif o[2] < -90:
        o[2] = -90
    if o[3] < 0:
        o[3] = 0
    elif o[3] > 135:
        o[3] = 135
    return o

def forwardKinematicsLeft(o):
    l = [-4.5,5.25,7.75]
    ang = [math.radians(o[0]),math.radians(o[1]),math.radians(o[2]),math.radians(o[3])]
    c1,s1,c2,s2,c3,s3,c4,s4 =  math.cos(ang[0]),math.sin(ang[0]),\
    math.cos(ang[1]),math.sin(ang[1]),math.cos(ang[2]),math.sin(ang[2]),\
    math.cos(ang[3]),math.sin(ang[3])
    x = c1*s2*l[1]+c1*s2*c4*l[2]-s1*c3*s4*l[2]-c1*c2*s3*s4*l[2]
    y = s1*s2*l[1]+s1*s2*c4*l[2]+c1*c3*s4*l[2]-s1*c2*s3*s4*l[2]
    z = -c2*l[1]-c2*c4*l[2]-s2*s3*s4*l[2]+l[0]
    return np.array([x,y,z])
    
def comparePositions(e,cP,diff):
    return abs(e[0]-cP[0]) < diff and abs(e[1]-cP[1]) < diff and abs(e[2]-cP[2]) < diff
    
def inverseKinematicsLeft(e,o):
    didBreak = False
    e = inRangeLeft(e)
    cP = forwardKinematicsLeft(o)
    i = 0
    while not(comparePositions(e,cP,0.1)):
        i += 1
        dO = getDeltaOrientationLeft(e,o)*25
        o[0] += dO[0]
        o[1] += dO[1]
        o[2] += dO[2]
        o[3] += dO[3]
        cP = forwardKinematicsLeft(o)
        if i > 100:
            didBreak = True
            break
    o = setBoundsLeft(o)
    return o,didBreak
    
def getDeltaOrientationLeft(e,o):
    JT = np.transpose(computeJacobianLeft(o))
    V = e - forwardKinematicsLeft(o)
    dO = JT.dot(V)
    return dO
    
def computeJacobianLeft(o):
    lst = []
    diff = 0.000000001
    cP = forwardKinematicsLeft(o)
    for i in range(len(o)):
        o[i] += diff
        dP = forwardKinematicsLeft(o)
        o[i] -= diff
        for j in range(3):
            lst += [(dP[j]-cP[j])/diff]
    return np.array([[lst[0],lst[3],lst[6],lst[9]],[lst[1],lst[4],lst[7],lst[10]],[lst[2],lst[5],lst[8],lst[11]]])

def getCoordinatesLeft(joints):
    sX = joints[PyKinectV2.JointType_ShoulderLeft].Position.x 
    sY = joints[PyKinectV2.JointType_ShoulderLeft].Position.y 
    sZ = joints[PyKinectV2.JointType_ShoulderLeft].Position.z
    s = [sX, sY, sZ]
    eX = joints[PyKinectV2.JointType_ElbowLeft].Position.x 
    eY = joints[PyKinectV2.JointType_ElbowLeft].Position.y 
    eZ = joints[PyKinectV2.JointType_ElbowLeft].Position.z
    e = [eX, eY, eZ]
    hX = joints[PyKinectV2.JointType_WristLeft].Position.x 
    hY = joints[PyKinectV2.JointType_WristLeft].Position.y 
    hZ = joints[PyKinectV2.JointType_WristLeft].Position.z
    h = [hX, hY, hZ]
    return s,e,h



def boundAngle(ang):
    '''Makes sure an angle being sent to the robot is within the bounds 180 and 0'''
    if ang > 180:
        return 180
    elif ang < 0:
        return 0
    return ang

def formatAngles(angRight,angLeft):
    '''formats the angles from the visualization into angles that can be given to the real 
    robots motors by taking into account their offset and their real world constraints'''
    angles = angRight + angLeft
    angles[0] += 35
    angles[1] = - angles[1]
    angles[1] += 100
    angles[2] += 90
    angles[4] = 150 - angles[4]
    angles[5] += 90
    angles[6] = -angles[6]
    angles[6] += 90
    angles[7] = 190 - angles[7]
    for i in range(len(angles)):
        angles[i] = int(angles[i])
        angles[i] = boundAngle(angles[i])
    return angles

#Kinect code is using this code as a template and has been highly modified:https://github.com/Kinect/PyKinect2/blob/master/examples/PyKinectBodyGame.py

class BodyGameRuntime(object):
    def __init__(self):
        pygame.init()

        self._clock = pygame.time.Clock()
        #self.ArduinoSerial = serial.Serial('com10',9600) 
        #time.sleep(2)
        self.angRight = [0, 90, 0,0]
        self.angLeft = [0, 90, 0,0]
        self.breakRight = False
        self.breakLeft = False
        pygame.display.set_caption("Kinect for Windows v2 Body Game")
        self._done = False
        self._clock = pygame.time.Clock()
        self._kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Body)
        self._frame_surface = pygame.Surface((self._kinect.color_frame_desc.Width, self._kinect.color_frame_desc.Height), 0, 32)
        self._bodies = None

        #creating a socket so that data can be sent to processing
        #Code modified from: https://realpython.com/python-sockets/#running-the-echo-client-and-server
          
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_address = ('localhost', 10000)
        sock.bind(server_address)
        print('waiting for connection')
        sock.listen(1)
        self.connection, client_address = sock.accept()
        c = False
        while c:
            try:
                print('connection: ', client_address)
                c = True
            except:pass
    
    def draw_color_frame(self, frame, target_surface):
        target_surface.lock()
        address = self._kinect.surface_as_array(target_surface.get_buffer())
        ctypes.memmove(address, frame.ctypes.data, frame.size)
        del address
        target_surface.unlock()


    def run(self):
        display = (800,800)
        pygame.display.set_mode(display, DOUBLEBUF|OPENGL)
        #setting up scene to be displayed
        gluPerspective(45, (display[0]/display[1]), 0.1, 50.0)
        glTranslatef(0.0,0.0,-35)
        glRotatef(90, 0, 1, 0)
        glRotatef(-90,0,0,1)
        while not self._done:
            for event in pygame.event.get(): # User did something
                if event.type == pygame.QUIT: # If user clicked close
                    self._done = True # Flag that we are done so we exit this loop                    
            if self._kinect.has_new_color_frame():
                frame = self._kinect.get_last_color_frame()
                self.draw_color_frame(frame, self._frame_surface)
                frame = None
            if self._kinect.has_new_body_frame(): 
                self._bodies = self._kinect.get_last_body_frame()
            if self._bodies is not None: 
                for i in range(0, self._kinect.max_body_count):
                    body = self._bodies.bodies[i]
                    if not body.is_tracked: 
                        continue 
                    joints = body.joints 
                    #doing logic for inverse kinematics
                    sR,eR,hR = getCoordinatesRight(joints)
                    vR = getVectorRight(sR,eR,hR)
                    self.angRight,self.breakRight = inverseKinematicsRight(vR,self.angRight)
                    sL,eL,hL = getCoordinatesLeft(joints)
                    vL = getVectorLeft(sL,eL,hL)
                    self.angLeft,self.breakLeft = inverseKinematicsLeft(vL,self.angLeft)
            #displaying virtual robot on pyopengl
            glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
            angles = formatAngles(self.angRight,self.angLeft)
            #sending over data
            self.connection.sendall(struct.pack('BBBBBBBB',angles[0],angles[1],angles[2],angles[3],\
                angles[4],angles[5],angles[6],angles[7]))
            drawRobotRight(self.angRight[0],self.angRight[1],self.angRight[2],self.angRight[3])
            drawRobotLeft(self.angLeft[0],self.angLeft[1],self.angLeft[2],self.angLeft[3])
            drawText((-5,0,-8),"Hand Following Robot!")
            #if calculations break, reset angles
            if self.breakRight:
                self.angRight = [0,90,0,0]
            if self.breakLeft:
                self.angLeft = [0,90,0,0]
            pygame.display.flip()

            self._clock.tick(20)

        self._kinect.close()
        self.connection.close()
        pygame.quit()


__main__ = "Kinect v2 Body Game"
game = BodyGameRuntime();
game.run();





