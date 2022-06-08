
from tkinter import *
#import tkMessageBox
#import tkSimpleDialog
import math, numpy
import time
import threading
import socket
import select
import struct
import random
import sys, glob # for listing serial ports
import os  # to command the mp3 and wav player omxplayer

from tkinter import *

'''This file has 2 main parts, mainly the implementation of the GUI and the 
control of the roomba'''

#implementing GUI here

def init(data):
    data.width = 600
    data.height = 600
    data.state = "start"
    data.gameState = "start"
    data.countdown = 5
    data.moveTime = 3
    data.timer = 0
    

def mousePressed(event, data):
    if data.state == "start":
        if data.width/2-125 <= event.x <= data.width/2+125 and 150 <= event.y <= 225:
            data.state = "play"
        elif data.width/2-125 <= event.x <= data.width/2+125 and 275 <= event.y <= 350:
            data.state = "instructions"
        elif data.width/2-125 <= event.x <= data.width/2+125 and 400 <= event.y <= 475:
            data.state = "about"
    if data.state == "about" or "instructions" or "play":
        if 30 <= event.x <= 130 and data.height - 60 <= event.y <= data.height - 30:
            data.state = "start"
    if data.state == "play" and data.gameState == "start":
        if data.width/2-125 <= event.x <= data.width/2+125 and 200 <= event.y <= 275:
            data.gameState = "waiting"
    if data.state == "play" and data.gameState == "end":
        if data.width/2-125 <= event.x <= data.width/2+125 and 275 <= event.y <= 350:
            data.gameState = "start"
        elif data.width/2-125 <= event.x <= data.width/2+125 and 400 <= event.y <= 475:
            data.gameState = "reset"
    
            

def keyPressed(event, data):
    pass

def timerFired(data):
    if data.gameState == "waiting":
        data.timer += 1
        if data.timer%10 == 0: data.countdown -= 1
        if data.countdown <= 0:
            data.gameState = "approaching"
            data.countdown = 5
            data.timer = 0
    elif data.gameState == "approaching":
        data.timer += 1
        if data.timer%10 == 0: data.moveTime -= 1
        if data.moveTime <= 0:
            data.gameState = "end"
            data.moveTime = 3
            data.timer = 0
    elif data.gameState == "reset":
        data.timer += 1
        if data.timer%10 == 0: data.moveTime -= 1
        if data.moveTime <= 0:
            data.gameState = "start"
            data.moveTime = 3
            data.timer = 0
    pass

def drawStart(data,canvas):
    canvas.create_rectangle(0,0,data.width,data.height,fill = "deep sky blue")
    canvas.create_text(data.width/2,100,text = "Robot Hole In A Wall!", font = "Arial 30 bold", fill = "snow")
    canvas.create_rectangle(data.width/2-125,150,data.width/2+125,225,fill = "snow")
    canvas.create_text(data.width/2,187,text = "Play", font = "Arial 30 bold", fill = "black")
    canvas.create_rectangle(data.width/2-125,275,data.width/2+125,350,fill = "snow")
    canvas.create_text(data.width/2,312,text = "Instructions", font = "Arial 25 bold", fill = "black")
    canvas.create_rectangle(data.width/2-125,400,data.width/2+125,475,fill = "snow")
    canvas.create_text(data.width/2,437,text = "About Project", font = "Arial 22 bold", fill = "black")
    
def drawInstructions(data,canvas):
    canvas.create_rectangle(0,0,data.width,data.height,fill = "deep sky blue")
    canvas.create_rectangle(30,data.height - 30,130,data.height - 60,fill = "snow")
    canvas.create_text(80,data.height-45,text = "Back", font = "Arial 13 bold", fill = "black" )
    canvas.create_text(data.width/2,100,text = "Instructions", font = "Arial 30 bold", fill = "snow")
    canvas.create_text(data.width/2,250,text = "1. Move your hands to control", font = "Arial 20 bold", fill = "snow")
    canvas.create_text(data.width/2,280,text = "the robots hands", font = "Arial 20 bold", fill = "snow")
    canvas.create_text(data.width/2,320,text = "2. Try to dodge the obstacles", font = "Arial 20 bold", fill = "snow")
    canvas.create_text(data.width/2,350,text = "before you hit them", font = "Arial 20 bold", fill = "snow")
    canvas.create_text(data.width/2,390,text = "3. Good Luck!!", font = "Arial 20 bold", fill = "snow")
    

    
def drawAbout(data,canvas):
    canvas.create_rectangle(0,0,data.width,data.height,fill = "deep sky blue")
    canvas.create_rectangle(30,data.height - 30,130,data.height - 60,fill = "snow")
    canvas.create_text(80,data.height-45,text = "Back", font = "Arial 13 bold", fill = "black" )
    canvas.create_text(data.width/2,100,text = "About Project", font = "Arial 30 bold", fill = "snow")
    canvas.create_text(data.width/2,150,text = "1. The data for the human's position is taken ", font = "Arial 15 bold", fill = "snow")
    canvas.create_text(data.width/2,180,text = "using a kinect sensor", font = "Arial 15 bold", fill = "snow")
    canvas.create_text(data.width/2,250,text = "2. The motor angles for the robot are calculated", font = "Arial 15 bold", fill = "snow")
    canvas.create_text(data.width/2,280,text = "using an inverse kinematics algorithm", font = "Arial 15 bold", fill = "snow")
    canvas.create_text(data.width/2,350,text = "3. The robot is controlled using an arduino", font = "Arial 15 bold", fill = "snow")
    canvas.create_text(data.width/2,380,text = "and a roomba", font = "Arial 15 bold", fill = "snow")
    canvas.create_text(data.width/2,450,text = "4. Shoutout to my mentor Fletcher and Antonio", font = "Arial 15 bold", fill = "snow")
    canvas.create_text(data.width/2,480,text = "from the robotics club for making this possible! ", font = "Arial 15 bold", fill = "snow")
    

    
def drawPlay(data,canvas):
    canvas.create_rectangle(0,0,data.width,data.height,fill = "deep sky blue")
    canvas.create_rectangle(30,data.height - 30,130,data.height - 60,fill = "snow")
    canvas.create_text(80,data.height-45,text = "Back", font = "Arial 13 bold", fill = "black" )
    canvas.create_text(data.width/2,100,text = "Play", font = "Arial 45 bold", fill = "snow")
    if data.gameState == "start":
        driveDirect(0,0)
        canvas.create_rectangle(data.width/2-125,200,data.width/2+125,275,fill = "snow")
        canvas.create_text(data.width/2,237,text = "Start Game", font = "Arial 25 bold", fill = "black")
    elif data.gameState == "waiting":
        canvas.create_text(data.width/2,237,text = "Get Ready!, Game starting in", font = "Arial 25 bold", fill = "snow")
        canvas.create_text(data.width/2,287,text = str(data.countdown), font = "Arial 25 bold", fill = "snow")
    elif data.gameState == "approaching":
        driveDirect(30,30)
        canvas.create_text(data.width/2,237,text = "Wall Approaching!", font = "Arial 25 bold", fill = "snow")
    elif data.gameState == "end":
        driveDirect(0,0)
        canvas.create_text(data.width/2,237,text = "Game Over!", font = "Arial 25 bold", fill = "snow")
        canvas.create_rectangle(data.width/2-125,275,data.width/2+125,350,fill = "snow")
        canvas.create_text(data.width/2,312.5,text = "Play Again", font = "Arial 25 bold", fill = "black")
        canvas.create_rectangle(data.width/2-125,400,data.width/2+125,475,fill = "snow")
        canvas.create_text(data.width/2,437,text = "Reset", font = "Arial 25 bold", fill = "black")
    elif data.gameState == "reset":
        driveDirect(-30,-30)
        canvas.create_text(data.width/2,237,text = "Robot Resetting!", font = "Arial 25 bold", fill = "snow")
    
def redrawAll(canvas, data):
    if data.state == "start":
        driveDirect(0,0)
        drawStart(data,canvas)
    elif data.state == "instructions":
        drawInstructions(data,canvas)
    elif data.state == "about":
        drawAbout(data,canvas)
    elif data.state == "play":
        drawPlay(data,canvas)


#Run function taken from 112 website

####################################
# use the run function as-is
####################################

def run(width=300, height=300):
    def redrawAllWrapper(canvas, data):
        canvas.delete(ALL)
        canvas.create_rectangle(0, 0, data.width, data.height,
                                fill='white', width=0)
        redrawAll(canvas, data)
        canvas.update()    

    def mousePressedWrapper(event, canvas, data):
        mousePressed(event, data)
        redrawAllWrapper(canvas, data)

    def keyPressedWrapper(event, canvas, data):
        keyPressed(event, data)
        redrawAllWrapper(canvas, data)

    def timerFiredWrapper(canvas, data):
        timerFired(data)
        redrawAllWrapper(canvas, data)
        # pause, then call timerFired again
        canvas.after(data.timerDelay, timerFiredWrapper, canvas, data)
    # Set up data and call init
    class Struct(object): pass
    data = Struct()
    data.width = width
    data.height = height
    data.timerDelay = 100 # milliseconds
    root = Tk()
    root.resizable(width=False, height=False) # prevents resizing window
    init(data)
    # create the root and the canvas
    canvas = Canvas(root, width=data.width, height=data.height)
    canvas.configure(bd=0, highlightthickness=0)
    canvas.pack()
    # set up events
    root.bind("<Button-1>", lambda event:
                            mousePressedWrapper(event, canvas, data))
    root.bind("<Key>", lambda event:
                            keyPressedWrapper(event, canvas, data))
    timerFiredWrapper(canvas, data)
    # and launch the app
    root.mainloop()  # blocks until window is closed
    print("bye!")


#Majority code here is implementing the logic for controlling the roomba, all taken from here:https://github.com/Tall67Paul/PythonRobot
#My code is written after comment line "my code starts from here".
try:
    import serial
except ImportError:
    print("Import error.  Please install pyserial.")
    raise

connection = None
global FAILURE
FAILURE = False

def toTwosComplement2Bytes( value ):
        """ returns two bytes (ints) in high, low order
        whose bits form the input value when interpreted in
        two's complement
        """
        # if positive or zero, it's OK
        if value >= 0:
            eqBitVal = value
            # if it's negative, I think it is this
        else:
            eqBitVal = (1<<16) + value
    
        return ( (eqBitVal >> 8) & 0xFF, eqBitVal & 0xFF )

# sendCommandASCII takes a string of whitespace-separated, ASCII-encoded base 10 values to send
def sendCommandASCII(command):
    cmd = ""
    for v in command.split():
        cmd += chr(int(v))
    sendCommandRaw(cmd)

# sendCommandRaw takes a string interpreted as a byte array
def sendCommandRaw(command):
    global connection
    try:
        if connection is not None:
            connection.write(command.encode())
        else:
            print("Not connected.")
    except serial.SerialException:
        print("Lost connection")
        connection = None
    #print ' '.join([ str(ord(c)) for c in command ])

# sendCommandRaw takes a string interpreted as a byte array
def sendCommandRawBytes(command):
    global connection
    try:
        if connection is not None:
            connection.write(command)
        else:
            print("Not connected.")
    except serial.SerialException:
        print("Lost connection")
        connection = None
    #print ' '.join([ str(ord(c)) for c in command ])

# getDecodedBytes returns a n-byte value decoded using a format string.
# Whether it blocks is based on how the connection was set up.
def getDecodedBytes( n, fmt):
    global connection
        
    try:
        return struct.unpack(fmt, connection.read(n))[0]
    except serial.SerialException:
        print("Lost connection")
        tkMessageBox.showinfo('Uh-oh', "Lost connection to the robot!")
        connection = None
        return None
    except struct.error:
        print("Got unexpected data from serial port.")
        return None

def bytesOfR( r ):
        """ for looking at the raw bytes of a sensor reply, r """
        print('raw r is', r)
        for i in range(len(r)):
            print('byte', i, 'is', ord(r[i]))
        print('finished with formatR')

def toBinary( val, numBits ):
        """ prints numBits digits of val in binary """
        if numBits == 0:  return
        toBinary( val>>1 , numBits-1 )
#        print((val & 0x01), end=' ')  # print least significant bit

def bitOfByte( bit, byte ):
    """ returns a 0 or 1: the value of the 'bit' of 'byte' """
    if bit < 0 or bit > 7:
        print('Your bit of', bit, 'is out of range (0-7)')
        print('returning 0')
        return 0
    return ((byte >> bit) & 0x01)

# get8Unsigned returns an 8-bit unsigned value.
def get8Unsigned():
    return getDecodedBytes(1, "B")

# get lowest bit from an unsigned byte
def getLowestBit():
    wheelsAndBumpsByte = getDecodedBytes(1, "B")
    print(wheelsAndBumpsByte)
    return bitOfByte(0, wheelsAndBumpsByte)

# get second lowest bit from an unsigned byte
def getSecondLowestBit():
    wheelsAndBumpsByte = getDecodedBytes(1, "B")
    print(wheelsAndBumpsByte)
    return bitOfByte(1, wheelsAndBumpsByte)

def bumped():
    sendCommandASCII('142 7') 
    time.sleep( 0.02 )
    bumpedByte = getDecodedBytes( 1, "B" )
    if bumpedByte == 0:
        return False
    elif bumpedByte > 3:
        print("CRAZY BUMPER SIGNAL!")
    else:
        return True

def cleanButtonPressed():
    sendCommandASCII('142 18') 
    buttonByte = getDecodedBytes( 1, "B" )
    if buttonByte == 0:
        return False
    elif buttonByte == 1:
        print("Clean Button Pressed!")
        return True
    elif buttonByte == 4:
        return False
    else:
        print("Some other button pressed!")
        FAILURE = True
        return False

def dockButtonPressed():
    sendCommandASCII('142 18') 
    buttonByte = getDecodedBytes( 1, "B" )
    if buttonByte != 4:
        return False
    else:
        print("Dock button pressed!")
        return True

def shudder( period, magnitude, numberOfShudders):
    i = 0
    timestep = 0.02
    while i < numberOfShudders:
        i = i + 1
        #shake left
        t = 0
        while t < period:
            driveDirectRot( 0, magnitude )
            t = t + timestep
            time.sleep( timestep )
    #Shake right
    t = 0
    while t < period:
        driveDirectRot( 0, -magnitude )
        t = t + timestep
        time.sleep( timestep )
    driveDirect( 0, 0 )  # stop the previous motion command

def onConnect():
    global connection

    if connection is not None:
        print("Oops- You're already connected!")
        return

    try:
        ports = getSerialPorts()
        print("Available ports:\n" + '   '.join(ports))
        #port = raw_input("Port? Enter COM port to open.\nAvailable options:\n" + '\n'.join(ports))
        port = str( ports[0] )  # I'm guessing that the Roomba port is first in the list.  So far this works!  :)
    except EnvironmentError:
        port = raw_input("Port?  Enter COM port to open.")

    if port is not None:
        print("Trying " + "com13" + "... ")
    try:   #:tty
        #connection = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)
        connection = serial.Serial("com13", baudrate=115200, timeout=1 )
        #connection = serial.Serial( str(ports[2]), baudrate=115200, timeout=1 )
        print("Connected!")
    except:
        print("Failed.  Could not connect to " + str( port ))

def getSerialPorts():
    """Lists serial ports
    From http://stackoverflow.com/questions/12090503/listing-available-com-ports-with-python

    :raises EnvironmentError:
        On unsupported or unknown platforms
    :returns:
        A list of available serial ports
    """
    if sys.platform.startswith('win'):
        ports = ['COM' + str(i + 1) for i in range(256)]

    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this is to exclude your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')

    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')

    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result    

def driveDirectTime( left, right, duration ):
    print("driveDirectTime()")
    t = 0   # initialize timer
    while t < duration:
        driveDirect( left, right )
        time.sleep( 0.05 )
        t = t + .05
    driveDirect( 0, 0 )  # stop

def driveDirect( leftCmSec = 0, rightCmSec = 0 ):
    """ sends velocities of each wheel independently
           left_cm_sec:  left  wheel velocity in cm/sec (capped at +- 50)
           right_cm_sec: right wheel velocity in cm/sec (capped at +- 50)
    """
    print("driveDirect()")
    if leftCmSec < -50: leftCmSec = -50
    if leftCmSec > 50:  leftCmSec = 50
    if rightCmSec < -50: rightCmSec = -50
    if rightCmSec > 50: rightCmSec = 50
    # convert to mm/sec, ensure we have integers
    leftHighVal, leftLowVal = toTwosComplement2Bytes( int( leftCmSec * 10 ) )
    rightHighVal, rightLowVal = toTwosComplement2Bytes( int( rightCmSec * 10 ) )

    # send these bytes and set the stored velocities
    byteListRight = ( rightHighVal , rightLowVal )
    byteListLeft = ( leftHighVal , leftLowVal )
    sendCommandRawBytes(struct.pack( ">Bhh", 145, int(rightCmSec * 10), int(leftCmSec * 10) ))
    return

def driveDirectRot( robotCmSec = 0, rotation = 0 ):
    """ implements the driveDirect with a given rotation
        Positive rotation turns the robot CCW
        Negative rotation turns the robot CW
    """
    print("driveDirectRot()")
    vl = robotCmSec - rotation/2
    vr = robotCmSec + rotation/2
    driveDirect ( vl, vr )

def initiateRobotCommunication():
    print("Initiating Communications to the Create 2 Robot...")
    onConnect()
    time.sleep( 0.3 )
    sendCommandASCII('128')   # Start Open Interface in Passive
    time.sleep( 0.3 )
    sendCommandASCII('140 3 1 64 16 141 3')  # Beep
    time.sleep( 0.3 )
    #sendCommandASCII('131')   # Safe mode
    sendCommandASCII( '132' )   # Full mode 
    time.sleep( 0.3 )
    sendCommandASCII('140 3 1 64 16 141 3')  # Beep
    time.sleep( 0.1 )
    sendCommandASCII('139 4 0 255')  # Turn on Clean and Dock buttons
    time.sleep( 0.03 )

def closeRobotCommunication():
    print("Closing Communication to the Create 2 Robot...")
    driveDirect( 0, 0 )  # stop robot if moving
    time.sleep( 0.05 )
    sendCommandASCII('140 3 1 64 16 141 3')  # Beep
    time.sleep( 0.3 )
    #sendCommandASCII('139 0 0 0')  # Turn off Clean and Dock buttons
    time.sleep( 0.03 )
    sendCommandASCII('138 0')  # turn off vacuum, etractors, and side brush
    time.sleep( 0.03 )
    #sendCommandASCII( '7' )  # Resets the robot    
    sendCommandASCII( '173' )  # Stops the Open Interface to Roomba
    time.sleep( 0.3 )
    connection.close()
    time.sleep( 0.1 )
    raise SystemExit    #  Exit program


##########################################################################
# my code starts from here #
##########################################################################
print("Hellooooooooooooo, world!")
print("Starting my Python Robot Controller Program.  This is so cool!")


#initiateRobotCommunication()
run(600,600)
#while True:
#    i = input("Enter 1 to move")
#    if i == '1':
#        driveDirect(30,30)
#        time.sleep(2)
#        driveDirect(0,0)
#    if i == '0':
#        break

#Close the robot communication interface
closeRobotCommunication()



