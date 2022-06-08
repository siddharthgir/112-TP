# 112-TP

This is the code for a robot that plays the game "hole in a wall", inspired from a game show of the same name that is normally done with humans. The main feature is the ability of the robots arms to copy the position of a humans arms. It requires a humanoid robot controlled with an arduino mounted on a roomba robot as the base to make it move.

To run the project, the user needs to run the files "inverseKinematics.py" and "roomba.py" together in visual studio running 32 bit python. Then the user needs to start the file "robot.pde" in processing to communicate with the robot. The arduino needs to have the standardFirmata code from the arduino examples directory loaded onto to it. The roomba file will also open a GUI to communicate with the robot once it has established connection with the roomba. Make sure the code for "roomba.py" and "robot.pde" have the appropiate com port in the code. After this, turn the power on for the robot humanoid and it should be able to move.

Things needed

Hardware needed:
1. Humanoid robot arms (Controlled with arduino)
2. Roomba robot
3. Kinect Sensor

General model for program is:
All logic for inverse kinematics, visualization by pyopengl and the kinect is done in the inverseKinematics.py file, which sends data over to the robot.pde file through a socket, which then transmits the data to the arduino using the firmata library, which allows for very stable safe communication. These two programs allow for control of the arm.

The roomba code creates the gui for interation and moves the roomba to move the robot forward and back.

Things to install:

Visual Studio: https://visualstudio.microsoft.com

Processing:https://processing.org/

Drivers for hardware:
Arduino drive(https://www.arduino.cc/en/Guide/DriverInstallation)
Kinect SDK v2 (https://www.microsoft.com/en-us/download/details.aspx?id=44561)

List of libraries for visual studio/python:
pykinect2, pyserial, socket, numpy, pyopengl

All can be installed using pip install or conda install on a 32-bit system.

List of libraries for processing:
processing.net
processing.serial
Arduino(Firmata)

Can be installed using built in library install in sketch -> install library

Install standardFirmata on arduino, can be found in sketch IDE by file -> examples -> Firmata -> Standard Firmata





