#!/usr/bin/env pybricks-micropython

"""
Example LEGO® MINDSTORMS® EV3 Robot Educator Color Sensor Down Program
----------------------------------------------------------------------

This program requires LEGO® EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

Building instructions can be found at:
https://education.lego.com/en-us/support/mindstorms-ev3/building-instructions#robot
"""

from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase

# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.D)

# Initialize the color sensor.
line_sensor = ColorSensor(Port.S1)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)
motor = Motor(port=Port.A)

# Calculate the light threshold. Choose values based on your measurements.
BLACK = 4
WHITE = 65
threshold = (BLACK + WHITE) / 2


# Set the drive speed at 100 millimeters per second.
DRIVE_SPEED = 35

# Set the gain of the proportional line controller. This means that for every
# percentage point of light deviating from the threshold, we set the turn
# rate of the drivebase to 1.2 degrees per second.

# For example, if the light value deviates from the threshold by 10, the robot
# steers at 10*1.2 = 12 degrees per second.
Kp = 2 #1 +- 0.5
Ki = 0 #0.5 +- 0.05
kd = 2 #1 +- 0.5
integral = 0
error = 0
last_error = 0
ps = +1


def prise_objet_():
    speed = 500 #deg/s
    
    montée = +1 #pour serrer et monter
    #montée = -1 #descendre et ouvrir et 
    
    #fixer les angles fixes 
    AngleRotationMin = 0
    AngleRotationMax = 0
    AngleRotationToTake = 0
    AngleRotationToLet = 0
    
    #identifier the current one 
    CurentAngle = take_motor.angle()
    print("le moteur est à ", take_motor.angle())
    print(1)
    
    #Tourner le moteur vers la position desiree
    if CurentAngle < AngleRotationToTake: 
        take_motor.run_angle(speed, AngleRotationToTake, then=Stop.HOLD, wait=True)
        #take_motor.run_time(speed, time, then=Stop.HOLD, wait=True)
        print("le moteur est à ", take_motor.angle())
        wait(10)
    else:
        take_motor.run_angle(-speed, AngleRotationToTake, then=Stop.HOLD, wait=True)
        print("le moteur est à ", take_motor.angle())
        wait(10)
    print(2)
        
    # for i in range(200):
        # rotation_angle = 2000*montée #deg
        # take_motor.run_angle(speed, rotation_angle, then=Stop.HOLD, wait=True)
        # print(i)
        # print("le moteur est à ", take_motor.angle())
        # wait(100)
    #time = 6000
    # print("dans 1 s !!!!!!!!!")
    # wait(200)
    #take_motor.run_angle(speed, rotation_angle, then=Stop.HOLD, wait=True)
    #take_motor.run_time(-speed, time, then=Stop.HOLD, wait=True)
    #take_motor.run_time(-speed, time, then=Stop.HOLD, wait=True)
    
def prise_objet():
    speed = 500 #deg/s 
    rotation_angle = 2000
    take_motor.run_angle(speed, rotation_angle, then=Stop.HOLD, wait=True)


def main():
    prise_objet()

print(1)

main()

