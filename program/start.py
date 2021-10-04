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



def start1():
    while(1):
        DRIVE_SPEED = 50
        turn_rate=0
        robot.drive(DRIVE_SPEED, turn_rate)
        detect_obstacle()
        #apres une certaine distance, il tourne à gauche
def start():
        DRIVE_SPEED = 150
        turn_rate_1=0 
        turn_rate_2=100 
        
        robot.drive(DRIVE_SPEED, turn_rate_1)
        #robot.run_time(speed, time, then=Stop.HOLD, wait=True) 
        robot.drive_time(speed, 0,time) #mm/sec; steeringdegrees/sec for timemilliseconds
        robot.drive(DRIVE_SPEED, turn_rate_2) #mm/sec;  steeringDeg/sec 
        detect_obstacle() 
        #apres une certaine distance, il tourne à gauche 


def detect_obstacle():
    while obstacle_sensor.distance() < 150:
        DRIVE_SPEED=-300
        turn_rate = 80
        robot.drive(DRIVE_SPEED, turn_rate)
    wait(10) 



def main():
    start()


print(1)

main()

