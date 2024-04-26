# BCIT-HEXAPOD ROBOTICS CLUB 
# Author(s): Spencer Arnold, Hassan Islam 
# Date : 2024-04-24

from adafruit_servokit import ServoKit
from Leg import Leg
import time
import math
import numpy as np

#leg = Leg("Leg1", COXA=8.4, FEMUR=29.3, TIBIA=24.8)

# all link lengths are set as 1, not so important for it to be too accurate 
leg = Leg("Leg1", COXA=1, FEMUR=1, TIBIA=1)

kit = ServoKit(channels=16)
# delay between servo movements 
delay = 0.5

min_pulse = 500  # Minimum pulse width
max_pulse = 2500  # Maximum pulse width
z_offset = 100

for i in range(4):
    kit.servo[i].set_pulse_width_range(min_pulse, max_pulse)

# all positions of leg 
WalkCyclePositions = [np.array([((3*np.sqrt(2))/2),0,((3*np.sqrt(2))/2)]), np.array([3,0,0]), np.array([((3*np.sqrt(2))/2),0,((-3*np.sqrt(2))/2)]), np.array([3,0,0])]

# gets position of legs on initialization
positionAngles = leg.getAngles()

while True:

    for targetPosition in WalkCyclePositions:

        # set new anlges of leg based on target position
        leg.Coordinate2JointAngles(positionAngles,targetPosition, False) 

        # recieve new angle
        positionAngles = leg.getAngles()

        # keeps position within -90 to 90 degrees
        for i in range(len(positionAngles)):
            if(positionAngles[i]>= 90):
                positionAngles[i] = 89
            elif(positionAngles[i] <= -90):
                positionAngles[i] = -89
        print(positionAngles[1])

        # an offset is added as servos work between 0 to 180 degrees
        kit.servo[0].angle = positionAngles[1] + 90
        kit.servo[1].angle = positionAngles[0] + 90
        kit.servo[2].angle = positionAngles[2] + 90

        # delay to wait before next movement of leg 
        time.sleep(delay)


    '''
    for angles in iteration:
        for i in range(len(angles)):
            if(angles[i] >= 90):
                angles[i] = 89
            elif(angles[i] <= -90):
                angles[i] = -89
        print(angles)
        kit.servo[0].angle = angles[1] + 90
        kit.servo[1].angle = angles[0] + 90
        kit.servo[2].angle = angles[2] + 90
        time.sleep(delay)
    '''
