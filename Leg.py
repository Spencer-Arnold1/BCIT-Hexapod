# BCIT-HEXAPOD ROBOTICS CLUB 
# Author(s): Spencer Arnold, Hassan Islam 
# Date : 2024-04-24
# DESC : A 3-joint 'RRR' joint Robotic arm. Each joint is a 180 degree Servo. 

import Coordinate2JointAngle as c2a
import numpy as np
from adafruit_servokit import ServoKit

class Leg:
    def __init__(self, name, COXA, FEMUR, TIBIA):

        self.linkLengths = np.array([COXA, FEMUR, TIBIA], dtype=float)

        # the following cannot be 0 or else it causes a jacobian singularity error 
        self.theta1 = 0.1
        self.theta2 = 0.1
        self.theta3 = 0.1
        
        # private attributes
        self.__kit = ServoKit(channels=16) 
        self.__min_pulse = 500  # Minimum pulse width
        self.__max_pulse = 2500  # Maximum pulse width

        for i in range(4):
            self.__kit.servo[i].set_pulse_width_range(self.__min_pulse, self.__max_pulse)
    
    
    def setPosition(self, CartesianPosition):

        # return current angular position of arm 
        currentPosition = self.__getAngles()

        # convert list to np.array 
        nextPosition = np.array([CartesianPosition[0],CartesianPosition[1],CartesianPosition[2]])

        # method to convert nextPosition to new angluar position 
        position, iteration = c2a.Coordinate2JointAngle(currentPosition, self.linkLengths, nextPosition, False)

        # new angles replace old angles 
        self.theta1 = position[0]; self.theta2 = position[1]; self.theta3 = position[2]

        self.__setServoAngles()
    
    
    # Note the method is not called ServoAngles. These angles are converted to work with the servo angles within __SetServoAngles
    def __getAngles(self):

        return np.array([self.theta1, self.theta2, self.theta3]) 
    
    def __setServoAngles(self):

        # recieve new angle
        positionAngles = self.__getAngles()

        # keeps position within -90 to 90 degrees
        for i in range(len(positionAngles)):
            if(positionAngles[i]>= 90):
                positionAngles[i] = 89
            elif(positionAngles[i] <= -90):
                positionAngles[i] = -89

        # To see angle of first servo 
        print("\n{theta, gamma, phi } =",format(positionAngles), "\n")

        # an offset is added as servos work between 0 to 180 degrees
        self.__kit.servo[0].angle = positionAngles[0] + 90
        self.__kit.servo[1].angle = positionAngles[1] + 90
        self.__kit.servo[2].angle = positionAngles[2] + 90
