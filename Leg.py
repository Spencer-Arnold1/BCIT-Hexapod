# BCIT-HEXAPOD ROBOTICS CLUB 
# Author(s): Spencer Arnold, Hassan Islam 
# Date : 2024-04-24
# DESC : A 3-joint 'RRR' joint Robotic arm. Each joint is a 180 degree Servo. 

import Coordinate2JointAngle as c2a
import numpy as np
from adafruit_servokit import ServoKit
import time

class Leg:
    def __init__(self, name, COXA, FEMUR, TIBIA):

        self.linkLengths = np.array([TIBIA, FEMUR, COXA], dtype=float)

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
    

    def setPosition(self, CartesianPosition, steps):
        # Return current angular position of arm -------- DOES NOT ANYMORE 
        currentAngles = np.array([0.1, 0.1, 0.1])  #self.__getAngles()

        # Convert list to np.array
        nextPosition = np.array([CartesianPosition[0], CartesianPosition[1], CartesianPosition[2]])

        # Method to convert nextPosition to new angular position
        position, iteration = c2a.Coordinate2JointAngle(currentAngles, self.linkLengths, nextPosition, False)

        print("\n Before adjusting {theta, gamma, phi} =", "[{:.5f}, {:.5f}, {:.5f}]".format(*position))

        # Interpolate between old and new angles
        interpolated_angles = np.zeros((steps + 1, 3))
        if(steps == 0):
            self.theta1 = position[0]
            self.theta2 = position[1]
            self.theta3 = position[2]
            self.__setServoAngles()  # Update servo angles for each step

            return position

        else:
            for i in range(3):
                interpolated_angles[:, i] = np.linspace(currentAngles[i], position[i], steps + 1)

            # Update self.theta1, self.theta2, self.theta3 with interpolated values
            for i in range(steps + 1):
                self.theta1 = interpolated_angles[i, 0]
                self.theta2 = interpolated_angles[i, 1]
                self.theta3 = interpolated_angles[i, 2]
                self.__setServoAngles()  # Update servo angles for each step

            return interpolated_angles


    def resetPosition(self):
        self.theta1 = 0.1
        self.theta2 = 0.1
        self.theta3 = 0.1
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
                positionAngles[i] = 90
            elif(positionAngles[i] <= -90):
                positionAngles[i] = -90

        # print angles 
        print("\n{theta, gamma, phi} =", "[{:.5f}, {:.5f}, {:.5f}]".format(*positionAngles))

        # maybe add a wait here....
        time.sleep(0.3)

        
        # an offset is added as servos work between 0 to 180 degrees
        self.__kit.servo[0].angle = -positionAngles[0] + 90
        self.__kit.servo[1].angle = positionAngles[1] + 90
        self.__kit.servo[2].angle = positionAngles[2] + 90
        