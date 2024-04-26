# BCIT-HEXAPOD ROBOTICS CLUB 
# Author(s): Spencer Arnold, Hassan Islam 
# Date : 2024-04-24

import Coordinate2JointAngle 
import numpy as np

class Leg:
    def __init__(self, name, COXA, FEMUR, TIBIA):
        self.name = name
        self.COXA = COXA
        self.FEMUR = FEMUR
        self.TIBIA = TIBIA
        # the following cannot be 0 or else it causes a jacobian singularity error 
        self.theta1 = 0.1 
        self.theta2 = 0.1
        self.theta3 = 0.1

    def Coordinate2JointAngles(self, initialAngluarposition, targetCartesionPosition, returnIterations):

        linkLengths = np.array([self.COXA, self.FEMUR, self.TIBIA])

        position, iteration = Coordinate2JointAngle.Coordinate2JointAngle(initialAngluarposition, linkLengths, targetCartesionPosition, returnIterations)

        self.theta2 = position[0]; self.theta1 = position[1]; self.theta3 = position[2]

        return iteration


    def getAngles(self):

        return np.array([self.theta2, self.theta1, self.theta3]) # This order is weird, ask me why! (Hassan)
    
    