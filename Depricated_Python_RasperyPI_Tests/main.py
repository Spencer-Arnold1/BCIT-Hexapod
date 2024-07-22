# BCIT-HEXAPOD ROBOTICS CLUB 
# Author(s): Spencer Arnold, Hassan Islam 
# Date : 2024-04-24
# DESC : Hexapod Main 
# NOTE: measurements --> COXA = 8.4, FEMUR = 29.3, TIBIA = 24.8


#NOTE I COMMENTED OUT THE SERVO CODE FOR NOW!

from Leg import Leg
import numpy as np
import time
import display as dp

leg = Leg("Leg1", COXA=0, FEMUR=1, TIBIA=1)

leg.resetPosition()

leg.setPosition([0,   1.707,   0.707], 0) # all the way right


'''
time.sleep(3)

leg.setPosition([0.1,   0.1,   62.5], 0) # all the way right

leg.setPosition([0.1,   0.1,  -62.5], 0) # all the way left

time.sleep(0.5)

leg.setPosition([26.5874,   44.0226,  26.5874], 0) # (45,45,25)

time.sleep(0.5)

for i in range(4): 

    leg.setPosition([26.5874,   44.0226,  26.5874], 0) # (45,45,25)
    leg.setPosition([38.1259,   20.7182,  38.1259], 0) # (45,45,-45)

time.sleep(0.5)

leg.setPosition([8.1897, 38.2545, -8.1897], 0) # (-45,45, 90)

time.sleep(0.5)

for i in range(4): 

    leg.setPosition([8.1897, 38.2545, -8.1897], 0) # (45,45,90)
    leg.setPosition([20.5897, 45.5182, -20.5897], 0) # (-45,45, 45)

leg.setPosition([8.1897, 38.2545, -8.1897], 20) # (-45,45, 90)

time.sleep(1)

leg.setPosition([0.1,   0.1,   62.5], 20) # all the way right

time.sleep(1)

leg.setPosition([8.1897, 38.2545, -8.1897], 20) # (45,45,90)



'''

'''
def path(pos1,pos2,num,duration):
    start = np.array(pos1)
    end = np.array(pos2)

    for p in range(num):
        l = p/(num-1)
        point = (1-l)*start+l*end
        leg.setPosition(point,1)
        time.sleep(duration/num)

#coords [up,
leg = Leg("Leg1", COXA=1, FEMUR=1, TIBIA=1)
firstPos =  [2,2,2]
secondPos = [3,2,2]
while True:

    #leg.setPosition([3,0,0])
    path(firstPos,secondPos,5,1)
    time.sleep(3)

    #leg.setPosition([1,1,0])
    path(secondPos,firstPos,5,1)
    time.sleep(3)
'''


