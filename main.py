# BCIT-HEXAPOD ROBOTICS CLUB 
# Author(s): Spencer Arnold, Hassan Islam 
# Date : 2024-04-24
# DESC : Hexapod Main 
# NOTE: measurements --> COXA = 8.4, FEMUR = 29.3, TIBIA = 24.8

from Leg import Leg
import time

leg = Leg("Leg1", COXA=1, FEMUR=1, TIBIA=1)

while True:

    leg.setPosition([1.3077,1.7071,1.0973])

    time.sleep(3)

    leg.setPosition([2.2807, 1.5263, 0])

    time.sleep(3)


