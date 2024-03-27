from adafruit_servokit import ServoKit
from Leg import Leg
import time
import math

def move(x,y,z):
    L2 = 84*2
    L3 = 192*2
    H = math.sqrt(x**2 + y**2)
    L = math.sqrt(z**2 + H**2)
    A = math.atan2(z,H)
    B = math.acos((L**2+L2**2-L3**2)/(2*L*L2))
    A1 = math.atan2(x,y)*180/math.pi
    A2 = (B - A)*180/math.pi
    A3 = math.acos((L2**2+L3**2-L**2)/(2*L2*L3))*180/math.pi
    S1 = (A1 + 90)
    S2 = (A2) % 360
    S3 = (180 - A3) % 360
    kit.servo[1].angle = 80
    kit.servo[0].angle = S2
    kit.servo[2].angle = S3
    return 0

leg = Leg("Leg1", COXA=85, FEMUR=200, TIBIA=260)

kit = ServoKit(channels=16)
delay = 0.001

min_pulse = 500  # Minimum pulse width
max_pulse = 2500  # Maximum pulse width

for i in range(4):
    kit.servo[i].set_pulse_width_range(min_pulse, max_pulse)


try:
    while True:
        for z in range(100,300,1):
            angles = leg.inverseKinematics(target=[100,300,z])
            kit.servo[1].angle = 180-angles[0]
            kit.servo[0].angle = 180-angles[1]
            kit.servo[2].angle = 180-angles[2]
            time.sleep(delay)
        for y in range(100,300,1):
            angles = leg.inverseKinematics(target=[100,400-y,300])
            kit.servo[1].angle = 180-angles[0]
            kit.servo[0].angle = 180-angles[1]
            kit.servo[2].angle = 180-angles[2]
            time.sleep(delay)
        for z in range(100,300,1):
            angles = leg.inverseKinematics(target=[100, 100, 400-z])
            kit.servo[1].angle = 180-angles[0]
            kit.servo[0].angle = 180-angles[1]
            kit.servo[2].angle = 180-angles[2]
            time.sleep(delay)
        for y in range(100,300,1):
            angles = leg.inverseKinematics(target=[100,y,100])
            kit.servo[1].angle = 180-angles[0]
            kit.servo[0].angle = 180-angles[1]
            kit.servo[2].angle = 180-angles[2]
            time.sleep(delay)

except KeyboardInterrupt:
    pass




