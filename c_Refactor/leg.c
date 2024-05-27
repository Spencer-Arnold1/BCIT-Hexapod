/*
BCIT - HEXAPOD ROBOTICS CLUB
-----------------------------------------------------
# Author(s): Hassan Islam
# Date: 2024 - 05 - 26
# DESC:
#    A 3-joint 'RRR' joint Robotic leg. Each joint is a 180 degree Servo.
#
*/


#include "leg.h"

//*******************************************************************************
//
// Setting Position of Leg
//
//*******************************************************************************
void setPosition(struct leg *self, double cartesianPosition[3]){  // int steps

    double* angles = malloc(3*sizeof(angles));

    if( angles == NULL){
        return;
    }

    double angularPositon[] = { 0.1, 0.1, 0.1 };

    angles = coordinate2JointAngle(angularPositon, self->linkLengths, cartesianPosition);

    int i; 

    for (i = 0; i < 3; i++) {
       self->angularPositon[i] = angles[i];
    }


    free(angles);

}


//*******************************************************************************
//
// Reseting Position of Leg
//
//*******************************************************************************
void resetPosition(struct leg *self){

    legInit(self, self->linkLengths[0], self->linkLengths[0], self->linkLengths[0]);
}



//*******************************************************************************
//
// Getter for Leg angles
//
//*******************************************************************************
void getAngles(struct leg *self, double angles[3]){

    int i;

    for (i = 0; i < 3; i++) {
        angles[i] = self->angularPositon[i];
    }

}



//*******************************************************************************
//
// Setting Servo angles
//
//*******************************************************************************
void setServoAngles(struct leg *self){

    double positionAngles[3];

    getAngles(self, positionAngles);

    int i;

    for(i = 0; i < 3; i++) {
           if(positionAngles[i] >= 90) {
               positionAngles[i] = 90;
           } else if(positionAngles[i] <= -90) {
               positionAngles[i] = -90;
           }
    }
    //Rest of code here will interface with  PCA9685
}



//*******************************************************************************
//
// Leg Initialization
//
//*******************************************************************************
void legInit(struct leg* self, double coxa, double femur, double tibia) {

    self->linkLengths[0] = coxa;
    self->linkLengths[1] = femur;
    self->linkLengths[2] = tibia;

    int i;

    for (i = 0; i < 3; i++) {
        self->angularPositon[i] = 0.1;
    }

    self->setPosition = setPosition;
    self->setServoAngles = setServoAngles;
    self->resetPosition = resetPosition;
    self->getAngles = getAngles; 

}




















