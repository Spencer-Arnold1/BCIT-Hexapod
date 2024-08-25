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
#include <math.h>


#ifndef M_PI
    #define M_PI 3.14159265358979323846 // not always defined
#endif


int numberofLegs = 0; // tracks number of legs initialized

struct leg **legs; // container for leg instances



//*******************************************************************************
//
// Getter for Leg angles
//
//*******************************************************************************
void getAngles(struct leg *self, double angles[3]){

    int i;

    for (i = 0; i < 3; i++) {
        angles[i] = self->angularPosition[i];
    }

}



//*******************************************************************************
//
// Setting Servo angles
//
//*******************************************************************************
void updatePositon(){

    int i = 0, j = 0;

    double jointAngles[6][3]; // hardcoded 6 for now, must use malloc in the future.
    double positionAngles[3];

    // *concentric writing
    for(i = 0; i < numberofLegs; i++ ) {

        legs[i]->getAngles(legs[i], positionAngles);

        positionAngles[1] = positionAngles[1]*(-1);
        positionAngles[2] = positionAngles[2] + 90;

        if(legs[i]->driverAddress == 0x41){
            jointAngles[i][0] = positionAngles[0]*(-1);
            jointAngles[i][1] = positionAngles[1]*(-1);
            jointAngles[i][2] = positionAngles[2]*(-1);
        }
        else{
            jointAngles[i][0] = positionAngles[0];
            jointAngles[i][1] = positionAngles[1];
            jointAngles[i][2] = positionAngles[2];
        }

    }
    for( i = 0; i < numberofLegs; i++){
        for( j = 0; j < 3; j++) {
            servoControl(jointAngles[i][j], legs[i]->baseAddress + 4*j, legs[i]->driverAddress);
        }
   }
}


//*******************************************************************************
//
// Setting Position of Leg
//
//*******************************************************************************
void setPosition(struct leg *self, double cartesianPosition[3]){  // int steps

    double* angles;

    /* The shoulder joint position is calculated using the Cartesian position's unit vector. This position is then used as an initial guess for the Newton-Raphson algorithm.
     * This approach ensures that the generated solution aligns with the expected, and physically possible position of the leg, as there can be two possible solutions.
     */

    // Calculate the azimuthal angle using the Cartesian position coordinates
    double azimuthalAngle = arctan_fast(cartesianPosition[1]/cartesianPosition[0]);
    //double azimuthalAngle = atan2(cartesianPosition[1], cartesianPosition[0]);

    // Use the azimuthal angle as part of the initial guess for the Newton-Raphson algorithm
    double angularPosition[] = { azimuthalAngle, 0.1, 0.1 };

    angles = coordinate2JointAngle(angularPosition, self->linkLengths, cartesianPosition);

    int i;

    for (i = 0; i < 3; i++) {
       self->angularPosition[i] = angles[i];
    }

    free(angles);
}



//*******************************************************************************
//
// Setting Position of all instances of Leg
//
//*******************************************************************************
void setAllPosition(double cartesianPosition[3]) {

    int i;

    for(i = 0; i < numberofLegs ; i++) {

        legs[i]->setPosition(legs[i], cartesianPosition);

    }

}


//*******************************************************************************
//
// Reseting Position of Leg
//
//*******************************************************************************
void resetPosition(struct leg *self){

    legInit(self, self->linkLengths[2], self->linkLengths[1], self->linkLengths[0], self->baseAddress, self->driverAddress);
}




//*******************************************************************************
//
// Leg Initialization
//
//*******************************************************************************
void legInit(struct leg* self, double coxa, double femur, double tibia, uint32_t baseAddress, uint32_t driverAddress ) {

    self->linkLengths[0] = tibia;
    self->linkLengths[1] = femur;
    self->linkLengths[2] = coxa;

    self->baseAddress = baseAddress;

    int i;

    for (i = 0; i < 3; i++) {
        self->angularPosition[i] = 0.1;
    }

    self->setPosition = setPosition;
    self->resetPosition = resetPosition;
    self->getAngles = getAngles;
    self->driverAddress = driverAddress;


    if (numberofLegs == 0) {
        // Initialize container for all leg instances
        legs = malloc(sizeof(struct leg*));
        if (legs == NULL) {

        }
    } else {

        struct leg **temp = realloc(legs, (numberofLegs + 1) * sizeof(struct leg*));
        if (temp == NULL) {

        }
        legs = temp;
    }

    // Copy address of leg to legs container
    legs[numberofLegs] = self;

    numberofLegs++; // Increment the number of legs initialized

}




















