/*
BCIT - HEXAPOD ROBOTICS CLUB
-----------------------------------------------------
# Author(s): Hassan Islam
# Date: 2024 - 05 - 26
# DESC:
#    'Class' definition for a 3-joint 'RRR' joint Robotic leg. Each joint is a 180 degree Servo.
#
*/

#pragma once

#include "coordinate2JointAngle.h"
#include "pca9685.h"
#include <stdlib.h>
#include "mathlookup.h"


#define ANALYTICAL
//#define ITERATED

#define LEG1 0x06
#define LEG2 0x12
#define LEG3 0x1E

#define LEG4 0x06
#define LEG5 0x12
#define LEG6 0x1E

#define SLAVE_ADDRESS 0x40

//*******************************************************************************
//
// 3-joint RRR Robotic Leg CLASS definition
//
//*******************************************************************************
struct leg {

   //Public Methods
   void (*setPosition)(struct leg *self, double cartesianPosition[3]);
   void (*resetPosition)(struct leg *self);
   void (*getAngles)(struct leg*self, double angles[3]);

   //Private Methods;

   double angularPosition[3];
   double linkLengths[3];
   uint32_t baseAddress, driverAddress;

};

// call to send positional data to servos
void updatePositon();

void setAllPosition(double cartesianPosition[3]);
//void setLegPosition(int legNum, double cartesianPosition[3]);

// constructor
void legInit(struct leg*self, double coxa, double femur, double tibia, uint32_t baseAddress, uint32_t driverAddress );

// make a destructor

