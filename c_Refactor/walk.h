/*
 * walk.h
 *
 *  Created on: Aug. 27, 2024
 *      Author: Slaptop
 */

#pragma once

#include "leg.h"

#define LEG1 0x06
#define LEG2 0x12
#define LEG3 0x1E

#define LEG4 0x06
#define LEG5 0x12
#define LEG6 0x1E

#define SLAVE_ADDRESS_ 0x40

extern void delayMs(uint32_t ui32Ms);

//interpolates points between key end effector positions and moves legs
void moveInterpolate(struct leg *leg, double* start_pos, double* end_pos, float progress);

//calculates new leg position and stores it in pose
void updatePose(double* pose, int x, int y, int z);

//updates end effector targets based on movement direction, speed and rotation
void updateTargets(double x_speed, double y_speed, double rotaion);

//constructor
void walkInit();

void updateLegNoClock();

void testMove(int legNum, double* pose);

void testCycle();


// initializing leg modules
struct leg leg1;    //legInit(&leg1, 32, 124, 221, LEG1, SLAVE_ADDRESS_ );
struct leg leg2;    //legInit(&leg2, 32, 124, 221, LEG2, SLAVE_ADDRESS_ );
struct leg leg3;    //legInit(&leg3, 32, 124, 221, LEG3, SLAVE_ADDRESS_ );
struct leg leg4;   // legInit(&leg4, 32, 124, 221, LEG4, SLAVE_ADDRESS_  + 1);
struct leg leg5;    //legInit(&leg5, 32, 124, 221, LEG5, SLAVE_ADDRESS_  + 1);
struct leg leg6;    //legInit(&leg6, 32, 124, 221, LEG6, SLAVE_ADDRESS_  + 1);


