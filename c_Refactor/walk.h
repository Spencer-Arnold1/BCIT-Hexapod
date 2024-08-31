/*
 * walk.h
 *
 *  Created on: Aug. 27, 2024
 *      Author: Slaptop
 */

#ifndef WALK_H_
#define WALK_H_



#include "leg.h"
#include "timers.h"


void delayMs2(uint32_t ui32Ms);
//enum StepState { STEP1, STEP2, STEP3, STEP4 };

//interpolates points between key end effector positions and moves legs
void moveInterpolate(int leg_num, double* start_pos, double* end_pos, float progress);

//calculates new leg position and stores it in pose
void updatePose(double* pose, int x, int y, int z);


//updates end effector targets based on movement direction, speed and rotation
void updateTargets(double x_speed, double y_speed, double rotaion);

//moves legs
void updateLegPositions();

void updateLegNoClock();

void testMove(int legNum, double* pose);

void testCycle();


#endif /* WALK_H_ */
