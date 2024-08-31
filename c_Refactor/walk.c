/*
 * walk.c
 *
 *  Created on: Aug. 27, 2024
 *      Author: Slaptop
 */
#include "walk.h"

const int wait_time = 300;
const int interval = 30;

unsigned long previousMils = 0;

//enum StepState { STEP1, STEP2, STEP3, STEP4 };

int currentStep = 1;


int legOffset = 100;
int frontStep = 0;
int sideStep = 0;
int height = 200;
int rotation = 0;
int stepHeight = 100;
int legWidth = 100;
int frontTilt = 0;
int sideTilt = 0;
int midSideOffset = 50;



//Left Middle
double pose1lm[3];
double pose2lm[3];
double pose3lm[3];
double pose4lm[3];

//Left Back
double pose1lb[3];
double pose2lb[3];
double pose3lb[3];
double pose4lb[3];

//Left Front
double pose1lf[3];
double pose2lf[3];
double pose3lf[3];
double pose4lf[3];

//Right Middle
double pose1rm[3];
double pose2rm[3];
double pose3rm[3];
double pose4rm[3];

//Right Back
double pose1rb[3];
double pose2rb[3];
double pose3rb[3];
double pose4rb[3];

//Right Front
double pose1rf[3];
double pose2rf[3];
double pose3rf[3];
double pose4rf[3];


void delayMs2(uint32_t ui32Ms) {

    SysCtlDelay(ui32Ms * (SysCtlClockGet() / 3 / 1000));
}


//interpolates points between key end effector positions and moves legs
void moveInterpolate(int leg_num, double* start_pos, double* end_pos, float progress)
{

    double target_pos[3];
    int i = 0;
    for (i = 0; i < 3; ++i) {
        target_pos[i] = start_pos[i] + (end_pos[i] - start_pos[i]) * progress;
    }
    setLegPosition(leg_num, target_pos);

}

//Stores the leg position in pose
void updatePose(double* pose, int x, int y, int z)
{
    pose[0] = x;
    pose[1] = y;
    pose[2] = z;
}

//updates end effector targets based on movement direction, speed and rotation
void updateTargets(double x_speed, double y_speed, double rotation_speed)
{
       frontStep = y_speed;
       sideStep = x_speed;
       rotation = rotation_speed;


       updatePose(pose1lm,legWidth+midSideOffset,0,-height-sideTilt);
       updatePose(pose2lm,legWidth-sideStep+midSideOffset,-frontStep,-height-sideTilt);
       updatePose(pose3lm,legWidth+midSideOffset,0,-height+stepHeight-sideTilt);
       updatePose(pose4lm,legWidth+sideStep+midSideOffset,frontStep,-height-sideTilt);

       updatePose(pose1lb,legWidth,-legOffset,-height-frontTilt-sideTilt);
       updatePose(pose2lb,legWidth-sideStep,-frontStep-legOffset,-height-frontTilt-sideTilt);
       updatePose(pose3lb,legWidth,-legOffset,-height+stepHeight-frontTilt-sideTilt);
       updatePose(pose4lb,legWidth+sideStep,frontStep-legOffset,-height-frontTilt-sideTilt);

       updatePose(pose1lf,legWidth,legOffset,-height+frontTilt-sideTilt);
       updatePose(pose2lf,legWidth-sideStep,-frontStep+legOffset,-height+frontTilt-sideTilt);
       updatePose(pose3lf,legWidth,legOffset,-height+stepHeight+frontTilt-sideTilt);
       updatePose(pose4lf,legWidth+sideStep,frontStep+legOffset,-height+frontTilt-sideTilt);

       updatePose(pose1rm,legWidth+midSideOffset,0,-height+sideTilt);
       updatePose(pose2rm,legWidth+sideStep+midSideOffset,-frontStep,-height+sideTilt);
       updatePose(pose3rm,legWidth+midSideOffset,0,-height+stepHeight+sideTilt);
       updatePose(pose4rm,legWidth-sideStep+midSideOffset,frontStep,-height+sideTilt);

       updatePose(pose1rb,legWidth,-legOffset,-height-frontTilt+sideTilt);
       updatePose(pose2rb,legWidth+sideStep,-frontStep-legOffset,-height-frontTilt+sideTilt);
       updatePose(pose3rb,legWidth,-legOffset,-height+stepHeight-frontTilt+sideTilt);
       updatePose(pose4rb,legWidth-sideStep,frontStep-legOffset,-height-frontTilt+sideTilt);

       updatePose(pose1rf,legWidth,legOffset,-height+frontTilt);
       updatePose(pose2rf,legWidth+sideStep,-frontStep+legOffset,-height+frontTilt+sideTilt);
       updatePose(pose3rf,legWidth,legOffset,-height+stepHeight+frontTilt+sideTilt);
       updatePose(pose4rf,legWidth-sideStep,frontStep+legOffset,-height+frontTilt+sideTilt);


}

//moves legs
void updateLegPositions()
{

       static unsigned long startMils = 0;
       unsigned long currentMils = timerVal;
       float progress = (currentMils - startMils) / wait_time;
       if (progress > 1.0) progress = 1.0;

       switch (currentStep) {
           case 1:
               moveInterpolate(0, pose1lb, pose2lb, progress);
               moveInterpolate(2, pose1lf, pose2lf, progress);
               moveInterpolate(4, pose1rm, pose2rm, progress);

               moveInterpolate(1, pose3lm, pose4lm, progress);
               moveInterpolate(3, pose3rf, pose4rf, progress);
               moveInterpolate(5, pose3rb, pose4rb, progress);
               break;
           case 2:
               moveInterpolate(0, pose2lb, pose3lb, progress);
               moveInterpolate(2, pose2lf, pose3lf, progress);
               moveInterpolate(4, pose2rm, pose3rm, progress);

               moveInterpolate(1, pose4lm, pose1lm, progress);
               moveInterpolate(3, pose4rf, pose1rf, progress);
               moveInterpolate(5, pose4rb, pose1rb, progress);
               break;
           case 3:
               moveInterpolate(0, pose3lb, pose4lb, progress);
               moveInterpolate(2, pose3lf, pose4lf, progress);
               moveInterpolate(4, pose3rm, pose4rm, progress);

               moveInterpolate(1, pose1lm, pose2lm, progress);
               moveInterpolate(3, pose1rf, pose2rf, progress);
               moveInterpolate(5, pose1rb, pose2rb, progress);
               break;
           case 4:
               moveInterpolate(0, pose4lb, pose1lb, progress);
               moveInterpolate(2, pose4lf, pose1lf, progress);
               moveInterpolate(4, pose4rm, pose1rm, progress);

               moveInterpolate(1, pose2lm, pose3lm, progress);
               moveInterpolate(3, pose2rf, pose3rf, progress);
               moveInterpolate(5, pose2rb, pose3rb, progress);
               break;
       }
       updatePositon();

       if (progress >= 1.0) {
           startMils = currentMils;
           currentStep = ((currentStep + 1) % 4);
       }


}



void updateLegNoClock()
{
      float progress = 0;
      for (progress = 0; progress <= 1; progress += 0.2)
      {

           switch (currentStep) {
               case 1:
                   moveInterpolate(0, pose1lb, pose2lb, progress);
                   moveInterpolate(2, pose1lf, pose2lf, progress);
                   moveInterpolate(4, pose1rm, pose2rm, progress);

                   moveInterpolate(1, pose3lm, pose4lm, progress);
                   moveInterpolate(3, pose3rf, pose4rf, progress);
                   moveInterpolate(5, pose3rb, pose4rb, progress);
                   break;
               case 2:
                   moveInterpolate(0, pose2lb, pose3lb, progress);
                   moveInterpolate(2, pose2lf, pose3lf, progress);
                   moveInterpolate(4, pose2rm, pose3rm, progress);

                   moveInterpolate(1, pose4lm, pose1lm, progress);
                   moveInterpolate(3, pose4rf, pose1rf, progress);
                   moveInterpolate(5, pose4rb, pose1rb, progress);
                   break;
               case 3:
                   moveInterpolate(0, pose3lb, pose4lb, progress);
                   moveInterpolate(2, pose3lf, pose4lf, progress);
                   moveInterpolate(4, pose3rm, pose4rm, progress);

                   moveInterpolate(1, pose1lm, pose2lm, progress);
                   moveInterpolate(3, pose1rf, pose2rf, progress);
                   moveInterpolate(5, pose1rb, pose2rb, progress);
                   break;
               case 4:
                   moveInterpolate(0, pose4lb, pose1lb, progress);
                   moveInterpolate(2, pose4lf, pose1lf, progress);
                  moveInterpolate(4, pose4rm, pose1rm, progress);

                   moveInterpolate(1, pose2lm, pose3lm, progress);
                   moveInterpolate(3, pose2rf, pose3rf, progress);
                   moveInterpolate(5, pose2rb, pose3rb, progress);
                   break;
           }
           updatePositon();

           if (progress >= 1.0) {
               currentStep = ((currentStep + 1) % 4);
           }
           delayMs2(20);
      }


}


void testMove(int legNum, double* pose)
{

    setLegPosition(legNum, pose);


}

void testCycle()
{

    //updatePose(pose1lm,legWidth+midSideOffset,0,-height-sideTilt);
    //updatePose(pose2lm,legWidth-sideStep+midSideOffset,-frontStep,-height-sideTilt);
    //updatePose(pose3lm,legWidth+midSideOffset,0,-height+stepHeight-sideTilt);
    //updatePose(pose4lm,legWidth+sideStep+midSideOffset,frontStep,-height-sideTilt);

    updateTargets(0,50,0);

    //double pose1[] = {377, 0 ,0};
    //double pose2[] = {156, 100, -60};

    while(1)
    {
        updateLegNoClock();
        //updateLegPositions();

        /*
        setLegPosition(1, pose1lm);
        updatePositon();
        delayMs2(300);

        setLegPosition(1, pose2lm);
        updatePositon();
        delayMs2(300);

        setLegPosition(1, pose3lm);
        updatePositon();
        delayMs2(300);

        setLegPosition(1, pose4lm);
        updatePositon();
        delayMs2(300);
        */

    }



}

