/*
BCIT - HEXAPOD ROBOTICS CLUB
----------------------------------------------------------
# Author(s) : Hassan Islam
# Date : 2024 - 05 - 23
# DESC : This following implements Newton's Method to numerically solve for the inverse kinematics of a 3-DOF
#        robotic arm that otherwise has no analytical solution.This is done in order to find the joint
#        angles required to position the end effector(wrist) of the robotic arm at a given Cartesian position
#
# NAMING CONVENTIONS NOTE :
#               * Z-coordinate refers to height, X-coordinate is forward discplacement of arm, Y-coordinate is lateral displacement (Right handed co-ordinate convention is adopted)
#               * theta angle is between z and x axis (Transverse plane), gamma is angle between first and second link, phi is angle between second and third link,
#
*/
#pragma once

//*******************************************************************************
//
// Forward Kinematics Solution
//
//*******************************************************************************
double** func(double x[3], double l[3], double p[3]);



//*******************************************************************************
//
// Inverse Kinematics Solution
//
//*******************************************************************************
double** InverseKinematicSolution(double initialAngularPosition[3], double lengths[3], double targetPosition[3], double** (*f)(double[3], double[3], double[3]));



//*******************************************************************************
//
// WRAPPER FUNCTION. User will interface with program using this.
// initialAngularPosition: specifies initial guess for Newtons Method { theta, gamma, phi } 
// linkLengths
// targetCaresianPosition: position that the arm should move towards. Specified in
// cartesian coordinates.
//
// *******************************************************************************
double* coordinate2JointAngle(double initialAngularPosition[3], double linkLengths[3], double targetCartesianPosition[3]);
