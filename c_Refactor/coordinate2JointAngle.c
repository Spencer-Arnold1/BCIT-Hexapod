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

#include "matrixOperations.h"
#include "coordinate2JointAngle.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "mathlookup.h"

#ifndef M_PI
    #define M_PI 3.14159265358979323846 // not always defined
#endif

#define MAXITERATIONS 1000
//todo: change tolerance to increase speed (was 1e-6)
#define TOLERANCE 0.1



//***************************************************************************************
//
// Forward Kinematics Solution
//
//***************************************************************************************

double** func(double x[3], double l[3], double p[3]) {

    /*
     *l[0] = Tibia
     *l[1] = Femur
     *l[2] = Coxa
     */

    int i;                                                               // iterator

    double theta = x[0], gamma = x[1], phi = x[2];

    double** result = malloc(3 * sizeof(double*));                       // allocating memory for rows

    for (i = 0; i < 3; i++) {
        result[i] = malloc(1 * sizeof(double));                          // allocating memory for columns
    }

    // pre-calculated function using MATLAB
    result[0][0] = l[0] * (cos_fast(gamma) * cos_fast(theta) * cos_fast(phi) - sin_fast(gamma) * cos_fast(theta) * sin_fast(phi)) + l[1] * cos_fast(gamma) * cos_fast(theta) + l[2] * cos_fast(theta) - p[0];
    result[1][0] = -l[0] * (sin_fast(gamma) * sin_fast(theta) * sin_fast(phi) - cos_fast(gamma) * sin_fast(theta) * cos_fast(phi)) + l[1] * cos_fast(gamma) * sin_fast(theta) + l[2] * sin_fast(theta) - p[1];
    result[2][0] = l[0] * (cos_fast(gamma) * sin_fast(phi) + sin_fast(gamma) * cos_fast(phi)) + l[1] * sin_fast(gamma) - p[2];

    // Note: Although F(x) is a vector, it is returned as a 2-D matrix to allow for compatibility between other functions

    return result;
}


//***************************************************************************************
//
// Inverse Kinematics Solution
//
//***************************************************************************************

double** InverseKinematicSolution(double initialAngularPosition[3], double lengths[3], double targetPosition[3],
                                  double** (*f)(double[3], double[3], double[3])) {

    double** JacobianResult, ** temp, ** f_u, ** x_iteration, ** initial_x ;

    initial_x = malloc(3 * sizeof(double*));
    double in_x[3];

    int i, j, iter;                                                                      // iterators

    for (i = 0; i < 3; i++) {

        initial_x[i] = malloc(1 * sizeof(double));

        initial_x[i][0] = initialAngularPosition[i];
    }

    for (iter = 0; iter < MAXITERATIONS; iter++) {                                  // Newtons Method approximation begins here

        for (j = 0; j < 3; j++)                                                     // initial guess copied into new matrix
            in_x[j] = initial_x[j][0];

        // Newton's Method iteration x_n+1 = xn - (J^-1)*f(xn)

        f_u  = (*f)(in_x, lengths, targetPosition);                                  // f(xn)
        JacobianResult = Jacobian(in_x, lengths, targetPosition, (*f));             // J
        matrixInvert(JacobianResult, 3);                                            // J^-1
        temp = matrixMultiply(JacobianResult, f_u, 3, 3, 3, 1);                     // (J^-1)*f(xn)
        x_iteration = matrixSubtract(initial_x, temp, 3, 1);                        // xn - (J^-1)*f(xn)


        double maxChange = 0.0;                                                     // iterates until the change in xn and x_(n+1) is negligeable (TOLERANCE)

        for (i = 0; i < 3; i++) {

            double change = fabs(x_iteration[i][0] - initial_x[i][0]);

            if (change > maxChange)
                maxChange = change;

            initial_x[i][0] = x_iteration[i][0];
        }

        for (i = 0; i < 3; i++) {                                                   // freeing memory
            free(f_u[i]);
            free(temp[i]);
            free(x_iteration[i]);
        }

        free(f_u);
        free(temp);
        free(x_iteration);

        for (i = 0; i < 3; i++) {
            free(JacobianResult[i]);
        }

        free(JacobianResult);

        if (maxChange < TOLERANCE)                                                  // breaks here when difference is negligeable
               break;

    }

    return initial_x;                                                              // return final iteration
}



//***************************************************************************************
//
// WRAPPER FUNCTION. User will interface with program using this.
// initialAngularPosition: specifies initial guess for Newtons Method { theta, gamma, phi }
// linkLengths
// targetCaresianPosition: position that the arm should move towards. Specified in
// cartesian coordinates.
//
// ***************************************************************************************
double* coordinate2JointAngle(double initialAngularPosition[3], double linkLengths[3], double targetCartesianPosition[3]) {

    int i;                                                                                                              // iterator

    double** solution = InverseKinematicSolution(initialAngularPosition, linkLengths, targetCartesianPosition, func);   // returns radian solution

    double* jointAngles = malloc(3 * sizeof(double));                                                                   // allocating memory for vector result

    for (i = 0; i < 3; i++) {

        jointAngles[i] = solution[i][0] * 180 / M_PI;                                                                   // Convert to degrees

        double angle = fmod(jointAngles[i], 360.0);                                                                     // Normalize the angle

        if (angle < 0)
            angle += 360.0;                                                                                             // Bring angle into domain on 0 < angle < 360

        if (angle > 180.0)                                                                                              // Further reduce domain to -180 < angle < 180
            angle -= 360.0;

        jointAngles[i] = angle;                                                                                         // store result in vector
    }

    for (i = 0; i < 3; i++)                                                                                             // freeing memory
        free(solution[i]);

    free(solution);

    return jointAngles;
}




#ifdef DEBUGMODE
//****************************************************************************************
//
// Debugging Functions
//
//****************************************************************************************
double** CreateRandomMatrix(int rowNum, int ColumnNum) {
    double** result = malloc(rowNum * sizeof(double*));

    for (i = 0; i < rowNum; i++)
        result[i] = malloc(ColumnNum * sizeof(double));

    for (i = 0; i < rowNum; i++) {
        for (j = 0; j < ColumnNum; j++) {
            result[i][j] = rand() % 16;
        }
    }
    return result;
}

void debug() {
    // For Testing
    double** matrixOne = CreateRandomMatrix(ROWMAT1, COLUMNMAT1);
    double** matrixTwo = CreateRandomMatrix(ROWMAT2, COLUMNMAT2);
    double** result;

    printf("------MATRIX ONE------ \n");
    for (i = 0; i < ROWMAT1; i++) {
        for (j = 0; j < COLUMNMAT1; j++)
            printf("%lf ", matrixOne[i][j]);
        printf("\n");
    }

    printf("------MATRIX TWO------ \n");
    for (i = 0; i < ROWMAT2; i++) {
        for (j = 0; j < COLUMNMAT2; j++)
            printf("%lf ", matrixTwo[i][j]);
        printf("\n");
    }

    result = matrixMultiply(matrixOne, matrixTwo, ROWMAT1, COLUMNMAT1, ROWMAT2, COLUMNMAT2);

    printf("------RESULT MATRIX------ \n");
    for (i = 0; i < ROWMAT1; i++) {
        for (j = 0; j < COLUMNMAT2; j++)
            printf("%lf ", result[i][j]);
        printf("\n");
    }

    matrixInvert(result, COLUMNMAT1);

    printf("------INVERTED MATRIX------ \n");
    for (i = 0; i < ROWMAT1; i++) {
        for (j = 0; j < COLUMNMAT2; j++)
            printf("%lf ", result[i][j]);
        printf("\n");
    }

    // Garbage collection
    for (i = 0; i < 3; i++) {
        free(result[i]);
        free(matrixOne[i]);
        free(matrixTwo[i]);
    }
    free(result);
    free(matrixOne);
    free(matrixTwo);
}

#endif
