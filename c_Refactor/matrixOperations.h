/*
BCIT - HEXAPOD ROBOTICS CLUB
-----------------------------------------------------
# Author(s): Hassan Islam
# Date: 2024 - 05 - 18
# DESC:
#	 Common operations for 2-D matrix manipulations
#
*/
#pragma once


//*******************************************************************************
//
// Matrix Addition Operator
//
//*******************************************************************************
double** matrixAdd(double** mat1, double** mat2, int rowNum, int ColumnNum);



//*******************************************************************************
//
// Matrix Subtraction Operator
//
//*******************************************************************************
double** matrixSubtract(double** mat1, double** mat2, int rowNum, int ColumnNum);



//*******************************************************************************
//
// Matrix Multiplication Operator
//
//*******************************************************************************
double** matrixMultiply(double** mat1, double** mat2, int mat1rowNum,
                        int mat1ColumnNum, int mat2rowNum, int mat2ColumnNum);



//*******************************************************************************
//
// Matrix Inversion Operator
//
//*******************************************************************************
void matrixInvert(double** mat, int matdim);




double** matrixTranspose(double** matrix, int rows, int cols);



//*******************************************************************************
//
// ERO--row swap Operator
//
//*******************************************************************************
void swapRows(double** mat, int row1, int row2, int matdim);



//*******************************************************************************
//
// Jacobian Operator, implements finite difference algorithm
//
//*******************************************************************************
double** Jacobian(double x[3], double length[3], double point[3],
                              double** (*f)(double[3], double[3], double[3]));


