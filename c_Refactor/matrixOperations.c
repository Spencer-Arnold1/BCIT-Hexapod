/*
BCIT - HEXAPOD ROBOTICS CLUB
-----------------------------------------------------
# Author(s): Hassan Islam
# Date: 2024 - 05 - 18
# DESC:
#    Common operations for 2-D matrix manipulations
#
*/


#include "matrixOperations.h"
#include <stdlib.h>


//*******************************************************************************
//
// Matrix Addition Operator
//
//*******************************************************************************
double** matrixAdd(double** mat1, double** mat2, int rowNum, int ColumnNum) {


	double** result = malloc(rowNum * sizeof(double*));  // allocating memory for rows

	int i, j;                                            // iterators

	for (i = 0; i < rowNum; i++) {

		result[i] = malloc(ColumnNum * sizeof(double)); // allocating memory for columns

		for (j = 0; j < ColumnNum; j++) {

			result[i][j] = mat1[i][j] + mat2[i][j];     // corresponding elements are added
		}
	}

	return result;

}



//*******************************************************************************
//
// Matrix Subtraction Operator
//
//*******************************************************************************
double** matrixSubtract(double** mat1, double** mat2, int rowNum, int ColumnNum) {

	double** result = malloc(rowNum * sizeof(double*));   // allocating memory for rows

    int i, j;                                             // iterators

	for (i = 0; i < rowNum; i++) {

		result[i] = malloc(ColumnNum * sizeof(double));   // allocating memory for columns

		for (j = 0; j < ColumnNum; j++) {

			result[i][j] = mat1[i][j] - mat2[i][j];       // corresponding elements are subtracted
		}
	}

	return result;

}



//*******************************************************************************
//
// Matrix Multiplication Operator
//
//*******************************************************************************
double** matrixMultiply(double** mat1, double** mat2, int mat1rowNum,
                        int mat1ColumnNum, int mat2rowNum, int mat2ColumnNum) {

	if (mat1ColumnNum != mat2rowNum)                        // Dimensions must match

		return NULL;

	int i, j, k;                                            // iterators

	double** result = malloc(mat1rowNum * sizeof(double*)); // allocating memory for rows

	for (i = 0; i < mat1rowNum; i++) {

		result[i] = malloc(mat2ColumnNum * sizeof(double)); // allocating memory for columns

	}

	for (i = 0; i < mat1rowNum; i++) {                     // multiplication operation begins

		for (j = 0; j < mat2ColumnNum; j++) {

			result[i][j] = 0;

			for (k = 0; k < mat1ColumnNum; k++) {

				result[i][j] += mat1[i][k] * mat2[k][j];   // dot products of corresponding row and column

			}
		}
	}
	return result;
}



//*******************************************************************************
//
// Matrix Inversion Operator
//
//*******************************************************************************
void matrixInvert(double** mat, int matdim) {

    int i, j, k;                                            // iterators


	double** identity = malloc(matdim * sizeof(double*));   // Create identity matrix for matrix augmentation

	for (i = 0; i < matdim; i++) {

		identity[i] = malloc(matdim * sizeof(double));

		for (j = 0; j < matdim; j++) {

			if (i == j)

				identity[i][j] = 1.0;

			else

				identity[i][j] = 0.0;

		}
	}


	for (k = 0; k < matdim; k++) {                          // Gaussian elimination starts here


		if (mat[k][k] == 0) {                               // finding initial pivot value by swapping rows if pivot is zero

			for (i = k + 1; i < matdim; i++) {

				if (mat[i][k] != 0) {

					swapRows(mat, k, i, matdim);
					swapRows(identity, k, i, matdim);

					break;
				}
			}
		}

		double pivot = mat[k][k];

		if (pivot == 0)                                     // Matrix is singular and cannot be inverted

			return; // exits function

		/*
		*  consider adding DETERMINANT calculation to determine if inversion is possible as well, not sure if needed.
		*  all matrices with redundant columns are irreversible, but do all irreversible matrices have a redundant column?
		*/


		for (j = 0; j < matdim; j++) {                      // normalize pivot row

			mat[k][j] /= pivot;
			identity[k][j] /= pivot;

		}

		for (i = 0; i < matdim; i++) {                          // Performing row reduction operations

			if (i != k) {                                       // skip the pivot row

				double factor = mat[i][k];                      // factor to eliminate the element below/above the pivot

				for (j = 0; j < matdim; j++) {                  // Loop through all columns

					mat[i][j] -= factor * mat[k][j];            // Eliminate element in the matrix

					identity[i][j] -= factor * identity[k][j];  // Apply the same operation to the identity matrix

				}
			}
		}

#ifdef DEBUGMODE
		printf("------Augmented Reduced Row Echelon form------ \n");

		for (i = 0; i < matdim; i++) {
			for (j = 0; j < matdim; j++)
				printf("%lf ", mat[i][j]);
			printf(" |  ");
			for (j = 0; j < matdim; j++)
				printf("%lf ", identity[i][j]);
			printf("\n");
		}
#endif
	}

	for (i = 0; i < matdim; i++) {                               // copying identity (inverted result) to input matrix

		for (j = 0; j < matdim; j++) {

			mat[i][j] = identity[i][j];

		}
	}

	for (i = 0; i < matdim; i++) {                               // freeing memory

		free(identity[i]);
	}

	free(identity);

}



//*******************************************************************************
//
// Jacobian Operator, implements finite difference algorithm
//
//*******************************************************************************
double** Jacobian(double x[3], double length[3], double point[3], double** (*f)(double[3], double[3], double[3])) {

    int i;                                                      // iterator

	double h = 1e-7;                                            // lim h --> 0

	double** result = malloc(3 * sizeof(double*));              // allocating memory for rows

	for (i = 0; i < 3; i++) {

		result[i] = malloc(3 * sizeof(double));                 // allocating memory for columns

	}

	double thetaplus_h[3] = { x[0] + h, x[1], x[2] };           // H = [ {h, 0, 0} , {0, h, 0}, {0, 0, h} ]
	double gammaplus_h[3] = { x[0], x[1] + h, x[2] };
	double phiplus_h[3] =   { x[0], x[1], x[2] + h };
	                                                            // x = { theta, gamma, phi }
	double** fx = (*f)(x, length, point);                       // F(x)
	double** ftheta = (*f)(thetaplus_h, length, point);         // F(x + h)
	double** fgamma = (*f)(gammaplus_h, length, point);
	double** fphi = (*f)(phiplus_h, length, point);

	for (i = 0; i < 3; i++) {
		result[i][0] = (ftheta[i][0] - fx[i][0]) / h;           // f(x) = ( f(x + h) - f(x) ) / h
		result[i][1] = (fgamma[i][0] - fx[i][0]) / h;
		result[i][2] = (fphi[i][0] - fx[i][0]) / h;
	}

	for (i = 0; i < 3; i++) {                                    // freeing memory
		free(fx[i]);
		free(ftheta[i]);
		free(fgamma[i]);
		free(fphi[i]);
	}

	free(fx);
	free(ftheta);
	free(fgamma);
	free(fphi);

	return result;
}




//*******************************************************************************
//
// ERO--row swap Operator
//
//*******************************************************************************
void swapRows(double** mat, int row1, int row2, int matdim) {

	double* temp = mat[row1];
	mat[row1] = mat[row2];
	mat[row2] = temp;

}





#ifdef DEBUGMODE
//********************************************************************************
//
// Debugging Function
//
//********************************************************************************

double** initializeRandomMatrix(int rowNum, int ColumnNum) {

	double** result = malloc(rowNum * sizeof(double*));

	for (i = 0; i < rowNum; i++)
		result[i] = malloc(ColumnNum * sizeof(double));

	for (i = 0; i < rowNum; i++) {
		for (j = 0; j < ColumnNum; j++) {
			result[i][j] = rand(time) % 16;
		}
	}
	return result;

}
#endif
