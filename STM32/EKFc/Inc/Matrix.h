/*
  Matrix.h

   Re-Created on: 05-Aug-2021
       Main Author: Roozbeh Abolpour  --> https://www.codeproject.com/Articles/5283245/Matrix-Library-in-C?sort=Position&view=Normal&fr=26
       Second - Venom
 */

#ifndef INC_MATRIX_H_
#define INC_MATRIX_H_

#include "stdlib.h"
#include "math.h"
#include "string.h"
#define gyro_noise 1e-6
#define accel_noise 1e-1

//Handle Struct for the Matrix
typedef struct Mat{

	float entries[50];
	int row;
	int col;

}Mat;

//Initialize the matrix with the custom values
Mat newmat_up(int r,int c,float* d);

//Universal Helper Function
Mat newmat(int r,int c,float d);

//Diagonal Identity matrix
Mat eye(int n);

//Zero Matrix
Mat zeros(int r,int c);

//Ones Matrix
Mat ones(int r,int c);

//Output a value at an index M[i][j]
float get(Mat M,int r,int c);

//Multiply the matrix with the scaler multiples
Mat scalermultiply(Mat M,float c);

//Summation Matrix
Mat sum(Mat A,Mat B);
//Subtraction Matrix
Mat minus(Mat A,Mat B);

//Multiply the matrix A and B --> Very Very important function to make your life easier.
Mat multiply(Mat A,Mat B);//Helper function

Mat removerow(Mat A,int r);
//Helper Function

Mat removecol(Mat A,int c);

//Helper Function
Mat removerow2(Mat A,Mat B,int r);
//Helper Function

Mat removecol2(Mat A,Mat B,int c);

//Transpose of the Matrix
Mat transpose(Mat A);

//Determinant
float det(Mat M);

//Adjoint of a matrix to compute the future inverse of a matrix
Mat adjoint(Mat A);

//The Inverse of a matrix
Mat inverse(Mat A);

//Horizontal Concatenate
Mat hconcat(int r1, int c1, float* A, int r2, int c2, float* B);

//Vertical Concatenate
Mat vconcat(int r1, int c1, float* A, int r2, int c2, float* B);

//Custom Functions
Mat eye_Q(int n);

Mat eye_R(int n);

#endif /* INC_MATRIX_H_ */
