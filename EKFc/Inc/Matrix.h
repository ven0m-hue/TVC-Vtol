/*
 * Matrix.h
 *
 *  Re-Created on: 05-Aug-2021
 *      Main Author: Roozbeh Abolpour  --> https://www.codeproject.com/Articles/5283245/Matrix-Library-in-C?sort=Position&view=Normal&fr=26
 *      Second - Venom
 */

#ifndef INC_MATRIX_H_
#define INC_MATRIX_H_


#include "stdlib.h"
#include "Math.h"

#define gyro_noise 1e-5
#define accel_noise 1e-1

//Handle Struct for the Matrix
typedef struct Mat
{
	double* entries;
	int row;
	int col;

}Mat;

//Initialize the matrix with the custom values
Mat* newmat_up(int r,int c,double *d){
	Mat* M=(Mat*)malloc(sizeof(Mat));
	M->row=r;M->col=c;
	M->entries=(double*)malloc(sizeof(double)*r*c);
	int k=0;
	int l = 0;
	for(int i=1;i<=M->row;i++){
		for(int j=1;j<=M->col;j++){
			M->entries[k++]=d[l++];
		}
	}
	return M;
}
//Universal Helper Function
Mat* newmat(int r,int c,double d){
	Mat* M=(Mat*)malloc(sizeof(Mat));
	M->row=r;M->col=c;
	M->entries=(double*)malloc(sizeof(double)*r*c);
	int k=0;
	for(int i=1;i<=M->row;i++){
		for(int j=1;j<=M->col;j++){
			M->entries[k++]=d;
		}
	}
	return M;
}


//Free Allocation
void freemat(Mat* A){
	free(A->entries);
	free(A);
}//Once the computations are over.



//Diagonal Identity matrix
Mat* eye(int n)
{
	Mat* I = newmat(n,n,0);
	for(int i = 1;i <= n; i++)
	{
		I->entries[(i-1)*n+i-1]=1;
	}
	return I;
}

//Zero Matrix
Mat* zeros(int r,int c){
	Mat* Z=newmat(r,c,0);
	return Z;
}

//Ones Matrix
Mat* ones(int r,int c){
	Mat* O=newmat(r,c,1);
	return O;
}

//Output a value at an index M[i][j]
double get(Mat* M,int r,int c){
	double d=M->entries[(r-1)*M->col+c-1];
	return d;
}

//Multiply the matrix with the scaler multiples
Mat* scalermultiply(Mat* M,double c){
	Mat* B=newmat(M->row,M->col,0);
	int k=0;
	for(int i=0;i<M->row;i++){
		for(int j=0;j<M->col;j++){
			B->entries[k]=M->entries[k]*c;
			k+=1;
		}
	}
	return B;
}

//Summation Matrix
Mat* sum(Mat* A,Mat* B){
	int r=A->row;
	int c=A->col;
	Mat* C=newmat(r,c,0);
	int k=0;
	for(int i=0;i<r;i++){
		for(int j=0;j<c;j++){
			C->entries[k]=A->entries[k]+B->entries[k];
			k+=1;
		}
	}
	return C;
}

//Subtraction Matrix
Mat* minus(Mat* A,Mat* B){
	int r=A->row;
	int c=A->col;
	Mat* C=newmat(r,c,0);
	int k=0;
	for(int i=0;i<r;i++){
		for(int j=0;j<c;j++){
			C->entries[k]=A->entries[k]-B->entries[k];
			k+=1;
		}
	}
	return C;
}

//Multiply the matrix A and B --> Very Very important function to make your life easier.
Mat* multiply(Mat* A,Mat* B){
	int r1=A->row;
	int r2=B->row;
	int c1=A->col;
	int c2=B->col;
	if (r1==1&&c1==1){
		Mat* C=scalermultiply(B,A->entries[0]);
		return C;
	}else if (r2==1&&c2==1){
		Mat* C=scalermultiply(A,B->entries[0]);
		return C;
	}
	Mat* C=newmat(r1,c2,0);
	for(int i=1;i<=r1;i++){
		for(int j=1;j<=c2;j++){
			double de=0;
			for(int k=1;k<=r2;k++){
				de+=A->entries[(i-1)*A->col+k-1]*B->entries[(k-1)*B->col+j-1];
			}
			C->entries[(i-1)*C->col+j-1]=de;
		}
	}
	return C;
}

//Helper function
Mat* removerow(Mat* A,int r){
	Mat* B=newmat(A->row-1,A->col,0);
	int k=0;
	for(int i=1;i<=A->row;i++){
		for(int j=1;j<=A->col;j++){
			if(i!=r){
				B->entries[k]=A->entries[(i-1)*A->col+j-1];
				k+=1;
			}
		}
	}
	return B;
}

//Helper Function
Mat* removecol(Mat* A,int c){
	Mat* B=newmat(A->row,A->col-1,0);
	int k=0;
	for(int i=1;i<=A->row;i++){
		for(int j=1;j<=A->col;j++){
			if(j!=c){
				B->entries[k]=A->entries[(i-1)*A->col+j-1];
				k+=1;
			}
		}
	}
	return B;
}

//Helper Function
void removerow2(Mat* A,Mat* B,int r){
	int k=0;
	for(int i=1;i<=A->row;i++){
		for(int j=1;j<=A->col;j++){
			if(i!=r){
				B->entries[k++]=A->entries[(i-1)*A->col+j-1];
			}
		}
	}
}

//Helper Function
void removecol2(Mat* A,Mat* B,int c){
	int k=0;
	for(int i=1;i<=A->row;i++){
		for(int j=1;j<=A->col;j++){
			if(j!=c){
				B->entries[k++]=A->entries[(i-1)*A->col+j-1];
			}
		}
	}
}

//Transpose of the Matrix
Mat* transpose(Mat* A){
	Mat* B=newmat(A->col,A->row,0);
	int k=0;
	for(int i=1;i<=A->col;i++){
		for(int j=1;j<=A->row;j++){
			B->entries[k]=A->entries[(j-1)*A->row+i-1];
			k+=1;
		}
	}
	return B;
}

//Determinant
double det(Mat* M){
	int r=M->row;
	int c=M->col;
	if(r==1&&c==1){
		double d=M->entries[0];
		return d;
	}
	Mat* M1=removerow(M,1);
	Mat* M2=newmat(M->row-1,M->col-1,0);
	double d=0, si=+1;
	for(int j=1;j<=M->col;j++){
		double c=M->entries[j-1];
		removecol2(M1,M2,j);
		d+=si*det(M2)*c;
		si*=-1;
	}
	freemat(M1);
	freemat(M2);
	return d;
}

//Adjoint of a matrix to compute the future inverse of a matrix
Mat* adjoint(Mat* A){
	Mat* B=newmat(A->row,A->col,0);
	Mat* A1=newmat(A->row-1,A->col,0);
	Mat* A2=newmat(A->row-1,A->col-1,0);
	for(int i=1;i<=A->row;i++){
		removerow2(A,A1,i);
		for(int j=1;j<=A->col;j++){
			removecol2(A1,A2,j);
			double si=pow(-1,(double)(i+j));
			B->entries[(i-1)*B->col+j-1]=det(A2)*si;
		}
	}
	Mat* C=transpose(B);
	freemat(A1);
	freemat(A2);
	freemat(B);
	return C;
}

//The Inverse of a matrix
Mat* inverse(Mat* A){
	Mat* B=adjoint(A);
	double de=det(A);
	Mat* C=scalermultiply(B,1/de);
	freemat(B);
	return C;
}

//Horizontal Concatenate
Mat* hconcat(Mat* A,Mat* B){
	Mat* C=newmat(A->row,A->col+B->col,0);
	int k=0;
	for(int i=1;i<=A->row;i++){
		for(int j=1;j<=A->col;j++){
			C->entries[k]=A->entries[(i-1)*A->col+j-1];
			k++;
		}
		for(int j=1;j<=B->col;j++){
			C->entries[k]=B->entries[(i-1)*B->col+j-1];
			k++;
		}
	}
	return C;
}

//Vertical Concatenate
Mat* vconcat(Mat* A,Mat* B){
	Mat* C=newmat(A->row+B->row,A->col,0);
	int k=0;
	for(int i=1;i<=A->row;i++){
		for(int j=1;j<=A->col;j++){
			C->entries[k]=A->entries[(i-1)*A->col+j-1];
			k++;
		}
	}
	for(int i=1;i<=B->row;i++){
		for(int j=1;j<=B->col;j++){
			C->entries[k]=B->entries[(i-1)*B->col+j-1];
			k++;
		}
	}
	return C;
}


//Custom Functions
Mat* eye_Q(int n){
	Mat* I=newmat(n,n,0);
	for(int i=1;i<=n;i++){
		I->entries[(i-1)*n+i-1]= gyro_noise;
	}
	return I;
}

Mat* eye_R(int n){
	Mat* I=newmat(n,n,0);
	for(int i=1;i<=n;i++){
		I->entries[(i-1)*n+i-1]= accel_noise;
	}
	return I;
}

#endif /* INC_MATRIX_H_ */
