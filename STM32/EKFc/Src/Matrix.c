/*
 * Matrix.c
 *
 *  Created on: Sep 9, 2021
 *      Author: Venom
 */

/*
 * Matrix.c
 *
 *  Created on: Sep 9, 2021
 *      Author: Venom
 */

#include "Matrix.h"

//Initialize the matrix with the custom values
Mat newmat_up(int r,int c,float *d){
	Mat M;
	M.row=r;M.col=c;
	memset(M.entries, 0, sizeof(M.entries));
	int k=0;
	int l = 0;
	for(int i=1;i<=M.row;i++){
		for(int j=1;j<=M.col;j++){
			M.entries[k++]=d[l++];
		}
	}
	return M;
}

Mat newmat(int r,int c,float d){
    	Mat M;
	M.row=r;M.col=c;
	memset(M.entries, 0, sizeof(M.entries));
	int k=0;
	for(int i=1;i<=M.row;i++){
		for(int j=1;j<=M.col;j++){
			M.entries[k++]=d;
		}
	}
	return M;
}
//Universal Helper Function


//Diagonal Identity matrix
Mat eye(int n){
	Mat I = newmat(n,n,0);
	for(int i=1; i<=n; i++){
		I.entries[(i-1)*n+i-1] = 1;
	}
	return I;
}

//Zero Matrix
Mat zeros(int r,int c){
	Mat Z = newmat(r,c,0);
	return Z;
}

//Ones Matrix
Mat ones(int r,int c){
	Mat O = newmat(r,c,1);
	return O;
}

//Output a value at an index M[i][j]
//Indexing starts from 1
float get(Mat M,int r,int c){
	float d = M.entries[(r-1)*M.col+c-1];
	return d;
}

//Multiply the matrix with the scaler multiples
Mat scalermultiply(Mat M,float c){
	Mat B=newmat(M.row,M.col,0);
	int k=0;
	for(int i=0;i<M.row;i++){
		for(int j=0;j<M.col;j++){
			B.entries[k]=M.entries[k]*c;
			k+=1;
		}
	}
	return B;
}
//Summation Matrix
Mat sum(Mat A,Mat B){
	int r=A.row;
	int c=A.col;
	Mat C=newmat(r,c,0);
	int k=0;
	for(int i=0;i<r;i++){
		for(int j=0;j<c;j++){
			C.entries[k]=A.entries[k]+B.entries[k];
			k+=1;
		}
	}
	return C;
}

//Subtraction Matrix
Mat minus(Mat A,Mat B){
	int r=A.row;
	int c=A.col;
	Mat C=newmat(r,c,0);
	int k=0;
	for(int i=0;i<r;i++){
		for(int j=0;j<c;j++){
			C.entries[k]=A.entries[k]-B.entries[k];
			k+=1;
		}
	}
	return C;
}

//Multiply the matrix A and B --> Very Very important function to make your life easier.
Mat multiply(Mat A,Mat B){
	int r1=A.row;
	int r2=B.row;
	//int c1=A.col;
	int c2=B.col;

	Mat C=newmat(r1,c2,0);
	for(int i=1;i<=r1;i++){
		for(int j=1;j<=c2;j++){
			float de=0;
			for(int k=1;k<=r2;k++){
				de+=A.entries[(i-1)*A.col+k-1]*B.entries[(k-1)*B.col+j-1];
			}
			C.entries[(i-1)*C.col+j-1]=de;
		}
	}
	return C;
}
/////////////////////////////////////Last edited //////////////////////////////////////////
//Helper function
Mat removerow(Mat A, int r){
	Mat B=newmat(A.row-1,A.col,0);
	int k=0;
	for(int i=1;i<=A.row;i++){
		for(int j=1;j<=A.col;j++){
			if(i!=r){
				B.entries[k]=A.entries[(i-1)*A.col+j-1];
				k+=1;
			}
		}
	}
	return B;
}

//Helper Function
Mat removecol(Mat A,int c){
	Mat B=newmat(A.row,A.col-1,0);
	int k=0;
	for(int i=1;i<=A.row;i++){
		for(int j=1;j<=A.col;j++){
			if(j!=c){
				B.entries[k]=A.entries[(i-1)*A.col+j-1];
				k+=1;
			}
		}
	}
	return B;
}

//Helper Function
void removerow2(Mat A,Mat B,int r){
	int k=0;
	for(int i=1;i<=A.row;i++){
		for(int j=1;j<=A.col;j++){
			if(i!=r){
				B.entries[k++]=A.entries[(i-1)*A.col+j-1];
			}
		}
	}
}

//Helper Function
void removecol2(Mat A,Mat B,int c){
	int k=0;
	for(int i=1;i<=A.row;i++){
		for(int j=1;j<=A.col;j++){
			if(j!=c){
				B.entries[k++]=A.entries[(i-1)*A.col+j-1];
			}
		}
	}
}

//Transpose of the Matrix
Mat transpose(Mat A){
	Mat B=newmat(A.col,A.row,0);
	int k=0;
	for(int i=1;i<=A.col;i++){
		for(int j=1;j<=A.row;j++){
			B.entries[k]=A.entries[(j-1)*A.row+i-1];
			k+=1;
		}
	}
	return B;
}

//Determinant
float det(Mat M){
	int r=M.row;
	int c=M.col;
	if(r==1&&c==1){
		float d=M.entries[0];
		return d;
	}
	Mat M1=removerow(M,1);
	Mat M2=newmat(M.row-1,M.col-1,0);
	float d=0, si=+1;
	for(int j=1;j<=M.col;j++){
		float c=M.entries[j-1];
		removecol2(M1,M2,j);
		d+=si*det(M2)*c;
		si*=-1;
	}

	return d;
}

//Adjoint of a matrix to compute the future inverse of a matrix
Mat adjoint(Mat A){
	Mat B=newmat(A.row,A.col,0);
	Mat A1=newmat(A.row-1,A.col,0);
	Mat A2=newmat(A.row-1,A.col-1,0);
	for(int i=1;i<=A.row;i++){
		removerow2(A,A1,i);
		for(int j=1;j<=A.col;j++){
			removecol2(A1,A2,j);
			float si=pow(-1,(float)(i+j));
			B.entries[(i-1)*B.col+j-1]=det(A2)*si;
		}
	}
	Mat C=transpose(B);
	return C;
}

//The Inverse of a matrix
Mat inverse(Mat A){
	Mat B=adjoint(A);
	float de=det(A);
	Mat C;
	if(de > 0)
		 C = scalermultiply(B,1/de);
	else
		 C = scalermultiply(B, 1e+10);
	return C;
}

//Horizontal Concatenate
Mat hconcat(int r1, int c1, float* A, int r2, int c2, float* B){
	Mat C=newmat(r1,c1+c2,0);
	int k=0, l=0, m = 0;
	for(int i=1;i<=r1;i++){
		for(int j=1;j<=c1;j++){
			C.entries[k]=A[l++];
			k++;
		}
		for(int j=1;j<=c2;j++){
			C.entries[k]=B[m++];
			k++;
		}
	}
	return C;
}

//Vertical Concatenate
Mat vconcat(int r1, int c1, float* A, int r2, int c2, float* B){
	Mat C=newmat(r1+r2,c1,0);
	int k=0; int l=0;
	for(int i=1;i<=r1;i++){
		for(int j=1;j<=c1;j++){
			C.entries[k]=A[l++];
			k++;
		}
	}
	l = 0;
	for(int i=1;i<=r2;i++){
		for(int j=1;j<=c2;j++){
			C.entries[k]=B[l++];
			k++;
		}
	}
	return C;
}

//Custom Functions
Mat eye_Q(int n){
	Mat I=newmat(n,n,0);
	for(int i=1;i<=n;i++){
		I.entries[(i-1)*n+i-1]= 1e-2;
	}
	return I;
}

Mat eye_R(int n){
	Mat I=newmat(n,n,0);
	for(int i=1;i<=n;i++){
		I.entries[(i-1)*n+i-1]= 1e-1;
	}
	return I;
}


