/*
 * EKFLite.c
 *
 *  Created on: 14-Nov-2021
 *      Author: Venom
 */
#include "EKFLite.h"

#define PI 3.142857f
#define g  9.80665f
#define Rad2Deg (180.0f/PI)

static float invSqrt(float x);
static void quat2euler(EKF_Handle_t *ekf);

//Orientation Estimation

uint8_t EKF_Init(EKF_Handle_t *ekf)
{

	//1. P[4,4] -> Initialized to 4x4 Identity matrix
	ekf->Pi = eye(4);

	//2. Q -> Noise covariance
	ekf->Qi = eye_Q(4); // sigma^2

	//3. R -> Process covariance matrix
	ekf->Ri = eye_R(3);

	//5. Initialize first set of quaternions
	ekf->x[0] = 1.0f;
	ekf->x[1] = 0; ekf->x[2] = 0; ekf->x[3] = 0;

	//7. Test euler->quat and qut->euler, if true proceed
	ekf->roll = 0.0f;
	ekf->pitch = 0.0f;
	ekf->yaw = 0.0f;
	return 1;

}

void EKF_Predict(EKF_Handle_t *ekf, float *gyr, float Ts)
{

	//1. Subtract the bias with the gyro data
	//2. (GYRO) inits(INPUT)
	float gx = gyr[0];
	float gy = gyr[1];
	float gz = gyr[2];

	//3. Construct State W matrix
	//Sensor frame reffered to the earth frame
	float W[16] = {
					  0, -gx, -gy, -gz,
					  gx,  0,  gz, -gy,
					  gy, -gz,  0,  gx, 				//2-D matrix written in the form of 1-D matrix
					  gz,  gy, -gx,  0
					}; // 4x4 Angular velocity (w)

	//4. State transistion matrix and state space equations
	float q0 = ekf->x[0];
	float q1 = ekf->x[1];
	float q2 = ekf->x[2];
	float q3 = ekf->x[3];

	//Estimate states
	ekf->x[0] += 0.5 * Ts * ( W[0]*q0 + W[1]*q1 + W[2]*q2 + W[3]*q3);
	ekf->x[1] += 0.5 * Ts * ( W[4]*q0 + W[5]*q1 + W[6]*q2 + W[7]*q3);
	ekf->x[2] += 0.5 * Ts * ( W[8]*q0 + W[9]*q1 + W[10]*q2 + W[11]*q3);
	ekf->x[3] += 0.5 * Ts * ( W[12]*q0 + W[13]*q1 + W[14]*q2 + W[15]*q3);

	//Normalize
	float norm = invSqrt(ekf->x[0] * ekf->x[0] + ekf->x[1] * ekf->x[1] + ekf->x[2] * ekf->x[2] + ekf->x[3] * ekf->x[3]);

	ekf->x[0] *= norm; ekf->x[1] *= norm; ekf->x[2] *= norm; ekf->x[3] *= norm;

	//Re-extract the quaternions
	q0 = ekf->x[0];
	q1 = ekf->x[1];
	q2 = ekf->x[2];
	q3 = ekf->x[3];


	//7. Linearize the model, compute jacobians
	//Compute the jacobian
	Mat Ak = newmat_up(4, 4, W);
	// 4x4 Matrix is initialized

	//Linearized state transistion model
	Mat F = sum(eye(4), scalermultiply(Ak, 0.5 * Ts)); // Diagonal identity matrix

	//P -> 4x4 estimated error covarinace matrix
	ekf->Pi = sum(multiply(F, multiply(ekf->Pi, transpose(F))), ekf->Qi);

}



void EKF_Update(EKF_Handle_t *ekf, float *acc)
{
	/*edit refactored*/

	//1. (ACCEL/MAG)inits(Measurement)
	float norm = invSqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
	acc[0] *= norm;
	acc[1] *= norm;
	acc[2] *= norm;

	//2. Construct the observation matrix
	float q0 = ekf->x[0];
	float q1 = ekf->x[1];
	float q2 = ekf->x[2];
	float q3 = ekf->x[3];

	//Non-linear
	float Hk[12] = {
						  2*q2, -2*q3,  2*q0, -2*q1,
						 -2*q1, -2*q0, -2*q3, -2*q2,
						 -2*q0,  2*q1,  2*q2, -2*q3
				     };  /*edit* added scalar multiplication manually */

	//3. Observation model
	//Linearized by finding the jacobian
	Mat H = newmat_up(3,4,Hk); // [3x4] State Observation matrix


	//Nasty computation start
	//4. Compute the Kalman gain

	//Find the inverse of the matrix
	Mat Sp = sum(multiply(H, multiply(ekf->Pi, transpose(H))), ekf->Ri); //[3x3]

	Mat K = multiply(ekf->Pi, multiply(transpose(H), inverse(Sp))); // [4x3]

	//* 5. Jacobian of the observation model
	float hk[3] = {
					-2*(q1 * q3 - q0 * q2),

					-2*(q0 * q1 + q2 * q3),

					-1*(q0*q0 + q3*q3 - q1*q1 - q2*q2)

				   };

	Mat hm = newmat_up(3, 1, hk); //3x1

	Mat Z = newmat_up(3, 1, acc); //3x1

	//Error variance
	Mat S = minus(Z, hm); //3x1
	//Predicted state matrix
	Mat K_S = multiply(K, S); // [4x3] * [3x1] --> [4x1]

	ekf->x[0] = ekf->x[0] + get(K_S, 1, 1);
	ekf->x[1] = ekf->x[1] + get(K_S, 2, 1);
	ekf->x[2] = ekf->x[2] + get(K_S, 3, 1);
	ekf->x[3] = ekf->x[3] + get(K_S, 4, 1);

	//Normalize
	norm = invSqrt(ekf->x[0] * ekf->x[0] + ekf->x[1] * ekf->x[1] + ekf->x[2] * ekf->x[2] + ekf->x[3] * ekf->x[3]);
	ekf->x[0] *= norm; ekf->x[1] *= norm; ekf->x[2] *= norm;ekf->x[3] *= norm;

	ekf->Pi = multiply(minus(eye(4), multiply(K, H)), ekf->Pi);  //4x4

	//Extraction block
	quat2euler(ekf);
}


//ALTITUDE ESTIMATION
uint8_t KF_Init(EKF_Handle_t *ekf, float Ts)
{
		//Measurement matrix
		ekf->R = eye_R(2);

		//Process noise
		float q[4] = {  Qpn*(Ts*Ts*Ts*Ts)/4, Qpn*(Ts*Ts*Ts)/2,
					    Qpn*(Ts*Ts*Ts)/2,    Qpn*(Ts*Ts)      };

		ekf->Q = newmat_up(2, 2, q);

		//Error covariance
		ekf->P = eye(2);

		//Initial state
		float xi[2] = { 0, 0}; //get_Height(ekf,ekf->bias.pressure_offset)
		ekf->Xi = newmat_up(1, 2, xi);

		//State model
		float a[4] = {1, Ts, 0, 1};
		ekf->A = newmat_up(2, 2, a);

		//Input matrix
		float b[2] = {(Ts*Ts)/2, Ts};
		ekf->B = newmat_up(1, 2, b);

		//State Observer
		float c[4] = {1, 0, 0, 0};
		ekf->C = newmat_up(2, 2, c);

		return 1;
}



void KF_Predict_Update(EKF_Handle_t *ekf, float acc, float pres)
{

	//Estimated State update equation
	Mat X = sum(multiply(ekf->A, ekf->Xi), scalermultiply(ekf->B, acc));

	//Predicted State update equation
	ekf->P = sum(multiply(ekf->A, multiply(ekf->P, transpose(ekf->A))), ekf->Q);

	//Measurement model
	float y[2] = {get_Height(ekf, pres), 0};
	Mat Y = newmat_up(1, 2, y);

	//Corrector equation
	Mat S = minus(Y, multiply(ekf->C, X));

	//Kalman gain
	Mat Sp = sum(multiply(ekf->C, multiply(ekf->P, transpose(ekf->C))), ekf->R);
	Mat K = multiply(ekf->P, multiply(transpose(ekf->C), inverse(Sp)));

	//Update state equation
	ekf->Xi = sum(ekf->Xi, multiply(K, S));

	//Update state equation
	ekf->P = multiply(minus(eye(2), multiply(K, ekf->C)), ekf->P);

	//Output
	ekf->altitude = ekf->Xi.entries[0];

}



static void quat2euler(EKF_Handle_t *ekf)
{
	float q[4];

	q[0] = ekf->x[0];
	q[1] = ekf->x[1];
	q[2] = ekf->x[2];
	q[3] = ekf->x[3];

   // rotatation matrix interms of quaternions
   float R[3][3] = {
					   { 1 - 2 * (q[3] * q[3]+q[2] * q[2]),      2 * (q[1] * q[2]-q[0] * q[3]),      2 * (q[1] * q[3]+q[0] * q[2])},
					   { 2 * (q[1] * q[2]+q[0] * q[3]),      1 - 2 * (q[3] * q[3]+q[1] * q[1]),      2 * (q[2] * q[3]-q[0] * q[1])},
					   { 2 * (q[1] * q[3]-q[0] * q[2]),          2 * (q[2] * q[3]+q[0] * q[1]),  1 - 2 * (q[1] * q[1]+q[2] * q[2])}
			       };

   ekf->roll  = ((atan2(R[2][1], R[2][2]))) * Rad2Deg;
   ekf->pitch = ((asin(-R[2][0]))) * Rad2Deg;
   ekf->yaw   = atan2(R[1][0], R[0][0]) * Rad2Deg;

}

float get_Height(EKF_Handle_t *ekf, float pres)
{
	  return (44330 * (1.0 - pow((pres/ekf->bias.pressure_offset), 0.1903)));
}

//Fastest method for inverse sqrt
static float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;

	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));

	return y;
}



