/*
 * ExtendedKalmanFilter.h
 *
 *  Created on: 29-Jul-2021
 *      Author: Venom
 */

#ifndef INC_EXTENDEDKALMANFILTER_H_
#define INC_EXTENDEDKALMANFILTER_H_

#include "main.h"
#include "math.h"
#include "Matrix.h"


typedef struct
{
	/*
	 * 1. Noises
	 * 2. gyr Bias
	 * 3. Offset bias
	 */
	double gyr_noise;
	double acc_noise;
	double mag_noise;

	double gyr_bias;

	double roll_offset;
	double pitch_offset;
	double yaw_offset;
}Biases_t;


typedef struct
{
	/*
	 * State estimates
	 * P, Q, R matrix
	 * biases
	 * output
	 */
	float x[7]; // quaternion states.

#if 0 //This was prior to using the matrix library
	float P[7][7];
	float Q[7][7];
	float R[3][3];
#endif
	//Matrix library
	Mat *Pi;
	Mat *Qi;
	Mat *Ri;

	Biases_t bias;

	float roll;
	float pitch;
	float yaw;


}EKF_Handle_t;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * API Calls
 * 1. EKF init
 * 2. EKF Preditc
 * 3. EKF Update
 * 4. EKF Extract the orientation roll and pitch only
 * 5. EKF Extract the orientation yaw
 */
uint8_t EKF_Init(EKF_Handle_t *ekf, float *acc, float *mag);
void EKF_Predict(EKF_Handle_t *ekf, float *gyr, float Ts);
void EKF_Update(EKF_Handle_t *ekf, float *acc, float *mag);
void EKF_Extract(EKF_Handle_t *ekf);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Some generic macros
#define PI 3.142857f
#define g  9.80665f
#define Deg2Rad 180/ PI
#define Rad2Deg PI/ 180

#endif /* INC_EXTENDEDKALMANFILTER_H_ */
