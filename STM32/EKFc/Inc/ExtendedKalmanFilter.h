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

/*
	 * 1. Noises
	 * 2. gyr Bias
	 * 3. Offset bias
 */
typedef struct
{

	float gyr_noise;
	float acc_noise;
	float mag_noise;

	float gyr_bias;

	float roll_offset;
	float pitch_offset;
	float yaw_offset;

	float pressure_offset;

}Biases_t;


/*
	 * State estimates
	 * P, Q, R matrix
	 * biases
	 * output

 */
typedef struct
{

	float x[7]; // quaternion states.

	//Matrix library
	Mat Pi;
	Mat Qi;
	Mat Ri;

	Biases_t bias;

	//From the IMU
	float roll;
	float pitch;
	float yaw;

	//From the BMP
	Mat Xi;
	Mat P;
	Mat Q;
	Mat R;
	Mat A, B, C;


	float altitude;

}EKF_Handle_t;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * API Calls IMU(Gyro and ACCEL)
 * 1. EKF init
 * 2. EKF Predict
 * 3. EKF Update
 * 4. EKF Extract the orientation roll and pitch only
 * 5. EKF Extract the orientation yaw
 */
uint8_t EKF_Init(EKF_Handle_t *ekf,float *acc, float *mag);
void EKF_Predict(EKF_Handle_t *ekf, float *gyr, float Ts);
void EKF_Update(EKF_Handle_t *ekf, float *acc);
void EKF_Extract(EKF_Handle_t *ekf);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * API Calls BMP and ACCEL
 * 1. KF init
 * 2. KF Predict
 * 3. KF Update
 * 4. EKF Extract the Altitude
 */
float get_Height(EKF_Handle_t *ekf, float pres);
uint8_t KF_Init(EKF_Handle_t *ekf, float Ts);
void KF_Predict_Update(EKF_Handle_t *ekf, float acc, float pres);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define Qpn  1e-3


#endif /* INC_EXTENDEDKALMANFILTER_H_ */
