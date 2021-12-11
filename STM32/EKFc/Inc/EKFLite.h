/*
 * EKFLite.h
 *
 *  Created on: 14-Nov-2021
 *      Author: Venom
 */

#ifndef INC_EKFLITE_H_
#define INC_EKFLITE_H_

#include <stdint.h>
#include <math.h>
#include "Matrix.h"
typedef struct
{

	volatile float x[4]; // quaternion states.

	//Matrix library
	Mat Pi;
	Mat Qi;
	Mat Ri;
	//Noises and biases
	//float Racc; //For acc seperately instead of matrix
	//float Rmag; //For mag seperately instead of matrix

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

uint8_t EKF_Init(EKF_Handle_t *ekf);
void EKF_Predict(EKF_Handle_t *ekf, float *gyr, float Ts);
void EKF_Update(EKF_Handle_t *ekf, float *acc);



#endif /* INC_EKFLITE_H_ */
