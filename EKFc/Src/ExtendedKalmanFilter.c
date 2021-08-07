/*
 * ExtendedKalmanFilter.c
 *
 *  Created on: 6-Aug-2021
 *      Author: Venom
 */

/*
 * TO DO's
 * 1. Qinit and Rinit improvment
 * 2. #Define for more redability
 */

#include "ExtendedKalmanFilter.h"


//Some helper function prototypes
static void euler2quat(EKF_Handle_t *ekf, float roll, float pitch, float yaw);
static void quat2euler(EKF_Handle_t *ekf);
static void get_Acc_Mag(EKF_Handle_t *ekf, float *accel_data, float *mag_data);
static float yaw_correction(float roll, float pitch, float *mag_data);
static float invSqrt(float x);


uint8_t EKF_Init(EKF_Handle_t *ekf,float *acc, float *mag)
{
	/*
	 * Matrix inits
	 * 1. P[7,7] -> 7x7
	 * 2. Q[7,7] -> 7x7
	 * 3. R[3,3] -> 3x3
	 * 4. Initialize first set of roll, pitch, yaw
	 * 5. Initialize first set of quaternions
	 * 6. Initialize the rest of the states with the input gyro bias
	 * 7. Test euler->quat and qut->euler, if true proceed
	 * //Future
	 * 8. Accel_calibrate - roll, pitch compensation
	 */
	//1. P[7,7] -> 7x7
	ekf->Pi = eye(7);   // Initialized to 7x7 Identity matrix

	//2. Q[7,7] -> 7x7
	ekf->Qi = eye_Q(7);
	//7x7 Noise covariance matrix

	//3. R[3,3] -> 3x3
	ekf->Ri = eye_R(3);
	//3x3 Process covariance matrix

	//4. Compute first set of roll, pitch, yaw
	get_Acc_Mag(ekf, acc, mag);

	//5. Initialize first set of quaternions
	euler2quat(ekf, ekf->roll, ekf->pitch, ekf->yaw);

	//6. Initialize the rest of the states with the input gyro bias
	ekf->x[4] = ekf->bias.gyr_bias; ekf->x[5] = ekf->bias.gyr_bias; ekf->x[6] = ekf->bias.gyr_bias;

	//7. Test euler->quat and qut->euler, if true proceed
	return 1;

}

void EKF_Predict(EKF_Handle_t *ekf, float *gyr, float Ts)
{
	/*
	 * 1. Subtract the bias with the gyro data
	 * 2. (GYRO) inits(INPUT)
	 * 3. Construct State W matrix
	 * 4. State transistion matrix and state space equations
	 * 5. Update the sample time, update the quaternions
	 * 6. Normalize the equations
	 * 7. Linearize the model, compute jacobians
	 * 8. Compute the Estimated error covarinace matrix
	 */
	//1. Subtract the bias with the gyro data
	float gxb = ekf->bias.gyr_bias;
	float gyb = ekf->bias.gyr_bias;
	float gzb = ekf->bias.gyr_bias;

	//2. (GYRO) inits(INPUT)
	float gx = gyr[0] - gxb;
	float gy = gyr[1] - gyb;
	float gz = gyr[2] - gzb;

	//3. Construct State W matrix
	//Sensor frame reffered to the earth frame
	double W[16] = {
					  0, -gx, -gy, -gz,
					  gx,  0,  gz, -gy,
					  gy, -gz,  0,  gx, 				//2-D matrix written in the form of 1-D matrix
					  gz,  gy, -gx,  0
					}; // 4x4 Angular velocity (w)

	Mat * Wm = newmat_up(4, 4, W);

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
	ekf->x[0] *= norm; ekf->x[1] *= norm; ekf->x[2] *= norm;ekf->x[3] *= norm;

	//Re-extract
	q0 = ekf->x[0];
	q1 = ekf->x[1];
	q2 = ekf->x[2];
	q3 = ekf->x[3];

	//State transistion matrix
	double ST[12] = {
					   q1,  q2,  q3,
					  -q0,  q3, -q2,
					  -q3, -q0,  q1,
					   q2, -q1, -q0
					 };//Convert them to actual matrix

	Mat* Sm = newmat_up(4, 3, ST);

	//7. Linearize the model, compute jacobians
	//Horizontal concat and then later vertical concat
	Mat* WST = hconcat(Wm, Sm);   // 4x7
	//Compute the jacobian
	Mat* Ak = vconcat(WST, zeros(3, 7)); // 7x7 Matrix is initialized

	//Linearized state transistion model
	Mat *F = sum(eye(7), scalermultiply(Ak, 0.5 * Ts)); // Diagonal identity matrix

	//P -> 7x7 estimated error covarinace matrix
	//Clean up the code later. Matrix lib starts here.
	ekf->Pi = sum(multiply(F, multiply(ekf->Pi, transpose(F))), ekf->Qi);

}
void EKF_Update(EKF_Handle_t *ekf, float *acc, float *mag)
{
	/*
	 * Fusion Block
	 * 1. (ACCEL/MAG)inits(Measurement)
	 * 2. Construct the observation matrix
	 * 3. Linearize the model
	 * 4. Compute the Kalman gain
	 * 5. Update the state estimate
	 * 6. Update the Predicted error covariance matrix
	 */
	//1. (ACCEL/MAG)inits(Measurement)
	float norm = invSqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
	double ax = acc[0] *norm;
	double ay = acc[1] *norm;
	double az = acc[2] *norm;
	double accel[3] = {ax,ay,az};

	Mat *Z = newmat_up(3, 1, accel);

	//2. Construct the observation matrix
	float q0 = ekf->x[0];
	float q1 = ekf->x[1];
	float q2 = ekf->x[2];
	float q3 = ekf->x[3];
	//Non-linear
	double Hk[12] = {
						  q2, -q3,  q0, -q1,
						 -q1, -q0, -q3, -q2,
						 -q0,  q1,  q2, -q3
				     };
	//3. Linearize the model
	//Linearized by finding the jacobian
	Mat *Hm = newmat_up(3, 4, Hk);
	Hm = scalermultiply(Hm, 2);

	Mat *H = hconcat(Hm, zeros(3, 3)); // 3x7 State Observation matrix

	//4. Compute the Kalman gain
	//Find the inverse of the matrix
	Mat *Sp = sum(multiply(H, multiply(ekf->Pi, transpose(H))), ekf->Ri);

	Mat *K = multiply(ekf->Pi, multiply(transpose(H), inverse(Sp)));

	//* 5. Jacobian of the observation model
	double hk[3] = {
					-2*(q1 * q3 - q0 * q2),

					-2*(q0 * q1 + q2 * q3),

					-1*(q0*q0 + q3*q3 - q1*q1 - q2*q2)

				   };
	Mat *hm = newmat_up(3, 1, hk);

	//Error variance
	Mat *S = minus(Z, hm);
	//Predicted state matrix
	Mat *K_S = multiply(K, S);

	ekf->x[0] = ekf->x[0] + get(K_S, 1, 1);
	ekf->x[1] = ekf->x[1] + get(K_S, 2, 1);
	ekf->x[2] = ekf->x[2] + get(K_S, 3, 1);
	ekf->x[3] = ekf->x[3] + get(K_S, 4, 1);
	ekf->x[4] = ekf->x[4] + get(K_S, 5, 1);
	ekf->x[5] = ekf->x[5] + get(K_S, 6, 1);
	ekf->x[6] = ekf->x[6] + get(K_S, 7, 1);

	//Normalize
	norm = invSqrt(ekf->x[0] * ekf->x[0] + ekf->x[1] * ekf->x[1] + ekf->x[2] * ekf->x[2] + ekf->x[3] * ekf->x[3]);
	ekf->x[0] *= norm; ekf->x[1] *= norm; ekf->x[2] *= norm;ekf->x[3] *= norm;

    //* 6. Update the Predicted error covariance matrix
	Mat *aux = minus(eye(7), multiply(K, H));
	ekf->Pi = multiply(aux, ekf->Pi);


}


void EKF_Extract(EKF_Handle_t *ekf)
{
	/*
	 * Extraction block
	 * 1. Extract the data from state estimates and convert quaternion to euler angles
	 * 2. Update the euler roll pitch or yaw.
	 */
	quat2euler(ekf);
}

///////////////////////////////////////////////////////////////////{HELPER_FUNCTIONS}////////////////////////////////////////////////////////////////////////////////
static void euler2quat(EKF_Handle_t *ekf, float roll, float pitch, float yaw)
{
	double croll = cos(roll/2);
	double sroll = sin(roll/2);

	double cpitch = cos(pitch/2);
	double spitch = sin(pitch/2);

	double cyaw = cos(yaw/2);
	double syaw = sin(yaw/2);

	ekf->x[0] = croll * cpitch * cyaw + sroll * spitch * syaw;
	ekf->x[1] = sroll * cpitch * cyaw - croll * spitch * syaw;
	ekf->x[2] = croll * spitch * cyaw + sroll * cpitch * syaw;
	ekf->x[3] = croll * cpitch * syaw - sroll * spitch * cyaw;
	ekf->x[4] = 0.0; ekf->x[5] = 0.0; ekf->x[6] = 0.0;

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
					   { 2 * (q[1] * q[3]-q[0] * q[2]),          2 * (q[2] * q[3]+q[0] * q[1]),  1 - 2 * (q[2] * q[1]+q[2] * q[2])}
			       };

   ekf->roll  = (atan2(R[2][1], R[2][2])) * Deg2Rad;
   ekf->pitch = -asin(R[2][0]) * Deg2Rad;
   ekf->yaw   = atan2(R[1][0], R[0][0]) * Deg2Rad;

}

static float yaw_correction(float roll, float pitch, float *mag)
{

	float mx = mag[0];
	float my = mag[1];
	float mz = mag[2];

	float norm = invSqrt(mx * mx+ my * my + mz * mz);
	mx *= norm; my *= norm; mz *= norm;

	mx = mx * cos(pitch) + my * sin(roll) * sin(pitch) + mz * cos(roll) * sin(pitch);
	my = my * cos(roll) - mz * sin(roll);

	float yaw = atan2(-my, mx);

	if(yaw < 0) yaw += 2*PI;

	return yaw;

}

static void get_Acc_Mag(EKF_Handle_t *ekf, float *accel_data, float *mag_data)
{
	float ax = accel_data[0];
	float ay = accel_data[1];
	float az = accel_data[2];

	float norm = invSqrt(ax * ax + ay * ay + az * az);
	ax *= norm; ay *= norm; az *= norm;

	ekf->roll = atan2(-ay, -az);
	ekf->pitch = atan2(ax, sqrtf(ay*ay + az*az));
	ekf->yaw = yaw_correction(ekf->roll, ekf->pitch, mag_data);

}


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
