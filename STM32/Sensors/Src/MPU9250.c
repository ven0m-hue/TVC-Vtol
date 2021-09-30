/*
 * MPU9250.c
 *
 *  Created on: 26-Jul-2021
 *      Author: Venom
 */
/*
 *
//To DO's
// give #Defines for more readability
// Define scales for each of the sensors(revist)
// Accel (Roll, Pitch) offset function --> the one implemented in the python script.
// Refer mag registers

//Future
// Mag scale factor revist
// Mag  and Accel Compensation
*/
#include "MPU9250.h"

#define PI 3.142857f
#define g  9.80665f
#define Deg2Rad 180/ PI
#define Rad2Deg PI/ 180
static void AK8963_init(I2C_HandleTypeDef *I2Chandle);

static void writeByte(I2C_HandleTypeDef *I2Chandle, uint8_t Address, uint8_t subAddress, uint8_t data);
static uint8_t readByte(I2C_HandleTypeDef *I2Chandle, uint8_t Address, uint8_t subAddress);

//Default Selection, overided with the enum scale selection
static uint8_t Ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
static uint8_t Gscale = GFS_250DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
static uint8_t Mscale = MFS_16BITS; // MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution
static uint8_t Mmode = 0x06;        // Either 8 Hz 0x02) or 100 Hz (0x06) magnetometer data ODR

float mRes, gRes, aRes;

uint8_t MPU9250_init(MPU9250_Handle_t *imu)
{

	/* 1.Reset all the sensors*/
	writeByte(imu->I2Chandle, MPU9250_ADDRESS, PWR_MGMT_1, 0x00);
	HAL_Delay(10); // 1ms delay

	/*2. Power management and Crystal clock settings*/
	writeByte(imu->I2Chandle, MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
	/*3. Configure the accel and gyro
	 *   Set the sample rate and bandwidth  as 1KHz and 42 Hz @refer_datasheet pg 15
	 *   DLPF_CFG = b'11
	 *   Sample_rate = gyro_output_rate/(1 + SMPLRT_DIV)
	 */
	writeByte(imu->I2Chandle, MPU9250_ADDRESS, CONFIG, 0x03);
    // Using the 200Hz rate -> From the above calculation.
	writeByte(imu->I2Chandle, MPU9250_ADDRESS, SMPLRT_DIV, 0x04);

	/*4.Gyro Scale selection
	 * 1. Clear the Gyro_config register
	 * 2. Set the Fchoice = b'11 aka f_choice_b = b'00 and clear the GFS
	 * 3. Select the gyro scale
	 *
	 * 5. Repeat the same sequence for the accel scale selection
	 */
	uint8_t rxData;

	//Gyro
	rxData = readByte(imu->I2Chandle, MPU9250_ADDRESS, GYRO_CONFIG);
	rxData &= ~(0x02); //celars fchoice
	rxData &= ~(0x18); //celars GFS
	rxData |= Gscale << 3;
	writeByte(imu->I2Chandle, MPU9250_ADDRESS, GYRO_CONFIG, rxData);

	//Accel
	rxData = readByte(imu->I2Chandle, MPU9250_ADDRESS, ACCEL_CONFIG);
	rxData &= ~(0x18); //celars GFS
	rxData |= Ascale << 3;
	writeByte(imu->I2Chandle, MPU9250_ADDRESS, ACCEL_CONFIG, rxData);

	/*
	 * Set accel sample rate @4KHz refer data_sheet pg 17
	 * Bw = 41Hz
	 */
	rxData = readByte(imu->I2Chandle, MPU9250_ADDRESS, ACCEL_CONFIG2);
	rxData &= ~(0x0F);
	rxData |= 0x03;
	writeByte(imu->I2Chandle, MPU9250_ADDRESS, ACCEL_CONFIG2, rxData);

	//Originally all the sensor are set to 1KHz, however refacotred using SMPLRT_DIV to 200hz

	/*6.Configure the interrupt pins
	 * Interrupt to rasing edge and clears on read
	 * @refer reference manual  pg 29
	 */
	writeByte(imu->I2Chandle, MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
	writeByte(imu->I2Chandle, MPU9250_ADDRESS, INT_ENABLE, 0x01);

	/*7.Configure the magnetometer*/
	AK8963_init(imu->I2Chandle);

	return 1;

}


void MPU9250_ReadAccel(MPU9250_Handle_t *imu)
{
	uint8_t rawdata[6];
	HAL_I2C_Mem_Read(imu->I2Chandle, MPU9250_ADDRESS, ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, rawdata, 6, MPU9250_I2C_TIMEOUT);

	int16_t accX = (int16_t)((int16_t)rawdata[0] << 8 | rawdata[1]);
	int16_t accY = (int16_t)((int16_t)rawdata[2] << 8 | rawdata[3]);
	int16_t accZ = (int16_t)((int16_t)rawdata[4] << 8 | rawdata[5]);

	imu->acc[0] =  accX * getAres(Ascale);
	imu->acc[1] =  accY * getAres(Ascale);
	imu->acc[2] = -accZ * getAres(Ascale);

}

void MPU9250_ReadGyro(MPU9250_Handle_t *imu)
{
	uint8_t rawdata[6];
	HAL_I2C_Mem_Read(imu->I2Chandle, MPU9250_ADDRESS, GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, rawdata, 6, MPU9250_I2C_TIMEOUT);

	int16_t gyrX = (int16_t)((int16_t)rawdata[0] << 8 | rawdata[1]);
	int16_t gyrY = (int16_t)((int16_t)rawdata[2] << 8 | rawdata[3]);
	int16_t gyrZ = (int16_t)((int16_t)rawdata[4] << 8 | rawdata[5]);

	imu->gyr[0] =  gyrX * getGres(Gscale);
	imu->gyr[1] =  gyrY * getGres(Gscale);
	imu->gyr[2] =  gyrZ * getGres(Gscale);
}

void MPU9250_ReadMag(MPU9250_Handle_t *imu)
{
	uint8_t rawdata[6];
	/*Wait for Mag to be ready*/
	if(readByte(imu->I2Chandle, MPU9250_ADDRESS, AK8963_ST1) & 0x01)
	{
		HAL_I2C_Mem_Read(imu->I2Chandle, MPU9250_ADDRESS, AK8963_XOUT_L, I2C_MEMADD_SIZE_8BIT, rawdata, 3, MPU9250_I2C_TIMEOUT);
		/*Check the Overflow flag in the SR of AK8963
		 * wait until it gets cleared
		 * refer @ reference manual pg 50*/
		if( readByte(imu->I2Chandle, MPU9250_ADDRESS, AK8963_ST2) & 0x08)
		{

			int16_t magX = (int16_t)((int16_t)rawdata[1] << 8 | rawdata[0]);
			int16_t magY = (int16_t)((int16_t)rawdata[3] << 8 | rawdata[2]);
			int16_t magZ = (int16_t)((int16_t)rawdata[5] << 8 | rawdata[4]);

			imu->mag[0] =  magX * getMres(Mscale) * 0;
			imu->mag[1] =  magY * getMres(Mscale) * 0;
			imu->mag[2] =  magZ * getMres(Mscale) * 0;

		}
	}

}

void MPU9250_Reset(MPU9250_Handle_t *imu)
{
	/* Setting the bits of PWR_MGMT register will reset the sensors*/
	writeByte(imu->I2Chandle, MPU9250_ADDRESS, PWR_MGMT_1, 0x80);
	HAL_Delay(1);
}



////////////////////////////////////////////////////////////////////////{HELPER_FUNCTIONS}////////////////////////////////////////////////////////////////////////////

float getMres(uint8_t Mscale) {
  switch (Mscale)
  {

    /*
     * Possible magnetometer scales (and their register bit settings) are:
     * Mmode = 14 bit resolution (0) and 16 bit resolution (1)
     */

    case MFS_14BITS:
          mRes = 10.0*4912.0/8190.0; // Proper scale to return milliGauss
          break;
    case MFS_16BITS:
          mRes = 10.0*4912.0/32760.0; // Proper scale to return milliGauss
          break;
  }

  return mRes;
}



float getGres(uint8_t Gscale) {
  switch (Gscale)
  {
    /*
     *  Possible gyro scales (and their register bit settings) are:
     *  250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
     *  From the data sheet, divide the raw value with the accurate scale factors to get the Wx, Wy, Wz
     *  @refer datasheet pg 8
     *	The output unit is deg/sec
     *  To get the output in rad/sec for future computation convert deg/s -> rad/s
     */

    case GFS_250DPS:
          gRes = PI/ ( 180 * 131);
          break;
    case GFS_500DPS:
          gRes = PI/ (180 * 65.5);
          break;
    case GFS_1000DPS:
          gRes = PI/ (180 * 32.8);
          break;
    case GFS_2000DPS:
          gRes = PI/ (180 * 16.4);
          break;
  }

  return gRes;

}


float getAres(uint8_t Ascale) {
  switch (Ascale)
  {
    /*
     * Possible accelerometer scales (and their register bit settings) are:
     * 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
     * From the data sheet, divide the raw value with the accurate scale factors to get the Ax, Ay, Az
     * @refer datasheet pg 9
     *
     * Convert the acceleration value interms of acceleration due to gravity to make sense of the data.
     */

    case AFS_2G:
          aRes = g/16384.0;
          break;
    case AFS_4G:
          aRes = g/8192;
          break;
    case AFS_8G:
          aRes = g/4096;
          break;
    case AFS_16G:
          aRes = g/2048;
          break;
  }

  return aRes;
}

static void writeByte(I2C_HandleTypeDef *I2Chandle, uint8_t Address, uint8_t subAddress, uint8_t data)
{

	 uint8_t txData[] = {subAddress, data};
	 HAL_I2C_Master_Transmit(I2Chandle, MPU9250_ADDRESS, txData, 2, MPU9250_I2C_TIMEOUT);

}

static uint8_t readByte(I2C_HandleTypeDef *I2Chandle, uint8_t Address, uint8_t subAddress)
{
	uint8_t rxData[1];
	uint8_t txData[] = {subAddress};
	HAL_I2C_Master_Transmit(I2Chandle, MPU9250_ADDRESS, txData, 1, MPU9250_I2C_TIMEOUT);

	HAL_I2C_Master_Receive(I2Chandle, MPU9250_ADDRESS, rxData, 1, MPU9250_I2C_TIMEOUT);

	return rxData[0];

}

static void AK8963_init(I2C_HandleTypeDef *I2Chandle)
{
	/*1.Reset the Mag sensor*/
	writeByte(I2Chandle, MPU9250_ADDRESS, AK8963_CNTL, 0x00);
	HAL_Delay(1);
	/*2.Fuse rom access mode*/
	writeByte(I2Chandle, MPU9250_ADDRESS, AK8963_CNTL, 0x0F);
	HAL_Delay(1);
	/*3.Power doen Magnetometer*/
	writeByte(I2Chandle, MPU9250_ADDRESS, AK8963_CNTL, 0x00);
	HAL_Delay(1);
	/*4.Mscale enable the 16bit resolution mode
	 *  Enable continous mode data acquisition Mmode = b'0110 @refer data sheet
	 */
	writeByte(I2Chandle, MPU9250_ADDRESS, AK8963_CNTL, Mscale << (4 | Mmode));
	HAL_Delay(1);

}


