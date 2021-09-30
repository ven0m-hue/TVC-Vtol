/*
 * MPU9250.h
 *
 *  Created on: 26-Jul-2021
 *      Author: Venom
 */

#ifndef INC_MPU9250_H_
#define INC_MPU9250_H_

#include "main.h"   // While transporting the file remember to change the header directory and neccessary declarations

#if 0
typedef struct
{
	I2C_TypeDef *I2Chandle; /* Handle to the I2C peripheral */
	GPIO_InitTypeDef *GPIO_INT_PIN; /* Handle to configure the Alternate Functionality of the GPIO pins*/

}MPU_Config_t;
#endif

typedef struct
{
	I2C_HandleTypeDef *I2Chandle; /* Handle to the I2C peripheral */
	GPIO_InitTypeDef *GPIO_INT_PIN; /* Handle to configure the Alternate Functionality of the GPIO pins*/

	float acc[3];    /*3 axis accel OutPut*/
	float gyr[3];	 /*3 axis gyro OutPut*/
	float mag[3];	 /*3 axis mag OutPut*/

}MPU9250_Handle_t;


/*API calls*/
//Inits
uint8_t MPU9250_init(MPU9250_Handle_t *imu);
//Reads
void MPU9250_ReadAccel(MPU9250_Handle_t *imu);
void MPU9250_ReadGyro(MPU9250_Handle_t *imu);
void MPU9250_ReadMag(MPU9250_Handle_t *imu);
//Reset
void MPU9250_Reset();

//Global Helper Function
float getGres(uint8_t Gscale);
float getAres(uint8_t Ascale);
float getMres(uint8_t Mscale);

/*
 * Register defination
 */

//Magnetometer Registers
#define AK8963_ADDRESS   0x0C << 1
#define AK8963_WHO_AM_I  0x00 // should return 0x48
#define AK8963_INFO      0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L    0x03  // data
#define AK8963_XOUT_H    0x04
#define AK8963_YOUT_L    0x05
#define AK8963_YOUT_H    0x06
#define AK8963_ZOUT_L    0x07
#define AK8963_ZOUT_H    0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E
#define WOM_THR          0x1F

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23

//I2C Configurations
#define I2C_MST_CTRL     0x24
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36

//Interrupt Control
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A

//Accelerometer
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40

//Temperature
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42

//Gyroscope
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48

//I2C slave
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67

#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP

//Power Managment
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C

//FIFO's
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74

//Device ADDR register
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71

//Accel Calibration
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E

//Device address value
#define ADO 0
#if ADO
#define MPU9250_ADDRESS 0x69<<1  // Device address when ADO = 1
#else
#define MPU9250_ADDRESS 0xD0//(0x68 << 1)  // Device address when ADO = 0
#endif

#define MPU9250_I2C_TIMEOUT 100

//Math Constants


//Enums to select the scale and range
enum Ascale
{
	AFS_2G = 0, AFS_4G, AFS_8G, AFS_16G
};

enum Gscale
{
	GFS_250DPS = 0, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
};

enum Mscale
{
	MFS_14BITS = 0, MFS_16BITS
};

/*Scale selection Macros*/


#endif /* INC_MPU9250_H_ */
