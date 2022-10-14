/*
 * Mpu6050.h
 *
 *  Created on: Jun 23, 2022
 *      Author: Gesislab
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#define MPU6050_ADDR 0x68<<1
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define	GYRO_CNFG_REG 0x1B
#define ACC_CNFG_REG 0x1C
#define WHO_AM_I_REG 0x75
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43

typedef enum
{
	Mpu6050_AccelFullScale_2g = 0,
	Mpu6050_AccelFullScale_4g,
	Mpu6050_AccelFullScale_8g,
	Mpu6050_AccelFullScale_16g,
}Mpu6050_SpeedScale_e;

typedef enum
{
	Mpu6050_AngleFullScale_250 = 0,
	Mpu6050_AngleFullScale_500,
	Mpu6050_AngleFullScale_1000,
	Mpu6050_AngleFullScale_2000
}Mpu6050_AngleScale_e;

typedef struct Mpu6050_Config_Tag Mpu6050_Config_t;

typedef struct
{
	uint8_t reserved : 3;
	uint8_t fs_sel : 2;
	uint8_t xg_st : 1;
	uint8_t yg_st : 1;
	uint8_t zg_st : 1;
}Mpu6050_GyroConfigReg_t;

typedef struct
{
	uint8_t reserved : 3;
	uint8_t afs_sel : 2;
	uint8_t x_st : 1;
	uint8_t y_st : 1;
	uint8_t z_st : 1;
}Mpu6050_AccConfigReg_t;

Mpu6050_Config_t * Mpu6050_init(uint8_t addrs, Mpu6050_SpeedScale_e accelfullscale,Mpu6050_AngleScale_e anglefullscale);

void 	Mpu6050_Init				();
void   Mpu6050_updateMpu6050Data 	();
void    Mpu6050_deInit           	();

#endif /* INC_MPU6050_H_ */
