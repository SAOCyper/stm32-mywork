/*
 * Mpu6050.c
 *
 *  Created on: Jun 23, 2022
 *      Author: Gesislab
 */

#include "string.h"
#include "main.h"
#include "motion_controller.h"
#include "stdlib.h"
#include "Mpu6050.h"

extern  I2C_HandleTypeDef hi2c2;

uint32_t old;
uint32_t ellapsed_time;
uint32_t now;
uint32_t i=0;
uint8_t data=0;
uint8_t check=0;
uint8_t Data=0;
uint8_t Acc_Data[6];
uint8_t Gyr_Data[6];

int16_t Accel_X_RAW;
int16_t Accel_Y_RAW;
int16_t Accel_Z_RAW;
int16_t Gyro_X_RAW;
int16_t Gyro_Y_RAW;
int16_t Gyro_Z_RAW;

float Gxt,Gyt,Gzt,Axt,Ayt,Azt,Gx_,Gy_,Gz_,Ax_,Ay_,Az_;
float Ax,Ay,Az,Gx,Gy,Gz,Gz1;

struct Mpu6050_Config_Tag
{
  uint8_t I2cAddrs;
  Mpu6050_SpeedScale_e AccelFullScale;
  Mpu6050_AngleScale_e AngleFullScale;

};

Mpu6050_Config_t * Mpu6050_init(uint8_t addrs, Mpu6050_SpeedScale_e accelfullscale,Mpu6050_AngleScale_e anglefullscale)
{
  Mpu6050_Config_t *pConfig = malloc(sizeof(Mpu6050_Config_t));
  pConfig->I2cAddrs = addrs;
  pConfig->AccelFullScale = accelfullscale;
  pConfig->AngleFullScale = anglefullscale;

  return pConfig;
}

typedef struct{
	int16_t gx[1000];
	int16_t gy[1000];
	int16_t gz[1000];
	uint32_t counter;
}motion_t;

motion_t motion_rec = {.counter = 0};


uint32_t get_elapsed_time(){
	  old = now;
	  now = HAL_GetTick();
	  ellapsed_time = (now - old)/1000;

}

void Mpu6050_updateMpu6050Data()
{
	/*get_elapsed_time();*/
	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, GYRO_XOUT_H_REG, 1,Gyr_Data, 6, 1000);
    Gyro_X_RAW = (int16_t)(Gyr_Data[0] << 8 | Gyr_Data [1]);
	Gyro_Y_RAW = (int16_t)(Gyr_Data[2] << 8 | Gyr_Data [3]);
    Gyro_Z_RAW = (int16_t)(Gyr_Data[4] << 8 | Gyr_Data [5]);
    Gx = Gyro_X_RAW/65.5;
    Gy = Gyro_Y_RAW/65.5;
    Gz = Gyro_Z_RAW/65.5;
    /*if(abs(Gz) > 1){
    	Gx_ = (Gx + Gx_) * ellapsed_time;
    	Gy_ = (Gy + Gy_) * ellapsed_time;
    	Gz_ = (Gz + Gz_) * ellapsed_time;
     }*/
    if(Gz>1){
	  /*static uint8_t state=0;*/
	  for(i=0;i<150;i++){

		  HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, GYRO_XOUT_H_REG, 1,Gyr_Data, 6, 1000);
		  Gyro_X_RAW = (int16_t)(Gyr_Data[0] << 8 | Gyr_Data [1]);
		  Gyro_Y_RAW = (int16_t)(Gyr_Data[2] << 8 | Gyr_Data [3]);
		  Gyro_Z_RAW = (int16_t)(Gyr_Data[4] << 8 | Gyr_Data [5]);
		  Gx = Gyro_X_RAW/65.5;
		  Gy = Gyro_Y_RAW/65.5;
		  Gz = Gyro_Z_RAW/65.5;
		  Gxt = Gxt + Gx;
		  Gyt = Gyt + Gy;
		  Gzt = Gzt + Gz;
		  HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1,Acc_Data, 6, 1000);
		  Accel_X_RAW = (int16_t)(Acc_Data[0] << 8 | Acc_Data [1]);
		  Accel_Y_RAW = (int16_t)(Acc_Data[2] << 8 | Acc_Data [3]);
		  Accel_Z_RAW = (int16_t)(Acc_Data[4] << 8 | Acc_Data [5]);
		  Ax = Accel_X_RAW/4099.0;  // get the float g for +-8g sensitivity is 4099 LSB/g
		  Ay = Accel_Y_RAW/4099.0;
		  Az = Accel_Z_RAW/4099.0;
		  Axt=Axt+Ax;
		  Ayt=Ayt+Ay;
		  Azt=Azt+Az;
	  	  }
	  Gx_ = Gxt/150;
	  Gy_ = Gyt/150;
	  Gz_ = Gzt/150;
    }
    else if(Gz < -1){
    	  for(i=0;i<150;i++){

    		  HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, GYRO_XOUT_H_REG, 1,Gyr_Data, 6, 1000);
    		  Gyro_X_RAW = (int16_t)(Gyr_Data[0] << 8 | Gyr_Data [1]);
    		  Gyro_Y_RAW = (int16_t)(Gyr_Data[2] << 8 | Gyr_Data [3]);
    		  Gyro_Z_RAW = (int16_t)(Gyr_Data[4] << 8 | Gyr_Data [5]);
    		  Gx = Gyro_X_RAW/65.5;
    		  Gy = Gyro_Y_RAW/65.5;
    		  Gz = Gyro_Z_RAW/65.5;
    		  Gxt = Gxt + Gx;
    		  Gyt = Gyt + Gy;
    		  Gzt = Gzt + Gz;
    		  HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1,Acc_Data, 6, 1000);
    		  Accel_X_RAW = (int16_t)(Acc_Data[0] << 8 | Acc_Data [1]);
    		  Accel_Y_RAW = (int16_t)(Acc_Data[2] << 8 | Acc_Data [3]);
    		  Accel_Z_RAW = (int16_t)(Acc_Data[4] << 8 | Acc_Data [5]);
    		  Ax = Accel_X_RAW/4099.0;  // get the float g for +-8g sensitivity is 4099 LSB/g
    		  Ay = Accel_Y_RAW/4099.0;
    		  Az = Accel_Z_RAW/4099.0;
    		  Axt=Axt+Ax;
    		  Ayt=Ayt+Ay;
    		  Azt=Azt+Az;
    	  	  }
    	  Gx_ = Gxt/150;
    	  Gy_ = Gyt/150;
    	  Gz_ = Gzt/150;
        }
    else{
    	Mpu6050_Init();
    }
  /*if(motion_rec.counter<1000){
	  motion_rec.gx[motion_rec.counter] = Gyro_X_RAW;
	  motion_rec.gy[motion_rec.counter] = Gyro_Y_RAW;
	  motion_rec.gz[motion_rec.counter] = Gyro_Z_RAW;
	  motion_rec.counter++;
  }
  else{
	  motion_rec.counter = 0;
  }*/



}
void Mpu6050_Init()
{
	HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);
	if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
		{
			// power management register 0X6B we should write all 0's to wake the sensor up
			Data = 0;
			HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000);

			// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
			Data = 0x07;
			HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

			// Set accelerometer configuration in ACCEL_CONFIG Register
			Data=0x10;// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=2 -> � 8g
			HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, ACC_CNFG_REG, 1, &Data, 1, 1000);

			// Set Gyroscopic configuration in GYRO_CONFIG Register
			// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
			Data=0x08;// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=1 -> � 500 �/s
			HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, GYRO_CNFG_REG, 1, &Data, 1, 1000);
		}
}
void Mpu6050_deInit()
{
   uint8_t data;
   //free(pConfig);
   data = 1;
   HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&data, 1, 1000);
}
