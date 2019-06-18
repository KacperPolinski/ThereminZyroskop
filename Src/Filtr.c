
#include "Filtr.h"
#include <math.h>
#include <stdlib.h>
extern I2C_HandleTypeDef I2c1Handle;
void Init_MPU(){
	uint8_t power = 0;
	uint8_t data[3]={0b00000001,
				     0b00011000,
				     0b00001000};
	HAL_I2C_Mem_Write(&I2c1Handle,GYRO,0x6B,1,&power,1,1000);
    HAL_I2C_Mem_Write(&I2c1Handle,GYRO,0x1A,1,data,3,1000);
    data[0]=0b01111000;
    HAL_I2C_Mem_Write(&I2c1Handle,GYRO,0x23,1,data,1,1000);

}
void Read_MPU_Gyro(){
	uint8_t data[6]={};
	HAL_I2C_Mem_Read(&I2c1Handle,GYRO,0x43,1,data,6,1000);

	gyro[0] = (int16_t)((data[0]<<8) | data[1]);
	gyro[1] = (int16_t)((data[2]<<8) | data[3]);
	gyro[2] = (int16_t)((data[4]<<8) | data[5]);

}
void Read_MPU_Accl(){
	uint8_t data[6]={};
	HAL_I2C_Mem_Read(&I2c1Handle,GYRO,0x3B,1,data,6,1000);

	accel[0] = (int16_t)((data[0]<<8) | data[1]) - ACCEL_X_OFFSET;
	accel[1] = (int16_t)((data[2]<<8) | data[3]) - ACCEL_Y_OFFSET;
	accel[2] = (int16_t)((data[4]<<8) | data[5]) - ACCEL_Z_OFFSET;
}
void Exchange(){
	lacceleration_g[0] = acceleration_g[0];
	lacceleration_g[1] = acceleration_g[1];
	lacceleration_g[2] = acceleration_g[2];
	langular_rate_dps[0] = angular_rate_dps[0];
	langular_rate_dps[1] = angular_rate_dps[1];
	langular_rate_dps[2] = angular_rate_dps[2];

	acceleration_g[0] = (accel[0] / 8192) ;
	acceleration_g[1] = (accel[1] / 8192) ;
	acceleration_g[2] = (accel[2] / 8192) ;

	angular_rate_dps[0] = (gyro[0] /16.4);
	angular_rate_dps[1] = (gyro[1] /16.4);
	angular_rate_dps[2] = (gyro[2] /16.4);
}
void Filter(){

	laccRoll = accRoll;
	laccPitch = accPitch;
	//laccYaw = accYaw;
	accRoll = (1-alpha) * TO_DEG(atan2(acceleration_g[0],sqrt(acceleration_g[0] * acceleration_g[0] + acceleration_g[1] * acceleration_g[1] + acceleration_g[2] * acceleration_g[2]))) + alpha * laccRoll;
	accPitch = (1-alpha) * TO_DEG(atan2(acceleration_g[1],sqrt(acceleration_g[0] * acceleration_g[0] + acceleration_g[1] * acceleration_g[1] + acceleration_g[2] * acceleration_g[2]))) + alpha * laccPitch;
	//accYaw = (1-alpha) * TO_DEG(atan2(acceleration_g[2],sqrt(acceleration_g[0] * acceleration_g[0] + acceleration_g[1] * acceleration_g[1] + acceleration_g[2] * acceleration_g[2]))) + alpha * laccYaw;

	lgyroRoll = gyroRoll;
	lgyroPitch = gyroPitch;
	//lgyroYaw = gyroYaw;
	gyroRoll = (1-alpha) * lgyroRoll + (1-alpha)*(angular_rate_dps[0]-langular_rate_dps[0]);
	gyroPitch =(1-alpha) * lgyroPitch + (1-alpha)*(angular_rate_dps[1]-langular_rate_dps[1]);
	//gyroPitch =(1-alpha) * lgyroYaw + (1-alpha)*(angular_rate_dps[2]-langular_rate_dps[2]);


	timeElapsed = HAL_GetTick() - lastMeasurment;
	lastMeasurment = HAL_GetTick();

	roll_tmp = ((1 - alpha) * (roll_tmp + gyroRoll * timeElapsed / 1000.0) + alpha * accRoll);
	pitch_tmp = ((1 - alpha) * (pitch_tmp + gyroPitch * timeElapsed / 1000.0) + alpha * accPitch);
	//yaw = ((1 - alpha) * (yaw + gyroYaw * timeElapsed / 1000.0) + alpha * accYaw);

	roll = roll_tmp;
	pitch = pitch_tmp;
}
