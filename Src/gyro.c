/*
 * gyro.c
 *
 *  Created on: 05.06.2019
 *      Author: Hubert
 */
#include "gyro.h"


extern SPI_HandleTypeDef hspi2;
int Gyro_Init(gyro_Scale s){
	  uint8_t comm;
	  uint8_t resp;
	  uint8_t data;
	  uint8_t adr;

	  HAL_GPIO_WritePin(GYRO_CS_GPIO_Port,GYRO_CS_Pin,1);
	  HAL_GPIO_WritePin(XL_CS_GPIO_Port,XL_CS_Pin,1);
	  HAL_GPIO_WritePin(MAG_CS_GPIO_Port,MAG_CS_Pin,1);

	  ///REG1
	  	  data=0xFF;
	  	  adr=GYRO_REG_CTRL_REG1;
	  	  HAL_GPIO_WritePin(GYRO_CS_GPIO_Port,GYRO_CS_Pin,0);
	  	  HAL_SPI_Transmit(&hspi2, &adr, 1, 500);
	  	  HAL_SPI_Transmit(&hspi2, &data, 1, 500);
	  	  HAL_GPIO_WritePin(GYRO_CS_GPIO_Port,GYRO_CS_Pin,1);


	  ///REG4
	  	  	  	if(s == Scale_250){
	  		  data = 0x01;
	  	  }else if(s == Scale_500){
	  		  data = 0x11;
	  	  }else if(s == Scale_2000){
	  		  data = 0x21;
	  	  }
	  	  adr=GYRO_REG_CTRL_REG4;
	  	  HAL_GPIO_WritePin(GYRO_CS_GPIO_Port,GYRO_CS_Pin,0);
	  	  HAL_SPI_Transmit(&hspi2, &adr, 1, 500);
	  	  HAL_SPI_Transmit(&hspi2, &data, 1, 500);
	  	  HAL_GPIO_WritePin(GYRO_CS_GPIO_Port,GYRO_CS_Pin,1);

	  //HI-PASS filter
	  	  ///REG2
	  	  data=0x01;
	  	  adr=GYRO_REG_CTRL_REG2;
	  	  HAL_GPIO_WritePin(GYRO_CS_GPIO_Port,GYRO_CS_Pin,0);
	  	  HAL_SPI_Transmit(&hspi2, &adr, 1, 500);
	  	  HAL_SPI_Transmit(&hspi2, &data, 1, 500);
	  	  HAL_GPIO_WritePin(GYRO_CS_GPIO_Port,GYRO_CS_Pin,1);
	  	  ///REG5
	  	  data=0x10;
	  	  adr=GYRO_REG_CTRL_REG5;
	  	  HAL_GPIO_WritePin(GYRO_CS_GPIO_Port,GYRO_CS_Pin,0);
	  	  HAL_SPI_Transmit(&hspi2, &adr, 1, 500);
	  	  HAL_SPI_Transmit(&hspi2, &data, 1, 500);
	  	  HAL_GPIO_WritePin(GYRO_CS_GPIO_Port,GYRO_CS_Pin,1);


	  HAL_GPIO_WritePin(GYRO_CS_GPIO_Port,GYRO_CS_Pin,0);
	  resp=0;
	  comm = GYRO_REG_WHO_AM_I | 0b10000000;  // MSB bit = 1 to read
	  HAL_SPI_Transmit(&hspi2, &comm, 1, 500);
	  while(HAL_SPI_Receive(&hspi2, &resp, 1, 500)!=HAL_OK);
	  HAL_GPIO_WritePin(GYRO_CS_GPIO_Port,GYRO_CS_Pin,1);

	  //if(resp==GYRO_WHO_AM_I)
		//  return 0;
	  return resp;
}
int Gyro_Read(gyro* data){
	  float tmp, sens;
	  uint8_t axis[6];
	  uint8_t comm;
	  int16_t out[3];
	  HAL_GPIO_WritePin(GYRO_CS_GPIO_Port,GYRO_CS_Pin,0);
	  comm = GYRO_REG_OUT_X_L | 0b11000000;  // MSB bit = 1 to read, 1 = increment adress
	  HAL_SPI_Transmit(&hspi2, &comm, 1, 50);
	  while(HAL_SPI_Receive(&hspi2, axis, 6, 50)!=HAL_OK);
	  HAL_GPIO_WritePin(GYRO_CS_GPIO_Port,GYRO_CS_Pin,1);

	  out[0]=(axis[1]<<8)+axis[0];
	  out[1]=(axis[3]<<8)+axis[2];
	  out[2]=(axis[5]<<8)+axis[4];

	  	  	  if(GYRO_SENSITIVITY == Scale_250){
	  	  		  sens = GYRO_SENSITIVITY_250 * 0.001;
	  }else   if(GYRO_SENSITIVITY == Scale_500){
		  	  	  sens = GYRO_SENSITIVITY_500 * 0.001;
	  }else   if(GYRO_SENSITIVITY == Scale_2000){
		  	  	  sens = GYRO_SENSITIVITY_2000 * 0.001;
	  }

	  tmp = (float)out[0] * sens;
	  data->X = (int16_t)tmp;
	  tmp = (float)out[1] * sens;
	  data->Y = (int16_t)tmp;
	  tmp = (float)out[2] * sens;
	  data->Z = (int16_t)tmp;

	  return 0;
}
