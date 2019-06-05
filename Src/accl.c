/*
 * accl.c
 *
 *  Created on: 05.06.2019
 *      Author: Hubert
 */
#include "accl.h"

extern SPI_HandleTypeDef hspi2;
int mag_Init(){
	  uint8_t comm;
	  uint8_t resp;
	  uint8_t data;
	  uint8_t adr;

	  HAL_GPIO_WritePin(GYRO_CS_GPIO_Port,GYRO_CS_Pin,1);
	  ACCL_CS_HIGH;
	  MAG_CS_HIGH;


	  ///REG1
	  	  data=0b11111100;
	  	  adr=ACCL_REG_CTRL_REG1;
	  	  ACCL_CS_LOW;
	  	  HAL_SPI_Transmit(&hspi2, &adr, 1, 50);
	  	  HAL_SPI_Transmit(&hspi2, &data, 1, 50);
	  	  ACCL_CS_HIGH;

	///REG2
		  data=0b01100000;
	  	  adr=ACCL_REG_CTRL_REG2;
		  ACCL_CS_LOW;
		  HAL_SPI_Transmit(&hspi2, &adr, 1, 50);
		  HAL_SPI_Transmit(&hspi2, &data, 1, 50);
		  ACCL_CS_HIGH;


	///REG3
		  data=0b10000100;
	  	  adr=ACCL_REG_CTRL_REG3;
		  ACCL_CS_LOW;
		  HAL_SPI_Transmit(&hspi2, &adr, 1, 50);
		  HAL_SPI_Transmit(&hspi2, &data, 1, 50);
		  ACCL_CS_HIGH;

	  ///REG4
	  	  data=0b00001100;
	  	  adr=ACCL_REG_CTRL_REG4;
	  	  ACCL_CS_LOW;
	  	  HAL_SPI_Transmit(&hspi2, &adr, 1, 50);
	  	  HAL_SPI_Transmit(&hspi2, &data, 1, 50);
	  	  ACCL_CS_HIGH;

	  ACCL_CS_LOW;
	  comm = ACCL_REG_WHO_AM_I | 0x80;  // MSB bit = 1 to read
	  HAL_SPI_Transmit(&hspi2, &comm, 1, 50);
	  HAL_SPI_Receive(&hspi2, &resp, 1, 50);
	  ACCL_CS_HIGH;

	 // if(resp==MAG_WHO_AM_I)
		//  return 0;
	  return resp;
}
int mag_Read(mag* data){
	  uint8_t axis[6];
	  uint8_t comm;
	  int16_t out[3];
	  HAL_GPIO_WritePin(GYRO_CS_GPIO_Port,GYRO_CS_Pin,0);
	  comm = ACCL_REG_OUT_X_L | 0b11000000;  // MSB bit = 1 to read, 1 = increment adress
	  HAL_SPI_Transmit(&hspi2, &comm, 1, 50);
	  HAL_SPI_Receive(&hspi2, axis, 6, 50);
	  HAL_GPIO_WritePin(GYRO_CS_GPIO_Port,GYRO_CS_Pin,1);

	  out[0]=(axis[1]<<8)+axis[0];
	  out[1]=(axis[3]<<8)+axis[2];
	  out[2]=(axis[5]<<8)+axis[4];
	  data->X = out[0];
	  data->Y = out[1];
	  data->Z = out[2];

	  return 0;
}


int accl_Init(accl_Scale s){
	  uint8_t comm;
	  uint8_t resp;
	  uint8_t data;
	  uint8_t adr;

	  HAL_GPIO_WritePin(GYRO_CS_GPIO_Port,GYRO_CS_Pin,1);
	  ACCL_CS_HIGH;
	  MAG_CS_HIGH;


	  ///REG1
	  	  data=0b01100111;
	  	  adr=ACCL_REG_CTRL_REG1;
	  	  ACCL_CS_LOW;
	  	  HAL_SPI_Transmit(&hspi2, &adr, 1, 50);
	  	  HAL_SPI_Transmit(&hspi2, &data, 1, 50);
	  	  ACCL_CS_HIGH;
	///REG2
		  data=0b01100100;
		  adr=ACCL_REG_CTRL_REG2;
		  ACCL_CS_LOW;
		  HAL_SPI_Transmit(&hspi2, &adr, 1, 50);
		  HAL_SPI_Transmit(&hspi2, &data, 1, 50);
		  ACCL_CS_HIGH;

	  ///REG4
	  	  	    if(s==Scale_2g){
	  	  	    	data=0b11000011;
	  	  }else if(s==Scale_4g){
	  		  		data=0b11100011;
	  	  }else if(s==Scale_8g){
	  		  	  	data=0b11110011;
	  	  }
	  	  adr=ACCL_REG_CTRL_REG4;
	  	  ACCL_CS_LOW;
	  	  HAL_SPI_Transmit(&hspi2, &adr, 1, 50);
	  	  HAL_SPI_Transmit(&hspi2, &data, 1, 50);
	  	  ACCL_CS_HIGH;


	  ACCL_CS_LOW;
	  comm = ACCL_REG_WHO_AM_I | 0x80;  // MSB bit = 1 to read
	  HAL_SPI_Transmit(&hspi2, &comm, 1, 50);
	  HAL_SPI_Receive(&hspi2, &resp, 1, 50);
	  ACCL_CS_HIGH;

	  //if(resp==ACCL_WHO_AM_I)
		//  return 0;
	  return resp;
}
int accl_Read(accl* data){
	  float tmp, sens;
	  uint8_t axis[6];
	  uint8_t comm;
	  int16_t out[3];
	  HAL_GPIO_WritePin(GYRO_CS_GPIO_Port,GYRO_CS_Pin,0);
	  comm = ACCL_REG_OUT_X_L | 0b10000000;  // MSB bit = 1 to read
	  HAL_SPI_Transmit(&hspi2, &comm, 1, 50);
	  HAL_SPI_Receive(&hspi2, &axis[0], 1, 50);
	  comm = ACCL_REG_OUT_X_H | 0b10000000;  // MSB bit = 1 to read
	  HAL_SPI_Transmit(&hspi2, &comm, 1, 50);
	  HAL_SPI_Receive(&hspi2, &axis[1], 1, 50);

	  comm = ACCL_REG_OUT_Y_L | 0b10000000;  // MSB bit = 1 to read
	  HAL_SPI_Transmit(&hspi2, &comm, 1, 50);
	  HAL_SPI_Receive(&hspi2, &axis[2], 1, 50);
	  comm = ACCL_REG_OUT_Y_H | 0b10000000;  // MSB bit = 1 to read
	  HAL_SPI_Transmit(&hspi2, &comm, 1, 50);
	  HAL_SPI_Receive(&hspi2, &axis[3], 1, 50);

	  comm = ACCL_REG_OUT_Z_L | 0b10000000;  // MSB bit = 1 to read
	  HAL_SPI_Transmit(&hspi2, &comm, 1, 50);
	  HAL_SPI_Receive(&hspi2, &axis[4], 1, 50);
	  comm = ACCL_REG_OUT_Z_H | 0b10000000;  // MSB bit = 1 to read
	  HAL_SPI_Transmit(&hspi2, &comm, 1, 50);
	  HAL_SPI_Receive(&hspi2, &axis[5], 1, 50);
	  HAL_GPIO_WritePin(GYRO_CS_GPIO_Port,GYRO_CS_Pin,1);

	  out[0]=(axis[1]<<8)+axis[0];
	  out[1]=(axis[3]<<8)+axis[2];
	  out[2]=(axis[5]<<8)+axis[4];

	  	  	  if(ACCL_SENSITIVITY == Scale_2g){
	  	  		  sens = ACCL_SENSITIVITY_2g ;
	  }else   if(ACCL_SENSITIVITY == Scale_4g){
		  	  	  sens = ACCL_SENSITIVITY_4g ;
	  }else   if(ACCL_SENSITIVITY == Scale_8g){
		  	  	  sens = ACCL_SENSITIVITY_8g ;
	  }

	  tmp = (float)out[0] * sens;
	  data->X = (int16_t)tmp;
	  tmp = (float)out[1] * sens;
	  data->Y = (int16_t)tmp;
	  tmp = (float)out[2] * sens;
	  data->Z = (int16_t)tmp;

	  return 0;
}
