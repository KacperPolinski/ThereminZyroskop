/*
 * gyro.h
 *
 *  Created on: 05.06.2019
 *      Author: Hubert
 */
#include "main.h"


#ifndef GYRO_H_
#define GYRO_H_

#ifndef GYRO_CS_PIN
#define GYRO_CS_PORT				GYRO_CS_GPIO_Port
#define GYRO_CS_PIN					GYRO_CS_Pin
#endif

/* Pin macros */
#define GYRO_CS_LOW					HAL_GPIO_WritePin(CS_PORT,CS_PIN,0)
#define GYRO_CS_HIGH				HAL_GPIO_WritePin(CS_PORT,CS_PIN,1)

/* Identification number */
#define GYRO_WHO_AM_I			0xD4

/* Registers addresses */
#define GYRO_REG_WHO_AM_I		0x0F
#define GYRO_REG_CTRL_REG1		0x20
#define GYRO_REG_CTRL_REG2		0x21
#define GYRO_REG_CTRL_REG3		0x22
#define GYRO_REG_CTRL_REG4		0x23
#define GYRO_REG_CTRL_REG5		0x24
#define GYRO_REG_REFERENCE		0x25
#define GYRO_REG_OUT_TEMP		0x26
#define GYRO_REG_STATUS_REG		0x27
#define GYRO_REG_OUT_X_L		0x28
#define GYRO_REG_OUT_X_H		0x29
#define GYRO_REG_OUT_Y_L		0x2A
#define GYRO_REG_OUT_Y_H		0x2B
#define GYRO_REG_OUT_Z_L		0x2C
#define GYRO_REG_OUT_Z_H		0x2D
#define GYRO_REG_FIFO_CTRL_REG	0x2E
#define GYRO_REG_FIFO_SRC_REG	0x2F
#define GYRO_REG_INT1_CFG		0x30
#define GYRO_REG_INT1_SRC		0x31
#define GYRO_REG_INT1_TSH_XH	0x32
#define GYRO_REG_INT1_TSH_XL	0x33
#define GYRO_REG_INT1_TSH_YH	0x34
#define GYRO_REG_INT1_TSH_YL	0x35
#define GYRO_REG_INT1_TSH_ZH	0x36
#define GYRO_REG_INT1_TSH_ZL	0x37
#define GYRO_REG_INT1_DURATION	0x38

typedef enum {
	Scale_250, /*!< Set full scale to 250 mdps */
	Scale_500, /*!< Set full scale to 500 mdps */
	Scale_2000 /*!< Set full scale to 2000 mdps */
} gyro_Scale;

/* Sensitivity factors, datasheet pg. 9 */
#define GYRO_SENSITIVITY			Scale_2000
#define GYRO_SENSITIVITY_250		8.75	/* 8.75 mdps/digit */
#define GYRO_SENSITIVITY_500		17.5	/* 17.5 mdps/digit */
#define GYRO_SENSITIVITY_2000		70.0	/* 70 mdps/digit */

typedef struct {
	int16_t X; /*!< X axis rotation */
	int16_t Y; /*!< Y axis rotation */
	int16_t Z; /*!< Z axis rotation */
} gyro;



int Gyro_Init(gyro_Scale s);
int Gyro_Read(gyro* data);

#endif /* GYRO_H_ */
