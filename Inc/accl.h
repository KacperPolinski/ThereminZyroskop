/*
 * accl.h
 *
 *  Created on: 05.06.2019
 *      Author: Hubert
 */
#include "main.h"

#ifndef ACCL_H_
#define ACCL_H_

#ifndef ACCL_CS_PIN
#define ACCL_CS_PORT				XL_CS_GPIO_Port
#define ACCL_CS_PIN					XL_CS_Pin
#endif

#ifndef MAG_CS_PIN
#define MAG_CS_PORT					MAG_CS_GPIO_Port
#define MAG_CS_PIN					MAG_CS_Pin
#endif

/* Pin macros */
#define ACCL_CS_LOW					HAL_GPIO_WritePin(ACCL_CS_PORT,ACCL_CS_PIN,0)
#define ACCL_CS_HIGH				HAL_GPIO_WritePin(ACCL_CS_PORT,ACCL_CS_PIN,1)
#define MAG_CS_LOW					HAL_GPIO_WritePin(MAG_CS_PORT,MAG_CS_PIN,0)
#define MAG_CS_HIGH					HAL_GPIO_WritePin(MAG_CS_PORT,MAG_CS_PIN,1)

/* Identification number */
#define ACCL_WHO_AM_I			0b01000001
#define MAG_WHO_AM_I			0b00111101

/* Registers addresses */
#define ACCL_REG_WHO_AM_I		0x0F
#define ACCL_REG_ACT_THS		0x1E
#define ACCL_REG_ACT_DUR		0x1F
#define ACCL_REG_CTRL_REG1		0x20
#define ACCL_REG_CTRL_REG2		0x21
#define ACCL_REG_CTRL_REG3		0x22
#define ACCL_REG_CTRL_REG4		0x23
#define ACCL_REG_CTRL_REG5		0x24
#define ACCL_REG_CTRL_REG6		0x25
#define ACCL_REG_CTRL_REG7		0x26
#define ACCL_REG_STATUS_REG		0x27
#define ACCL_REG_OUT_X_L		0x28
#define ACCL_REG_OUT_X_H		0x29
#define ACCL_REG_OUT_Y_L		0x2A
#define ACCL_REG_OUT_Y_H		0x2B
#define ACCL_REG_OUT_Z_L		0x2C
#define ACCL_REG_OUT_Z_H		0x2D
#define ACCL_XL_REFERENCE		0x3A
#define ACCL_XH_REFERENCE		0x3B
#define ACCL_YL_REFERENCE		0x3C
#define ACCL_YH_REFERENCE		0x3D
#define ACCL_ZL_REFERENCE		0x3E
#define ACCL_ZH_REFERENCE		0x3F

typedef struct {
	int16_t X; /*!< X axis mag field */
	int16_t Y; /*!< Y axis mag field */
	int16_t Z; /*!< Z axis mag field */
} mag;

typedef struct {
	int16_t X; /*!< X axis translation */
	int16_t Y; /*!< Y axis translation */
	int16_t Z; /*!< Z axis translation */
} accl;

typedef enum {
	Scale_2g, /*!< Set full scale to +-2g  */
	Scale_4g, /*!< Set full scale to +-4g  */
	Scale_8g /*!< Set full scale to +-8g  */
} accl_Scale;

#define ACCL_SENSITIVITY		Scale_8g
#define ACCL_SENSITIVITY_2g		0.061
#define ACCL_SENSITIVITY_4g		0.122
#define ACCL_SENSITIVITY_8g		0.244

int accl_Init(accl_Scale s);
int mag_Init();

int accl_Read(accl* data);
int mag_Read(mag* data);

#endif /* ACCL_H_ */
