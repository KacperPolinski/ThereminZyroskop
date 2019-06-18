#ifndef DCM_H
#define DCM_H
#include <stdbool.h>
#include "stm32l4xx.h"

#define MAG_Write 0x3C
#define MAG_Read 0x3D
#define GYRO 0b11010010

// Accelerometer
// "accel x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define ACCEL_X_MIN ((float) -8187)
#define ACCEL_X_MAX ((float) 8568)
#define ACCEL_Y_MIN ((float) -8507)
#define ACCEL_Y_MAX ((float) 8667)
#define ACCEL_Z_MIN ((float) -9599)
#define ACCEL_Z_MAX ((float) 7648)


// Gyroscope
// "gyro x,y,z (current/average) = .../OFFSET_X  .../OFFSET_Y  .../OFFSET_Z
#define GYRO_AVERAGE_OFFSET_X ((float) 21.523394)
#define GYRO_AVERAGE_OFFSET_Y ((float) 3.794607)
#define GYRO_AVERAGE_OFFSET_Z ((float) -6.371927)


#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
#define ACCEL_X_SCALE (GRAVITY / (ACCEL_X_MAX - ACCEL_X_OFFSET))
#define ACCEL_Y_SCALE (GRAVITY / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
#define ACCEL_Z_SCALE (GRAVITY / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))


#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi

extern float accel[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
extern float gyro[3];
extern float acceleration_g[3];
extern float angular_rate_dps[3];
extern float lacceleration_g[3];
extern float langular_rate_dps[3];

extern float pitch;
extern float roll;
extern float yaw;
extern float pitch_tmp;
extern float roll_tmp;

extern double alpha;
extern double lastMeasurment;
extern double timeElapsed;

double accRoll ;
double accPitch;

double gyroRoll ;
double gyroPitch;

double laccRoll ;
double laccPitch;

double lgyroRoll ;
double lgyroPitch;

double accYaw ;
double gyroYaw;

double laccYaw ;
double lgyroYaw;

void Init_MPU();
void Read_MPU_Gyro();
void Read_MPU_Accl();
void Exchange();
void Filter();
#endif
