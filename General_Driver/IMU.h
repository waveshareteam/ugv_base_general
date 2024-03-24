#ifndef _IMU_H_
#define _IMU_H_

#include "AK09918.h"
#include "QMI8658.h"
#include <stdio.h>
#include <math.h>

typedef struct imu_st_angles_data_tag
{
  float fYaw;
  float fPitch;
  float fRoll;
}IMU_ST_ANGLES_DATA;

typedef struct imu_st_sensor_data_float
{
  float X;
  float Y;
  float Z;
}IMU_ST_SENSOR_DATA_FLOAT;

void imuInit();
void imuDataGet(EulerAngles *pstAngles, 
                IMU_ST_SENSOR_DATA_FLOAT *pstGyroRawData,
                IMU_ST_SENSOR_DATA_FLOAT *pstAccelRawData,
                IMU_ST_SENSOR_DATA *pstMagnRawData); 
void imuAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
float invSqrt(float x);

#endif
