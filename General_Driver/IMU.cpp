#include "IMU.h"

void calibrateMagn();
void imuAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
float invSqrt(float x);

/******************************************************************************
 * IMU module                                                                 *
 ******************************************************************************/
// #define S_SCL   33
// #define S_SDA   32

AK09918_err_type_t err;

QMI8658 qmi8658_;
AK09918 magnetometer_;

int16_t offset_x = -12, offset_y = 0, offset_z = 0;
int16_t x, y, z;
// Find the magnetic declination at your location
// http://www.magnetic-declination.com/
double declination_shenzhen = -3.22;

#define Kp 4.50f   // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 1.0f    // integral gain governs rate of convergence of gyroscope biases

float angles[3];
float q0, q1, q2, q3; 

void imuInit()
{
    // Wire.begin(S_SDA, S_SCL);
    // Serial.begin(115200);
   
    if (qmi8658_.begin() == 0)
	      Serial.println("qmi8658_init fail");

    if (magnetometer_.initialize())
        Serial.println("AK09918_init fail") ;
    magnetometer_.switchMode(AK09918_CONTINUOUS_100HZ);
    err = magnetometer_.isDataReady();
    int retry_times = 0;
    while (err != AK09918_ERR_OK) {
        Serial.println(err);
        Serial.println("Waiting Sensor");
        delay(100);
        magnetometer_.reset();
        delay(100);
        magnetometer_.switchMode(AK09918_CONTINUOUS_100HZ);
        err = magnetometer_.isDataReady();
        retry_times ++;
        if (retry_times > 10) {
          break;
        }
    }
    // Serial.println("Start figure-8 calibration after 1 seconds.");
    // delay(1000);
    // calibrate(10000, &offset_x, &offset_y, &offset_z);
    // calibrateMagn();
    q0 = 1.0f;  
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
}

void imuDataGet(EulerAngles *pstAngles, 
                IMU_ST_SENSOR_DATA_FLOAT *pstGyroRawData,
                IMU_ST_SENSOR_DATA_FLOAT *pstAccelRawData,
                IMU_ST_SENSOR_DATA *pstMagnRawData)
{

  float  acc[3], gyro[3];
  float MotionVal[9];

  magnetometer_.getData(&x, &y, &z);

  pstMagnRawData->s16X = x- offset_x;
  pstMagnRawData->s16Y = y- offset_y;
  pstMagnRawData->s16Z = z- offset_z;

  // qmi8658_.GetEulerAngles(&pstAngles->pitch,&pstAngles->roll,&pstAngles->yaw,acc,gyro);
  qmi8658_.read_sensor_data(acc,gyro);

  // pstAngles->roll = atan2((float)acc[1], (float)acc[2]);
  // pstAngles->pitch = atan2(-(float)acc[0], sqrt((float)(acc[1] * acc[1]) + (float)(acc[2] * acc[2])));

  // double Xheading = pstMagnRawData->s16X * cos(pstAngles->pitch) + pstMagnRawData->s16Y * sin(pstAngles->roll) * sin(pstAngles->pitch) + pstMagnRawData->s16Z * cos(pstAngles->roll) * sin(pstAngles->pitch);
  // double Yheading = pstMagnRawData->s16Y * cos(pstAngles->roll) - pstMagnRawData->s16Z * sin(pstAngles->pitch);
  
  // pstAngles->yaw = /*180 + */57.3 * atan2(Yheading, Xheading) + declination_shenzhen;

  // pstAngles->roll = atan2((float)acc[1], (float)acc[2]) * 57.3;
  // pstAngles->pitch = atan2(-(float)acc[0], sqrt((float)(acc[1] * acc[1]) + (float)(acc[2] * acc[2]))) * 57.3;
  MotionVal[0]=gyro[0];
  MotionVal[1]=gyro[1];
  MotionVal[2]=gyro[2];
  MotionVal[3]=acc[0];
  MotionVal[4]=acc[1];
  MotionVal[5]=acc[2];
  MotionVal[6]=pstMagnRawData->s16X;
  MotionVal[7]=pstMagnRawData->s16Y;
  MotionVal[8]=pstMagnRawData->s16Z;

  imuAHRSupdate((float)MotionVal[0] * 0.0175, (float)MotionVal[1] * 0.0175, (float)MotionVal[2] * 0.0175,
                (float)MotionVal[3], (float)MotionVal[4], (float)MotionVal[5], 
                (float)MotionVal[6], (float)MotionVal[7], MotionVal[8]);



  pstAngles->pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch
  pstAngles->roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
  pstAngles->yaw = atan2(-2 * q1 * q2 - 2 * q0 * q3, 2 * q2 * q2 + 2 * q3 * q3 - 1) * 57.3; 

  pstGyroRawData->X = gyro[0];
  pstGyroRawData->Y = gyro[1];
  pstGyroRawData->Z = gyro[2];

  pstAccelRawData->X = acc[0];
  pstAccelRawData->Y = acc[1];
  pstAccelRawData->Z = acc[2];

  return;  
}

void imuAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) 
{
  float norm;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float exInt = 0.0, eyInt = 0.0, ezInt = 0.0;
  float ex, ey, ez, halfT = 0.024f; /*half the sample period*/

  float q0q0 = q0 * q0;
  float q0q1 = q0 * q1;
  float q0q2 = q0 * q2;
  float q0q3 = q0 * q3;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q2q2 = q2 * q2;   
  float q2q3 = q2 * q3;
  float q3q3 = q3 * q3;          

  norm = invSqrt(ax * ax + ay * ay + az * az);       
  ax = ax * norm;
  ay = ay * norm;
  az = az * norm;

  norm = invSqrt(mx * mx + my * my + mz * mz);          
  mx = mx * norm;
  my = my * norm;
  mz = mz * norm;

  // compute reference direction of flux
  hx = 2 * mx * (0.5f - q2q2 - q3q3) + 2 * my * (q1q2 - q0q3) + 2 * mz * (q1q3 + q0q2);
  hy = 2 * mx * (q1q2 + q0q3) + 2 * my * (0.5f - q1q1 - q3q3) + 2 * mz * (q2q3 - q0q1);
  hz = 2 * mx * (q1q3 - q0q2) + 2 * my * (q2q3 + q0q1) + 2 * mz * (0.5f - q1q1 - q2q2);         
  bx = sqrt((hx * hx) + (hy * hy));
  bz = hz;     

  // estimated direction of gravity and flux (v and w)
  vx = 2 * (q1q3 - q0q2);
  vy = 2 * (q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
  wx = 2 * bx * (0.5 - q2q2 - q3q3) + 2 * bz * (q1q3 - q0q2);
  wy = 2 * bx * (q1q2 - q0q3) + 2 * bz * (q0q1 + q2q3);
  wz = 2 * bx * (q0q2 + q1q3) + 2 * bz * (0.5 - q1q1 - q2q2);  

  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay * vz - az * vy) + (my * wz - mz * wy);
  ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
  ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

  if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
  {
    exInt = exInt + ex * Ki * halfT;
    eyInt = eyInt + ey * Ki * halfT;  
    ezInt = ezInt + ez * Ki * halfT;

    gx = gx + Kp * ex + exInt;
    gy = gy + Kp * ey + eyInt;
    gz = gz + Kp * ez + ezInt;
  }

  q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
  q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
  q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
  q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;  

  norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 = q0 * norm;
  q1 = q1 * norm;
  q2 = q2 * norm;
  q3 = q3 * norm;
}

float invSqrt(float x) 
{
  float halfx = 0.5f * x;
  float y = x;
  
  long i = *(long*)&y;                //get bits for floating value
  i = 0x5f3759df - (i >> 1);          //gives initial guss you
  y = *(float*)&i;                    //convert bits back to float
  y = y * (1.5f - (halfx * y * y));   //newtop step, repeating increases accuracy
  
  return y;
}

void calibrateMagn(void)
{
  int16_t temp[9];
  Serial.printf("keep 10dof-imu device horizontal and it will read x y z axis offset value after 4 seconds\n");
  delay(4000);
  Serial.printf("start read all axises offset value\n");
  magnetometer_.getData(&x, &y, &z);
  temp[0] = x;
  temp[1] = y;
  temp[2] = z;
  
  Serial.printf("rotate z axis 180 degrees and it will read all axises offset value after 4 seconds\n");
  delay(4000);
  Serial.printf("start read all axises offset value\n");
  magnetometer_.getData(&x, &y, &z);
  temp[3] = x;
  temp[4] = y;
  temp[5] = z;

  Serial.printf("flip 10dof-imu device and keep it horizontal and it will read all axises offset value after 4 seconds\n");
  delay(4000);
  Serial.printf("start read all axises offset value\n");
  magnetometer_.getData(&x, &y, &z);
  temp[6] = x;
  temp[7] = y;
  temp[8] = z;

  offset_x = (temp[0]+temp[3])/2;
  offset_y = (temp[1]+temp[4])/2;
  offset_z = (temp[5]+temp[8])/2;
}
