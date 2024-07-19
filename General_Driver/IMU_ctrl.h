#include"IMU.h"

// define GPIOs for IIC.
EulerAngles stAngles;
IMU_ST_SENSOR_DATA_FLOAT stGyroRawData;
IMU_ST_SENSOR_DATA_FLOAT stAccelRawData;
IMU_ST_SENSOR_DATA stMagnRawData;
float temp;


void imu_init() {
	imuInit();
}


void updateIMUData() {
	imuDataGet( &stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);

	ax = stAccelRawData.X;
	ay = stAccelRawData.Y;
	az = stAccelRawData.Z;

	mx = stMagnRawData.s16X;
	my = stMagnRawData.s16Y;
	mz = stMagnRawData.s16Z;

	gx = stGyroRawData.X;
	gy = stGyroRawData.Y;
	gz = stGyroRawData.Z;

	icm_roll = stAngles.roll;
	icm_pitch = stAngles.pitch;
	icm_yaw = stAngles.yaw;

  temp = temperatureRead();
}


void imuCalibration() {

}


void getIMUData() {
	jsonInfoHttp.clear();
	jsonInfoHttp["T"] = FEEDBACK_IMU_DATA;

	jsonInfoHttp["r"] = icm_roll;
	jsonInfoHttp["p"] = icm_pitch;
	jsonInfoHttp["y"] = icm_yaw;

	jsonInfoHttp["ax"] = ax;
	jsonInfoHttp["ay"] = ay;
	jsonInfoHttp["az"] = az;

	jsonInfoHttp["gx"] = gx;
	jsonInfoHttp["gy"] = gy;
	jsonInfoHttp["gz"] = gz;

	jsonInfoHttp["mx"] = mx;
	jsonInfoHttp["my"] = my;
	jsonInfoHttp["mz"] = mz;

	jsonInfoHttp["temp"] = temp;

	String getInfoJsonString;
	serializeJson(jsonInfoHttp, getInfoJsonString);
	Serial.println(getInfoJsonString);
}

void getIMUOffset() {

}

void setIMUOffset(int16_t inputX, int16_t inputY, int16_t inputZ) {

}