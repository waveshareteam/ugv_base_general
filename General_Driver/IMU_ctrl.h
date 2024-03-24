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
}


void imuCalibration() {
	// jsonInfoHttp.clear();
	// switch (inputStep) {
	// case 0:
	// 	if (InfoPrint == 1){
	// 		jsonInfoHttp["info"] = "keep 10dof-imu device horizontal and then calibrate next step.";
	// 	}
	// 	break;
	// case 1:
	// 	calibrateStepA();
	// 	if (InfoPrint == 1){
	// 		jsonInfoHttp["info"] = "rotate z axis 180 degrees and then calibrate next step.";
	// 	}
	// 	break;
	// case 2:
	// 	calibrateStepB();
	// 	if (InfoPrint == 1){
	// 		jsonInfoHttp["info"] = "flip 10dof-imu device and keep it horizontal and then calibrate next step.";
	// 	}
	// 	break;
	// case 3:
	// 	calibrateStepC();
	// 	if (InfoPrint == 1){
	// 		jsonInfoHttp["info"] = "calibration done.";
	// 	}
	// 	break;
	// }
	// String getInfoJsonString;
	// serializeJson(jsonInfoHttp, getInfoJsonString);
	// Serial.println(getInfoJsonString);
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
	// getIMUOffsetData(&offsetData);
	// jsonInfoHttp.clear();
	// jsonInfoHttp["T"] = 101;

	// jsonInfoHttp["x"] = offsetData.X;
	// jsonInfoHttp["y"] = offsetData.Y;
	// jsonInfoHttp["z"] = offsetData.Z;

	// String getInfoJsonString;
	// serializeJson(jsonInfoHttp, getInfoJsonString);
	// Serial.println(getInfoJsonString);
}

void setIMUOffset(int16_t inputX, int16_t inputY, int16_t inputZ) {
	// setOffset(inputX, inputY, inputZ);
}