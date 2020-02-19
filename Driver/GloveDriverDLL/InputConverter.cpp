#include "InputConverter.h"


#define FS_SEL	131	// 250dgree/sec^2 constant
#define CAL		10	// gyro sensor log


void InputConverter::convertUnit()
{
	static bool isLogFull = false;
	static UINT8 index = 0;
	static GYRO	log[CAL] = { 0 };
	GYRO base = { 0 };
	UINT8 cnt = (isLogFull ? CAL : (index + 1));

	convertData.acc.x = (float)(rawData.acc.x / 8192.0f);
	convertData.acc.y = (float)(rawData.acc.y / 8192.0f);
	convertData.acc.z = (float)(rawData.acc.z / 8192.0f);

	convertData.mag.y = (float)rawData.mag.x * ((rawData.asa.x + 128.000f) / 256.000f) - 380;
	convertData.mag.x = (float)rawData.mag.y * ((rawData.asa.y + 128.000f) / 256.000f) + 85;
	convertData.mag.z = -(float)rawData.mag.z * ((rawData.asa.z + 128.000f) / 256.000f) - 325;

	for (int i = 0; i < cnt; i++)
	{
		base.x += log[i].x;
		base.y += log[i].y;
		base.z += log[i].z;
	}

	log[index].x = (float)rawData.gyro.x;
	log[index].y = (float)rawData.gyro.y;
	log[index].z = (float)rawData.gyro.z;

	base.x /= cnt;
	base.y /= cnt;
	base.z /= cnt;

	convertData.gyro.x = (convertData.gyro.x - base.x) / FS_SEL;
	convertData.gyro.y = (convertData.gyro.y - base.y) / FS_SEL;
	convertData.gyro.z = (convertData.gyro.z - base.z) / FS_SEL;

	index = (index + 1) % CAL;
	if (!index && !isLogFull)
		isLogFull = true;
}

bool InputConverter::SetData(HID_DEVICE& asyncDevice)
{
	//CHAR        szTempBuff[1024] = { 0 };
	PHID_DATA pData = asyncDevice.InputData;
	UINT        uLoop;
	PINT16      p16 = &(rawData.acc.x);
	PINT8       p8 = &(rawData.enc.index);	// We don't have thumb encoder now
	for (uLoop = 0; uLoop < asyncDevice.InputDataLength; uLoop++)
	{
		//ReportToString(pData, szTempBuff, sizeof(szTempBuff));
		//DriverLog("Dev] %s", szTempBuff);
		if (uLoop < 9) {
			*p16 = (INT16)(pData->ValueData.Value);
			++p16;
		}
		else {
			*p8 = (INT8)(pData->ValueData.Value);
			++p8;
		}

		pData++;
	}
	
	// match axis direction of mpu9250 & steamvr
	uLoop = rawData.acc.z;
	rawData.acc.z = 0-rawData.acc.x;
	rawData.acc.x = 0-uLoop;

	uLoop = rawData.gyro.z;
	rawData.gyro.z = 0-rawData.gyro.x;
	rawData.gyro.x = 0-uLoop;

	uLoop = rawData.mag.z;
	rawData.mag.z = 0-rawData.mag.x;
	rawData.mag.x = 0-uLoop;

	this->convertUnit();
	//MadgwickAHRSupdate(&convertData);
	MahonyAHRSupdate(&convertData);

	convertData.acc.x *= 9.8f;
	convertData.acc.y *= 9.8f;
	convertData.acc.z *= 9.8f;

	// Change gyro degree/sec to rad/sec
	convertData.gyro.x *= 0.0174533f;
	convertData.gyro.y *= 0.0174533f;
	convertData.gyro.z *= 0.0174533f;

	return true;
}

