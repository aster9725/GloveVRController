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

	refinedData.acc.x = (float)(rawData.acc.x / 8192.0f);// * 9.8;
	refinedData.acc.y = (float)(rawData.acc.y / 8192.0f);// * 9.8;
	refinedData.acc.z = (float)(rawData.acc.z / 8192.0f);// * 9.8;

	refinedData.mag.x = (float)rawData.mag.x * ((rawData.asa.x + 128.000f) / 256.000f) - 380;
	refinedData.mag.y = (float)rawData.mag.y * ((rawData.asa.y + 128.000f) / 256.000f) + 85;
	refinedData.mag.z = (float)rawData.mag.z * ((rawData.asa.z + 128.000f) / 256.000f) - 325;

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

	refinedData.gyro.x = (refinedData.gyro.x - base.x) / FS_SEL;
	refinedData.gyro.y = (refinedData.gyro.y - base.y) / FS_SEL;
	refinedData.gyro.z = (refinedData.gyro.z - base.z) / FS_SEL;

	refinedData.gyro.x *= 0.0174533f;
	refinedData.gyro.y *= 0.0174533f;
	refinedData.gyro.z *= 0.0174533f;

	index = (index + 1) % CAL;
	if (!index && !isLogFull)
		isLogFull = true;
}

bool InputConverter::SetRawData(USB_REPORT_DATA_T& src)
{
	rawData.acc.x = -src.acc.z;
	rawData.acc.y = src.acc.y;
	rawData.acc.z = src.acc.x;

	rawData.gyro.x = -src.gyro.z;
	rawData.gyro.y = src.gyro.y;
	rawData.gyro.z = src.gyro.x;

	rawData.mag.x = -src.mag.z;
	rawData.mag.y = src.mag.y;
	rawData.mag.z = src.mag.x;

	rawData.enc = src.enc;

	rawData.flex = src.flex;

	/*rawData.enc.thumb	= src.enc.thumb;
	rawData.enc.index	= src.enc.index;
	rawData.enc.middle	= src.enc.middle;
	rawData.enc.ring	= src.enc.ring;
	rawData.enc.pinky	= src.enc.pinky;

	rawData.flex.thumb	= src.flex.thumb;
	rawData.flex.index	= src.flex.index;
	rawData.flex.middle	= src.flex.middle;
	rawData.flex.ring	= src.flex.ring;
	rawData.flex.pinky	= src.flex.pinky;*/

	this->convertUnit();

	return true;
}

