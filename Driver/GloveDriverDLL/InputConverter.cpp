#include "InputConverter.h"

#define FS_SEL	131
#define CAL		10


void InputConverter::convertUnit()
{
	static UINT8 index = 0;
	static GYRO	log[CAL] = { 0 };
	GYRO base = { 0 };

	refinedData.acc.x = (double)(rawData.acc.x / 8192.0) * 9.8;
	refinedData.acc.y = (double)(rawData.acc.y / 8192.0) * 9.8;
	refinedData.acc.z = (double)(rawData.acc.z / 8192.0) * 9.8;

	for (int i = 0; i < CAL; i++)
	{
		base.x += log[i].x;
		base.y += log[i].y;
		base.z += log[i].z;
	}

	log[index].x = (double)rawData.gyro.x;
	log[index].y = (double)rawData.gyro.y;
	log[index].z = (double)rawData.gyro.z;

	for (int i = 0; i < CAL; i++)
	{
		refinedData.gyro.x += log[i].x;
		refinedData.gyro.y += log[i].y;
		refinedData.gyro.z += log[i].z;
	}

	base.x /= CAL;
	base.y /= CAL;
	base.z /= CAL;

	refinedData.gyro.x /= CAL;
	refinedData.gyro.y /= CAL;
	refinedData.gyro.z /= CAL;

	refinedData.gyro.x = (refinedData.gyro.x - base.x) / FS_SEL;
	refinedData.gyro.y = (refinedData.gyro.y - base.y) / FS_SEL;
	refinedData.gyro.z = (refinedData.gyro.z - base.z) / FS_SEL;

	index = (index + 1) % CAL;

	refinedData.mag.x = (double)rawData.mag.x * ((rawData.asa.x + 128.000) / 256.000) - 380;
	refinedData.mag.y = (double)rawData.mag.y * ((rawData.asa.y + 128.000) / 256.000) + 85;
	refinedData.mag.z = (double)rawData.mag.z * ((rawData.asa.z + 128.000) / 256.000) - 325;
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

