#include "InputConverter.h"
#include "driverlog.h"


#define FS_SEL	131	// 250dgree/sec^2 constant
#define CAL		10	// gyro sensor log

bool InputConverter::SetData(HID_DEVICE& asyncDevice)
{
	//CHAR        szTempBuff[1024] = { 0 };
	PHID_DATA pData = asyncDevice.InputData;
	UINT        uLoop;
	PFLOAT      pFloat = &(convertData.mag.z);
	PINT8       p8 = &(convertData.enc.thumb);

	for (uLoop = 0; uLoop < asyncDevice.InputDataLength; uLoop++)
	{
		//ReportToString(pData, szTempBuff, sizeof(szTempBuff));
		//DriverLog("Dev] %s", szTempBuff);

		if (uLoop < 9) {
			//DriverLog("Dev] %d", pData->ValueData.Usage);
			memcpy(pFloat, &(pData->ValueData.Value), 4);
			--pFloat;
		}
		else {
			//DriverLog("Dev] %d", pData->ValueData.Usage);
			memcpy(p8, &(pData->ValueData.Value), 1);
			++p8;
		}

		pData++;
	}
	
	FLOAT temp;
	temp = convertData.acc.x;
	convertData.acc.x = convertData.acc.z;
	convertData.acc.z = temp;

	temp = convertData.gyro.x;
	convertData.gyro.x = convertData.gyro.z;
	convertData.gyro.z = temp;

	temp = convertData.mag.x;
	convertData.mag.x = convertData.mag.z;
	convertData.mag.z = temp;

	//MadgwickAHRSupdate(&convertData);
	MahonyAHRSupdate(&convertData);


	DriverLog("Dev ] %+10.4f    %+10.4f    %+10.4f",
		convertData.mag.x, convertData.mag.y, convertData.mag.z);

	/*DriverLog("Dev ] %+10.4f    %+10.4f    %+10.4f    %+10.4f",
		convertData.qPos.w, convertData.qPos.x, convertData.qPos.y, convertData.qPos.z);*/

	return true;
}

