#include "InputConverter.h"


#define FS_SEL	131	// 250dgree/sec^2 constant
#define CAL		10	// gyro sensor log

bool InputConverter::SetData(HID_DEVICE& asyncDevice)
{
	//CHAR        szTempBuff[1024] = { 0 };
	PHID_DATA pData = asyncDevice.InputData;
	UINT        uLoop;
	PFLOAT      pFloat = &(convertData.acc.x);
	PINT8       p8 = &(convertData.enc.thumb);
	for (uLoop = 0; uLoop < asyncDevice.InputDataLength; uLoop++)
	{
		//ReportToString(pData, szTempBuff, sizeof(szTempBuff));
		//DriverLog("Dev] %s", szTempBuff);
		if (uLoop < 9) {
			*pFloat = (FLOAT)(pData->ValueData.Value);
			++pFloat;
		}
		else {
			*p8 = (INT8)(pData->ValueData.Value);
			++p8;
		}

		pData++;
	}

	//MadgwickAHRSupdate(&convertData);
	MahonyAHRSupdate(&convertData);

	return true;
}

