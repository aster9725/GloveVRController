#pragma once
#include "glovedatatype.h"

extern "C" {
#include "hid.h"
#include "MadgwickAHRS.h"
#include "MahonyAHRS.h"
}

class InputConverter
{
	USB_REPORT_DATA_T	rawData;
	GLOVE_POSE_DATA_T	convertData;
	void convertUnit();

public:
	InputConverter()
	{
		convertData = { 0 };
		rawData.asa.x = -380;
		rawData.asa.y = 85;
		rawData.asa.z = -325;
	}
	bool SetData(HID_DEVICE& asyncDevice);
	PUSB_REPORT_DATA_T GetRawData() { return &rawData; }
	PGLOVE_POSE_DATA_T GetPoseData() { return &convertData; }
};

