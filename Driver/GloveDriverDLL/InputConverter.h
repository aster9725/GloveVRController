#pragma once
#include "glovedatatype.h"

extern "C" {
#include "hid.h"
#include "MadgwickAHRS.h"
#include "MahonyAHRS.h"
}

class InputConverter
{
	GLOVE_POSE_DATA_T	convertData;

public:
	InputConverter()
	{
		convertData = { 0 };
		convertData.qPos.w = 1.0f;
	}
	bool SetData(HID_DEVICE& asyncDevice);
	PGLOVE_POSE_DATA_T GetPoseData() { return &convertData; }
};

