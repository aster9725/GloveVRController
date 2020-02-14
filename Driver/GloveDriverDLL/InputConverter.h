#pragma once
#include "glovedatatype.h"

class InputConverter
{
	USB_REPORT_DATA_T	rawData;
	GLOVE_POSE_DATA_T	refinedData;
	void convertUnit();

public:
	bool SetRawData(USB_REPORT_DATA_T& src);
	PUSB_REPORT_DATA_T GetRawData() { return &rawData; }
	PGLOVE_POSE_DATA_T GetPoseData() { return &refinedData; }
};

