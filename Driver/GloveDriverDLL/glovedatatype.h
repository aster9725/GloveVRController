#pragma once

#include <wtypes.h>

typedef struct {
	INT16 x;
	INT16 y;
	INT16 z;
}RAWACC, RAWGYRO, RAWMAG, ASA;

typedef struct {
	INT8 thumb;
	INT8 index;
	INT8 middle;
	INT8 ring;
	INT8 pinky;
}FLEX, ENCODER;

typedef struct {
	RAWACC	acc;
	RAWGYRO	gyro;
	RAWMAG	mag;
	ASA		asa;
	ENCODER	enc;
	FLEX	flex;
}USB_REPORT_DATA_T, * PUSB_REPORT_DATA_T;

typedef struct {
	double w;
	double x;
	double y;
	double z;
}QUATANION;

typedef struct {
	float x;
	float y;
	float z;
}ACC, GYRO, MAG;

typedef struct {
	QUATANION	qPos;
	ACC			acc;
	GYRO		gyro;
	MAG			mag;
}GLOVE_POSE_DATA_T, * PGLOVE_POSE_DATA_T;