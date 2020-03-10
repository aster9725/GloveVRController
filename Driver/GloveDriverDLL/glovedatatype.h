#pragma once

#include <wtypes.h>

typedef struct {
	INT8 thumb;
	INT8 index;
	INT8 middle;
	INT8 ring;
	INT8 pinky;
}FLEX, ENCODER;

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
	ENCODER		enc;
	FLEX		flex;
}GLOVE_POSE_DATA_T, * PGLOVE_POSE_DATA_T;