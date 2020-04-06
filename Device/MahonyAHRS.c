//=====================================================================================================
// MahonyAHRS.c
//=====================================================================================================
//
// Madgwick's implementation of Macc[1]hony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include "MahonyAHRS.h"
#include <math.h>

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	42.0f			// sample frequency in Hz
#define twoKpDef	(2.0f * 5.0f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki

//---------------------------------------------------------------------------------------------------
// Function declarations

float invSqrt(float x);

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void MahonyAHRSupdate(volatile float * gyro, volatile float * acc, volatile float * mag, float * q) {
	float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mag[0] == 0.0f) && (mag[1] == 0.0f) && (mag[2] == 0.0f)) {
		MahonyAHRSupdateIMU(gyro, acc, q);
		return;
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((acc[0] == 0.0f) && (acc[1] == 0.0f) && (acc[2] == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
		acc[0] *= recipNorm;
		acc[1] *= recipNorm;
		acc[2] *= recipNorm;     

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
		mag[0] *= recipNorm;
		mag[1] *= recipNorm;
		mag[2] *= recipNorm;   

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q[0] * q[0];
        q0q1 = q[0] * q[1];
        q0q2 = q[0] * q[2];
        q0q3 = q[0] * q[3];
        q1q1 = q[1] * q[1];
        q1q2 = q[1] * q[2];
        q1q3 = q[1] * q[3];
        q2q2 = q[2] * q[2];
        q2q3 = q[2] * q[3];
        q3q3 = q[3] * q[3];   

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mag[0] * (0.5f - q2q2 - q3q3) + mag[1] * (q1q2 - q0q3) + mag[2] * (q1q3 + q0q2));
        hy = 2.0f * (mag[0] * (q1q2 + q0q3) + mag[1] * (0.5f - q1q1 - q3q3) + mag[2] * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mag[0] * (q1q3 - q0q2) + mag[1] * (q2q3 + q0q1) + mag[2] * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
	
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (acc[1] * halfvz - acc[2] * halfvy) + (mag[1] * halfwz - mag[2] * halfwy);
		halfey = (acc[2] * halfvx - acc[0] * halfvz) + (mag[2] * halfwx - mag[0] * halfwz);
		halfez = (acc[0] * halfvy - acc[1] * halfvx) + (mag[0] * halfwy - mag[1] * halfwx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gyro[0] += integralFBx;	// apply integral feedback
			gyro[1] += integralFBy;
			gyro[2] += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gyro[0] += twoKp * halfex;
		gyro[1] += twoKp * halfey;
		gyro[2] += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gyro[0] *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gyro[1] *= (0.5f * (1.0f / sampleFreq));
	gyro[2] *= (0.5f * (1.0f / sampleFreq));
	qa = q[0];
	qb = q[1];
	qc = q[2];
	q[0] += (-qb * gyro[0] - qc * gyro[1] - q[3] * gyro[2]);
	q[1] += (qa * gyro[0] + qc * gyro[2] - q[3] * gyro[1]);
	q[2] += (qa * gyro[1] - qb * gyro[2] + q[3] * gyro[0]);
	q[3] += (qa * gyro[2] + qb * gyro[1] - qc * gyro[0]); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;
	q[3] *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MahonyAHRSupdateIMU(volatile float * gyro, volatile float * acc, float * q) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((acc[0] == 0.0f) && (acc[1] == 0.0f) && (acc[2] == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
		acc[0] *= recipNorm;
		acc[1] *= recipNorm;
		acc[2] *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q[1] * q[3] - q[0] * q[2];
		halfvy = q[0] * q[1] + q[2] * q[3];
		halfvz = q[0] * q[0] - 0.5f + q[3] * q[3];
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (acc[1] * halfvz - acc[2] * halfvy);
		halfey = (acc[2] * halfvx - acc[0] * halfvz);
		halfez = (acc[0] * halfvy - acc[1] * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gyro[0] += integralFBx;	// apply integral feedback
			gyro[1] += integralFBy;
			gyro[2] += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gyro[0] += twoKp * halfex;
		gyro[1] += twoKp * halfey;
		gyro[2] += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gyro[0] *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gyro[1] *= (0.5f * (1.0f / sampleFreq));
	gyro[2] *= (0.5f * (1.0f / sampleFreq));
	qa = q[0];
	qb = q[1];
	qc = q[2];
	q[0] += (-qb * gyro[0] - qc * gyro[1] - q[3] * gyro[2]);
	q[1] += (qa * gyro[0] + qc * gyro[2] - q[3] * gyro[1]);
	q[2] += (qa * gyro[1] - qb * gyro[2] + q[3] * gyro[0]);
	q[3] += (qa * gyro[2] + qb * gyro[1] - qc * gyro[0]); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;
	q[3] *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//====================================================================================================
// END OF CODE
//====================================================================================================
