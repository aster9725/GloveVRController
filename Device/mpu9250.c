/*
 * mpu9250.c
 *
 * Created: 2020-02-25 오후 4:54:33
 *  Author: bitcamp
 */ 

#include "mpu9250.h"
#include<util/delay.h>

#define BASEIDX	0
#define GETIDX()	(__COUNTER__ - BASEIDX)
#define ADDIDX(x, n) ((x) | (GETIDX() << (n)))

volatile float m_calibration[3];

uint16_t MPU9250_init(float* a_bias, float* g_bias)
{
	uint16_t ret;
	uint8_t data;	
	
	data = 0x80;	// reset register
	ret = TWI_WriteReg(MPU9250_ADDRESS, PWR_MGMT_1, &data, 1);
	if(ret)
	return ADDIDX(ret, 8);
	_delay_ms(100);

	data = 0x00;	// disable sleep, enable all sensor
	ret = TWI_WriteReg(MPU9250_ADDRESS, PWR_MGMT_1, &data, 1);
	if(ret)
		return ADDIDX(ret, 8);
	_delay_ms(100);
	
	data = 0x01;	// auto select clock source
	ret = TWI_WriteReg(MPU9250_ADDRESS, PWR_MGMT_1, &data, 1);
	if(ret)
		return ADDIDX(ret, 8);
	_delay_ms(100);
	
	//////////////////////////////////////////////////////////////////////////
	//		Start Calibrate
	//////////////////////////////////////////////////////////////////////////
	
	MPU9250_Calibrate(a_bias, g_bias);
	
	
	// Configure Gyro and Thermometer
	// Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
	// minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
	// be higher than 1 / 0.0059 = 170 Hz
	// DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
	// With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
	data = 0x03;	//set gyro 250Hz, low pass filter
	ret = TWI_WriteReg(MPU9250_ADDRESS, CONFIG, &data, 1);
	if(ret)
		return ADDIDX(ret, 8);
	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	data = 0x01;
	ret = TWI_WriteReg(MPU9250_ADDRESS, SMPLRT_DIV, &data, 1);	// Use a 1000 Hz rate; a rate consistent with the filter update rate
	if(ret)														// determined inset in CONFIG above
		return ADDIDX(ret, 8);
	
	// Set gyroscope full scale range
	// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	ret = TWI_ReadReg(MPU9250_ADDRESS, GYRO_CONFIG, &data, 1);
	if(ret) return ADDIDX(ret, 8);
	data &= ~0x18;	// Clear AFS bits [4:3] 
	data &= ~0x03;	// Fchoice bits [1:0]
	data |= (GFS_250DPS << 3);	// Set full scale for the gyro, dlpf 250Hz
	ret = TWI_WriteReg(MPU9250_ADDRESS, GYRO_CONFIG, &data, 1);
	if(ret) return ADDIDX(ret, 8);
	
	// Set accelerometer full-scale range configuration
	ret = TWI_ReadReg(MPU9250_ADDRESS, ACCEL_CONFIG, &data, 1);
	if(ret) return ADDIDX(ret, 8);
	data &= ~0x18;			// Clear AFS bits [4:3]
	data |= AFS_2G << 3;	// Set full scale range for the accelerometer
	ret = TWI_WriteReg(MPU9250_ADDRESS, ACCEL_CONFIG, &data, 1);	// [I2C_SLV3_REG]
	if(ret) return ADDIDX(ret, 8);
	
	// Set accelerometer sample rate configuration
	// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	ret = TWI_ReadReg(MPU9250_ADDRESS, ACCEL_CONFIG2, &data, 1);	// get current ACCEL_CONFIG2 register value
	if(ret) return ADDIDX(ret, 8);
	data &= ~0x0F;	// Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
	data |= 0x03;	// Set accelerometer rate to 218.1Hz 
	ret = TWI_WriteReg(MPU9250_ADDRESS, ACCEL_CONFIG2, &data, 1);	// Write new ACCEL_CONFIG2 register value
	if(ret) return ADDIDX(ret, 8);
	
	//////////////////////////////////////////////////////////////////////////
	//	Enable Pass-Through Mode
	//////////////////////////////////////////////////////////////////////////
	
	data = 0x00;	// Clear I2C Master Mode
	ret = TWI_WriteReg(MPU9250_ADDRESS, USER_CTRL, &data, 1);
	if(ret) return ADDIDX(ret, 8);
	_delay_ms(100);
	data = 0x02;	// Enable Bypass
	TWI_WriteReg(MPU9250_ADDRESS, INT_PIN_CFG, &data, 1);
	_delay_ms(100);
	
	
	return 0;
}

#undef BASEIDX
#define BASEIDX __COUNTER__

uint16_t AK8963_init(uint8_t Mmode, float* g_bias, float* g_scale)
{
	uint16_t ret;
	uint8_t rawData[3] = {0,};  // x/y/z gyro calibration data stored here
	uint8_t data;
	
	//power down	
	data = M_PD;
	ret = TWI_WriteReg(AK8963_ADDRESS, AK8963_CNTL, &data, 1);
	if(ret)		return ADDIDX(ret, 8);
	_delay_ms(10);
	
	// enter fuse
	data = M_FUSE;
	ret = TWI_WriteReg(AK8963_ADDRESS, AK8963_CNTL, &data, 1);
	if(ret)		return ADDIDX(ret, 8);
	_delay_ms(10);
	
	//read ASAX/Y/Z
	ret = TWI_ReadReg(AK8963_ADDRESS, AK8963_ASAX, &(rawData[0]), 3);
	if(ret)		return ADDIDX(ret, 8);
	m_calibration[0] = (float)(rawData[0] + 128) / 256.0f + 1.0f;
	m_calibration[1] = (float)(rawData[1] + 128) / 256.0f + 1.0f;
	m_calibration[2] = (float)(rawData[2] + 128) / 256.0f + 1.0f;
	
	//power down
	data = M_PD;
	ret = TWI_WriteReg(AK8963_ADDRESS, AK8963_CNTL, &data, 1);
	if(ret)
	return ADDIDX(ret, 8);
	_delay_ms(10);
	
		
	data = MFS_16BITS << 4 | Mmode;
	ret = TWI_WriteReg(AK8963_ADDRESS, AK8963_CNTL, &data, 1);
	if(ret)
		return ADDIDX(ret, 8);
	_delay_ms(10);
		
	return 0;
}

#undef BASEIDX
#define BASEIDX __COUNTER__

uint16_t AK8963_init_slave(uint8_t Mmode)	// may not work
{
	// First extract the factory calibration for each magnetometer axis
	uint8_t rawData[3];  // x/y/z gyro calibration data stored here
	                
	uint8_t data = AK8963_ADDRESS;
	TWI_WriteReg(MPU9250_ADDRESS, I2C_SLV0_ADDR, &data, 1);	// Set the I2C slave address of AK8963 and set for write.
	data = AK8963_CNTL2;
	TWI_WriteReg(MPU9250_ADDRESS, I2C_SLV0_REG, &data, 1);	// I2C slave 0 register address from where to begin data transfer
	data = 0x01;
	TWI_WriteReg(MPU9250_ADDRESS, I2C_SLV0_DO, &data, 1);	// Reset AK8963
	data = 0x81;
	TWI_WriteReg(MPU9250_ADDRESS, I2C_SLV0_CTRL, &data, 1);	// Enable I2C and write 1 byte
	_delay_ms(50);
                  
	data = AK8963_ADDRESS;
	TWI_WriteReg(MPU9250_ADDRESS, I2C_SLV0_ADDR, &data, 1);	// Set the I2C slave address of AK8963 and set for write.
	data = AK8963_CNTL;
	TWI_WriteReg(MPU9250_ADDRESS, I2C_SLV0_REG, &data, 1);	// I2C slave 0 register address from where to begin data transfer
	data = 0x00;
	TWI_WriteReg(MPU9250_ADDRESS, I2C_SLV0_DO, &data, 1);	// Power down magnetometer
	data = 0x81;
	TWI_WriteReg(MPU9250_ADDRESS, I2C_SLV0_CTRL, &data, 1);	// Enable I2C and write 1 byte
	_delay_ms(50);

	data = AK8963_ADDRESS;
	TWI_WriteReg(MPU9250_ADDRESS, I2C_SLV0_ADDR, &data, 1);	// Set the I2C slave address of AK8963 and set for write.
	data = AK8963_CNTL;
	TWI_WriteReg(MPU9250_ADDRESS, I2C_SLV0_REG, &data, 1);	// I2C slave 0 register address from where to begin data transfer
	data = 0x0F;
	TWI_WriteReg(MPU9250_ADDRESS, I2C_SLV0_DO, &data, 1);	// Enter fuse mode
	data = 0x81;
	TWI_WriteReg(MPU9250_ADDRESS, I2C_SLV0_CTRL, &data, 1);	// Enable I2C and write 1 byte
	_delay_ms(50);
	 
	data = AK8963_ADDRESS | 0x80;
	TWI_WriteReg(MPU9250_ADDRESS, I2C_SLV0_ADDR, &data, 1);		// Set the I2C slave address of AK8963 and set for read.
	data = AK8963_ASAX;
	TWI_WriteReg(MPU9250_ADDRESS, I2C_SLV0_REG, &data, 1);		// I2C slave 0 register address from where to begin data transfer
	data = 0x83;
	TWI_WriteReg(MPU9250_ADDRESS, I2C_SLV0_CTRL, &data, 1);		// Enable I2C and read 3 bytes
	_delay_ms(50);
	TWI_ReadReg(MPU9250_ADDRESS, EXT_SENS_DATA_00, &rawData[0], 3);	// Read the x-, y-, and z-axis calibration values
																	// Return x-axis sensitivity adjustment values, etc.

	data = AK8963_ADDRESS;
	TWI_WriteReg(MPU9250_ADDRESS, I2C_SLV0_ADDR, &data, 1);	// Set the I2C slave address of AK8963 and set for write.
	data = AK8963_CNTL;
	TWI_WriteReg(MPU9250_ADDRESS, I2C_SLV0_REG, &data, 1);	// I2C slave 0 register address from where to begin data transfer
	data = 0x00;
	TWI_WriteReg(MPU9250_ADDRESS, I2C_SLV0_DO, &data, 1);	// Power down magnetometer
	data = 0x81;
	TWI_WriteReg(MPU9250_ADDRESS, I2C_SLV0_CTRL, &data, 1);	// Enable I2C and transfer 1 byte
	_delay_ms(50);

	data = AK8963_ADDRESS;
	TWI_WriteReg(MPU9250_ADDRESS, I2C_SLV0_ADDR, &data, 1);	// Set the I2C slave address of AK8963 and set for write.
	data = AK8963_CNTL;
	TWI_WriteReg(MPU9250_ADDRESS, I2C_SLV0_REG, &data, 1);	// I2C slave 0 register address from where to begin data transfer
	// Configure the magnetometer for continuous read and highest resolution
	// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
	// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
	data = MFS_16BITS << 4 | 0x01;
	TWI_WriteReg(MPU9250_ADDRESS, I2C_SLV0_DO, &data, 1);	// Set magnetometer data resolution and sample ODR
	data = 0x81;
	TWI_WriteReg(MPU9250_ADDRESS, I2C_SLV0_CTRL, &data, 1);	// Enable I2C and transfer 1 byte
	_delay_ms(100);

	return 0;
}

uint8_t MPU9250_Calibrate(float* a_bias, float* g_bias)
{
	uint8_t data[12]; // for acc & gyro
	uint16_t i = 0, packet_count = 0, fifo_count = 0;
	int32_t gyro_bias[3] = {0,}, acc_bias[3] = {0,};
	int16_t a_temp[3] = {0,}, g_temp[3] = {0,};
	uint8_t regVal;
	uint16_t g_sens = 131;
	uint16_t a_sens = 16384;
	int32_t acc_bias_reg[3] = {0,};
	uint32_t mask = 1UL;
	uint8_t mask_bit[3] = {0,};
	uint16_t ret = 0;
	
	regVal = 0x00; // disable interrupts
	ret = TWI_WriteReg(MPU9250_ADDRESS, INT_ENABLE, &regVal, 1);
	if(ret)	return ADDIDX(ret, 8);
	regVal = 0x00; // disable fifo
	ret = TWI_WriteReg(MPU9250_ADDRESS, FIFO_EN, &regVal, 1);
	if(ret)	return ADDIDX(ret, 8);
	regVal = 0x00; // use internal clock
	ret = TWI_WriteReg(MPU9250_ADDRESS, PWR_MGMT_1, &regVal, 1);
	if(ret)	return ADDIDX(ret, 8);
	regVal = 0x00; // disable fifo, i2c master
	ret = TWI_WriteReg(MPU9250_ADDRESS, USER_CTRL, &regVal, 1);
	if(ret)	return ADDIDX(ret, 8);
	regVal = 0x0C; // reset fifo & dmp
	ret = TWI_WriteReg(MPU9250_ADDRESS, USER_CTRL, &regVal, 1);
	if(ret)	return ADDIDX(ret, 8);
	
	_delay_ms(15);
	
	regVal = 0x01; // set DLPF 1 FCHOICE_b 0b00 (FCHOICE 0x11)
	ret = TWI_WriteReg(MPU9250_ADDRESS, CONFIG, &regVal, 1);
	if(ret)	return ADDIDX(ret, 8);
	regVal = 0x00; // set sample rate 1kHz
	ret = TWI_WriteReg(MPU9250_ADDRESS, SMPLRT_DIV, &regVal, 1);
	if(ret)	return ADDIDX(ret, 8);
	regVal = 0x00; // set gyro scale 250
	ret = TWI_WriteReg(MPU9250_ADDRESS, GYRO_CONFIG, &regVal, 1);
	if(ret)	return ADDIDX(ret, 8);
	regVal = 0x00; // set acc scale 2g
	ret = TWI_WriteReg(MPU9250_ADDRESS, ACCEL_CONFIG, &regVal, 1);
	if(ret)	return ADDIDX(ret, 8);
	
	regVal = 0x40; // enable FIFO
	ret = TWI_WriteReg(MPU9250_ADDRESS, USER_CTRL, &regVal, 1);
	if(ret)	return ADDIDX(ret, 8);
	regVal = 0x78; // enable gyro & acc fifo
	ret = TWI_WriteReg(MPU9250_ADDRESS, FIFO_EN, &regVal, 1);
	if(ret)	return ADDIDX(ret, 8);
	
	_delay_ms(40);	// wait for fifo is filled by acc/gyro data
	
	regVal = 0x00; // disable fifo
	ret = TWI_WriteReg(MPU9250_ADDRESS, FIFO_EN, &regVal, 1);
	if(ret)	return ADDIDX(ret, 8);
	ret = TWI_ReadReg(MPU9250_ADDRESS, FIFO_COUNTH, &(data[0]), 2);
	if(ret)	return ADDIDX(ret, 8);
	fifo_count = ((uint16_t)data[0] << 8 | data[1]);
	packet_count = fifo_count / 12;
	
	for(i = 0; i < packet_count; i++)
	{
		TWI_ReadReg(MPU9250_ADDRESS, FIFO_R_W, &(data[0]), 12);
		
		a_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
		a_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
		a_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
		g_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
		g_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
		g_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
		
		acc_bias[0] += (int32_t) a_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		acc_bias[1] += (int32_t) a_temp[1];
		acc_bias[2] += (int32_t) a_temp[2];
		gyro_bias[0]  += (int32_t) g_temp[0];
		gyro_bias[1]  += (int32_t) g_temp[1];
		gyro_bias[2]  += (int32_t) g_temp[2];
	}
	
	acc_bias[0] /= (int32_t)packet_count; // set bias to average of fifo input
	acc_bias[1] /= (int32_t)packet_count;
	acc_bias[2] /= (int32_t)packet_count;
	gyro_bias[0] /= (int32_t)packet_count;
	gyro_bias[1] /= (int32_t)packet_count;
	gyro_bias[2] /= (int32_t)packet_count;
	
	g_base[0] = gyro_bias[0];
	g_base[1] = gyro_bias[1];
	g_base[2] = gyro_bias[2];
	
	if(acc_bias[2] > 0L)	// Remove gravity from z axis
		acc_bias[2] += (int32_t) a_sens;
	else
		acc_bias[2] -= (int32_t) a_sens;

	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per degree/s to conform to expected bias input format
	data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
	data[3] = (-gyro_bias[1]/4)       & 0xFF;
	data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
	data[5] = (-gyro_bias[2]/4)       & 0xFF;
	
	//Disable FIFO
	regVal = 0x00; // enable FIFO
	ret = TWI_WriteReg(MPU9250_ADDRESS, USER_CTRL, &regVal, 1);
	if(ret)	return ADDIDX(ret, 8);

	// Push gyro biases to hardware registers
	TWI_WriteReg(MPU9250_ADDRESS, XG_OFFSET_H, &(data[0]), 6);
	
	// Output scaled gyro biases for display in the main program
	g_bias[0] = (float) gyro_bias[0]/(float) g_sens;
	g_bias[1] = (float) gyro_bias[1]/(float) g_sens;
	g_bias[2] = (float) gyro_bias[2]/(float) g_sens;

	TWI_ReadReg(MPU9250_ADDRESS, XA_OFFSET_H, &(data[0]), 6);	// Read acc offsets to get temp bit
	
	acc_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
	acc_bias_reg[1] = (int32_t) (((int16_t)data[2] << 8) | data[3]);
	acc_bias_reg[2] = (int32_t) (((int16_t)data[4] << 8) | data[5]);

	for(i = 0; i < 3; i++)
	{
		if((acc_bias_reg[i] & mask))
			mask_bit[i] = 0x01;
	}

	acc_bias_reg[0] -= (acc_bias[0]/8);
	acc_bias_reg[1] -= (acc_bias[1]/8);
	acc_bias_reg[2] -= (acc_bias[2]/8);
	
	//acc_bias_reg[0] &= ~0x0001;
	//acc_bias_reg[1] &= ~0x0001;
	//acc_bias_reg[2] &= ~0x0001;
	
	data[0] = (acc_bias_reg[0] >> 8) & 0xFF;
	data[1] = (acc_bias_reg[0])      & 0xFF;
	data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[2] = (acc_bias_reg[1] >> 8) & 0xFF;
	data[3] = (acc_bias_reg[1])      & 0xFF;
	data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[4] = (acc_bias_reg[2] >> 8) & 0xFF;
	data[5] = (acc_bias_reg[2])      & 0xFF;
	data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	
	// Write bias values to acc_offset register
	TWI_ReadReg(MPU9250_ADDRESS, XA_OFFSET_H, &(data[0]), 6);
	
	a_bias[0] = (float)acc_bias[0]/(float)a_sens;
	a_bias[1] = (float)acc_bias[1]/(float)a_sens;
	a_bias[2] = (float)acc_bias[2]/(float)a_sens;
	
	return 0;
}

uint8_t AK8963_Calibrate(float* mag_bias, float* mag_scale)
{
	uint16_t i, j;
	int32_t m_bias[3] = {0,}, m_scale[3] = {0,};
	int16_t m_max[3] = {-32767, -32767, -32767}, m_min[3] = {32767, 32767, 32767}, m_temp[3] = {0, 0, 0};
	uint8_t m_raw[7] = {0};
	float avg_rad;
		
	_delay_ms(2000);
	
	for(i = 0; i < 1500; i++)	// 100Hz, 10ms for each measure
	{
		while(readMag(&(m_raw[0])));	// wait until magnetometer data read successfully
		
		m_temp[0] = ((int16_t)m_raw[0] << 8 | m_raw[1]);
		m_temp[1] = ((int16_t)m_raw[2] << 8 | m_raw[3]);
		m_temp[2] = ((int16_t)m_raw[4] << 8 | m_raw[5]);
		
		for(j = 0; j < 3; j++)
		{
			if(m_temp[j] > m_max[j])
				m_max[j] = m_temp[j];
			if(m_temp[j] < m_min[j])
				m_min[j] = m_temp[j];
		}
	}
	
	m_bias[0] = (m_max[0] + m_min[0]) / 2;
	m_bias[1] = (m_max[1] + m_min[1]) / 2;
	m_bias[2] = (m_max[2] + m_min[2]) / 2;
	
	mag_bias[0] = (float) m_bias[0] * M_RES * m_calibration[0];
	mag_bias[1] = (float) m_bias[1] * M_RES * m_calibration[1];
	mag_bias[2] = (float) m_bias[2] * M_RES * m_calibration[2];
	
	m_scale[0]  = (m_max[0] - m_min[0])/2;  // get average x axis max chord length in counts
	m_scale[1]  = (m_max[1] - m_min[1])/2;  // get average y axis max chord length in counts
	m_scale[2]  = (m_max[2] - m_min[2])/2;  // get average z axis max chord length in counts
	
	avg_rad = m_scale[0] + m_scale[1] + m_scale[2];
	avg_rad /= 3.0;
	
	mag_scale[0] = avg_rad / ((float)m_scale[0]);
	mag_scale[1] = avg_rad / ((float)m_scale[1]);
	mag_scale[2] = avg_rad / ((float)m_scale[2]);
	
	return 0;
}
 
uint8_t readACC(uint8_t* data)
{
	uint16_t ret;
	// Check Acc Data Ready
	ret = TWI_ReadReg(MPU9250_ADDRESS, INT_STATUS, &(data[0]), 1);
	if(ret)
		return ERR_DATA_READ_FAIL;	// Data read failed	
	else
	{
		if(data[0] & 0x01)
		{
			ret = TWI_ReadReg(MPU9250_ADDRESS, ACCEL_XOUT_H, &(data[0]), 6);
			if(ret)
				return ERR_DATA_READ_FAIL;	// Data read failed
		}
		else
			return ERR_DATA_NOT_READY(_ACC);	// Data not ready
	}
	
	return 0;
}
 
uint8_t readGyro(uint8_t* data)
{
	 uint16_t ret;
	 // Check Acc Data Ready
	 ret = TWI_ReadReg(MPU9250_ADDRESS, INT_STATUS, &(data[0]), 1);
	 if(ret)
	 return ERR_DATA_READ_FAIL;	// Data read failed
	 else
	 {
		 if(data[0] & 0x01)
		 {
			 ret = TWI_ReadReg(MPU9250_ADDRESS, GYRO_XOUT_H, &(data[0]), 6);
			 if(ret)
			 return ERR_DATA_READ_FAIL;	// Data read failed
		 }
		 else
			return ERR_DATA_NOT_READY(_GYRO);	// Data not ready
	 }
	 
	return 0;
}

uint8_t readMag(uint8_t* data)
{
	uint16_t ret;
	
	// Check Mag Data Ready
	ret = TWI_ReadReg(AK8963_ADDRESS, AK8963_ST1, &(data[0]), 1);
	if(ret)
		return ERR_DATA_READ_FAIL;	// Data read failed
	else
	{
		if(data[0] & 0x01)
		{
			// Read to ST2
			ret = TWI_ReadReg(AK8963_ADDRESS, AK8963_XOUT_L, &(data[0]), 7);
			if(ret)
				return ERR_DATA_READ_FAIL;	// Data read failed
			else
			{
				if(data[6] & 0x08)
					return ERR_DATA_OVERFLOW;	// Data overflowed. data need to be discard
			}
		}
		else
			return ERR_DATA_NOT_READY(_MAG);	// Data Not Ready
	}
	return 0;	// Data Successfully read
}

uint8_t readAll(uint8_t* data)
{
	uint8_t ret = 0;
	ret |= readACC(&(data[0]));
	if(ret)
		ret |= ERR_ACC;

	ret |= readGyro(&(data[6]));
	if(ret)
		ret |= ERR_GYRO;
		
	ret |= readMag(&(data[12]));
	if(ret)
		ret |= ERR_MAG;

	return ret;
}