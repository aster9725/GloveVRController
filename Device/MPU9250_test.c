


#define F_CPU 16000000UL

#define FS_SEL 16.384

#include<avr/io.h> 
#include<avr/interrupt.h>
#include<util/delay.h>
#include<math.h>
#include <stdio.h>
//#include "MadgwickAHRS.h"
#include "MahonyAHRS.h"
#include "UART.h"
#include "mpu9250.h"


float   gyroBias[3] = {0.96, -0.21, 0.12}, accelBias[3] = {0.00299, -0.00916, 0.00952};
float   magBias[3] = {71.04, 122.43, -36.90}, magScale[3]  = {1.01, 1.03, 0.96};


#define PRESCALE_VALUE				64
#define CLOCKS_PER_MICRO			(F_CPU / 1000000L)
#define CLOCKS_TO_MICROSECONDS(a)	((a) / CLOCKS_PER_MICRO)
#define MICROSECONDS_PER_TIMER0_OVERFLOW	\
(CLOCKS_TO_MICROSECONDS(PRESCALE_VALUE * 256))
#define MILLIS_INCREMENT_PER_OVERFLOW		\
(MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)
#define MICROS_INCREMENT_PER_OVERFLOW		\
(MICROSECONDS_PER_TIMER0_OVERFLOW % 1000)


// Debug string format
// check project options for using sprintf
#define FORM_CSV	"%+010.4f,%+010.4f,%+010.4f,"	\
					"%+010.4f,%+010.4f,%+010.4f,"	\
					"%+010.4f,%+010.4f,%+010.4f,"	\
					"%+010.4f,%+010.4f,%+010.4f,%+010.4f.\r\n"
					
#define FORM_SERIAL	"%+010.4f    %+010.4f    %+010.4f  |  "	\
					"%+010.4f    %+010.4f    %+010.4f  |  "	\
					"%+010.4f    %+010.4f    %+010.4f  |  "	\
					"%+010.4f    %+010.4f    %+010.4f    %+010.4f\r\n"


volatile uint16_t timer0_millis = 0;
volatile uint16_t timer0_micros = 0;

typedef struct {
	float acc[3];
	float gyro[3];
	float mag[3];
	uint8_t encData[5];
	uint8_t flexData[5];
}SENS_DATA_T, * PSENS_DATA_T;

inline uint16_t millis();
void TIMER0_INIT();



int main()
{  
	uint8_t i = 0;
	uint16_t ret = 0;
	unsigned long timeStamp = 0;
	
	float swap;
	uint8_t mpuData[19] = {0,};
	int16_t raw_a[3], raw_g[3], raw_m[3];
	volatile SENS_DATA_T sdt = {0,};
	volatile uint8_t volatile *  pData = &sdt;
	
#ifdef _DEBUG
	float quaternion[4];
	char buffer[256];
#endif
	
	sdt.encData[4] = 5;
	sdt.flexData[4] = -5;
	
	//UART
	UART_INIT(9600);

	//TWI(I2C)
	TWI_INIT();
	
	TIMER0_INIT();
	
	
	sei();
	
	TWI_ReadReg(MPU9250_ADDRESS, WHO_AM_I_MPU9250, &i, 1);
	
	if(i == 0x71)
	{
		ret = MPU9250_init(&(accelBias[0]), &(gyroBias[0]));
		if(ret)
			goto INIT_FAIL;
	}
	else
		goto CONNECT_FAIL;

	TWI_ReadReg(AK8963_ADDRESS, WHO_AM_I_AK8963, &i, 1);
	if(i == 0x48)
	{
		ret = AK8963_init(M_100Hz,&(magBias[0]), &(magScale[0]));
		if(ret)
			goto INIT_FAIL;
	}
	else
		goto CONNECT_FAIL;
	
	//AK8963_Calibrate(&(magBias[0]), &(magScale[0]));
	//UART_printString("Calibarating Start\r\n");
	//_delay_ms(1000);

	while(1)
	{
		if(millis() - timeStamp < 2)
			continue;
		timeStamp = millis();
		ret = readAll(&(mpuData[0]));
		if(ret)
		{
			UART_printString("Read MPU 9250 Fail");
			goto READ_FAIL;
		}

		for(i = 0; i < 3; i++)
		{
			raw_a[i] = ((int16_t)mpuData[i*2] << 8) | mpuData[i*2 + 1];
			raw_g[i] = ((int16_t)mpuData[i*2 + 6] << 8) | mpuData[i*2 + 6 + 1];
			raw_m[i] = ((int16_t)mpuData[i*2 + 12] << 8) | mpuData[i*2 + 12 + 1];
		}
		
		sdt.acc[0] = (float)raw_a[0] * A_RES - accelBias[0];
		sdt.acc[1] = (float)raw_a[1] * A_RES - accelBias[1];
		sdt.acc[2] = (float)raw_a[2] * A_RES - accelBias[2];
		
		sdt.gyro[0] = (float)raw_g[0] * G_RES;
		sdt.gyro[1] = (float)raw_g[1] * G_RES;
		sdt.gyro[2] = (float)raw_g[2] * G_RES;
		
		sdt.mag[0] = (float)raw_m[0] * M_RES - magBias[0];
		sdt.mag[1] = (float)raw_m[1] * M_RES - magBias[1];
		sdt.mag[2] = (float)raw_m[2] * M_RES - magBias[2];
		
		sdt.mag[0] *= magScale[0];
		sdt.mag[1] *= magScale[1];
		sdt.mag[2] *= magScale[2];

		sdt.acc[0] = 0 - sdt.acc[0];
		
		sdt.gyro[0] = sdt.gyro[0] * (3.141592f/180.0f);
		sdt.gyro[1] = sdt.gyro[1] * (3.141592f/180.0f);
		sdt.gyro[2] = sdt.gyro[2] * (3.141592f/180.0f);
		
		// Change magnetometer axis direction to fit acc/gyro axis
		// See MPU9250 data sheet's assembly part
		swap = sdt.mag[1];
		sdt.mag[1] = sdt.mag[0];
		sdt.mag[2] = 0.0f - sdt.mag[2];
		sdt.mag[0] = swap;

#ifdef _DEBUG
		MahonyAHRSupdate(&(sdt.gyro[0]), &(sdt.acc[0]), &(sdt.mag[0]), &(quaternion[0]));

		sprintf(buffer, 
			FORM_SERIAL,	// or FORM_CSV
			sdt.acc[0], sdt.acc[1], sdt.acc[2],
			sdt.gyro[0], sdt.gyro[1], sdt.gyro[2],
			sdt.mag[0], sdt.mag[1], sdt.mag[2],
			quaternion[0], quaternion[1], quaternion[2], quaternion[3]);

		UART_printString(buffer);
#endif

		// ACC, GYRO, MAG
		// ** check order **
		// ** Accelerometer MUST send at first
		for(ret = 0, pData = (uint8_t *)&(sdt.acc[0]); ret < 3; ret++)
		{
			for(i = 0; i < 3; i++)
			{
				mpuData[4*i + 0] = *(pData++);
				mpuData[4*i + 1] = *(pData++);
				mpuData[4*i + 2] = *(pData++);
				mpuData[4*i + 3] = *(pData++);
			}
			
			for(i = 0; i < 12; i++)
				USART_Transmit(mpuData[i]);
		}
		
		// ENCODER, FLEX
		// ** check order **
		// ** Encoder MUST send at first
		for(i = 0; i < 2; i++)
		{
			mpuData[5*i + 0] = *(pData++);
			mpuData[5*i + 1] = *(pData++);
			mpuData[5*i + 2] = *(pData++);
			mpuData[5*i + 3] = *(pData++);
			mpuData[5*i + 4] = *(pData++);
		}
		
		for(i = 0; i < 10; i++)
			USART_Transmit(mpuData[i]);
	} 
READ_FAIL:
CONNECT_FAIL:
INIT_FAIL:
	UART_printString("ShutDown\r\n");
	return 0;
} 

void TIMER0_INIT()
{
	TCCR0B |= (1<<CS01) | (1<<CS00);	// 64 prescale
	TIMSK0 |= (1<<TOIE0);
}

inline uint16_t millis()
{
	uint16_t m;
	uint8_t oldSREG = SREG;
	
	cli();
	
	m = timer0_millis;
	SREG = oldSREG;
	return m;
}

ISR(TIMER0_OVF_vect)
{
	unsigned long m = timer0_millis;
	unsigned int f = timer0_micros;
	
	m += MILLIS_INCREMENT_PER_OVERFLOW;
	f += MICROS_INCREMENT_PER_OVERFLOW;
	
	m += (f/1000);
	f = f%1000;
	
	timer0_millis = m;
	timer0_micros = f;
}
