


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


volatile uint16_t timer0_millis = 0;
volatile uint16_t timer0_micros = 0;

typedef struct {
	float acc[3];
	float gyro[3];
	float mag[3];
	uint8_t encData[5];
	uint8_t flexData[5];
	uint8_t dataEnd;
}SENS_DATA_T, * PSENS_DATA_T;

inline uint16_t millis();
void TIMER0_INIT();

int main()
{  
	uint8_t i = 0;
	uint8_t id = 0;
	uint16_t ret = 0;
	
	uint8_t mpuData[19] = {0,};
	SENS_DATA_T sdt = {0,};
	int16_t raw_a[3], raw_g[3], raw_m[3];
	float quaternion[4];
		
	uint8_t* dp;
		
	float temp;
		
	char buffer[256];
	
	sdt.encData[4] = 5;
	sdt.flexData[4] = 5;
	
	//UART
	UART_INIT(9600);

	//TWI(I2C)
	TWI_Init();
	
	TIMER0_INIT();
	
	
	sei();
	
	TWI_ReadReg(MPU9250_ADDRESS, WHO_AM_I_MPU9250, &id, 1);
	
	if(id == 0x71)
	{
		ret = MPU9250_init(&(accelBias[0]), &(gyroBias[0]));
		if(ret)
			goto INIT_FAIL;
	}
	else
		goto CONNECT_FAIL;

	TWI_ReadReg(AK8963_ADDRESS, WHO_AM_I_AK8963, &id, 1);
	if(id == 0x48)
	{
		ret = AK8963_init(M_100Hz,&(magBias[0]), &(magScale[0]));
		if(ret)
			goto INIT_FAIL;
	}
	else
		goto CONNECT_FAIL;
		
	//UART_printString("ID MATCH");
	
	//AK8963_Calibrate(&(magBias[0]), &(magScale[0]));
	//UART_printString("Calibarating Start\r\n");
	//_delay_ms(1000);

	while(1)
	{
		ret = readAll(&(mpuData[0]));

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

		//acc[0] = 0 - acc[0];
		//
		//gyro[0] = gyro[0] * (3.141592f/180.0f);
		//gyro[1] = (0.0f - gyro[1]) * (3.141592f/180.0f);
		//gyro[2] = (0.0f - gyro[2]) * (3.141592f/180.0f);
		//
		//temp = mag[1];
		//mag[1] = 0.0f - mag[0];
		//mag[0] = temp;
				//
		//MahonyAHRSupdate(&(gyro[0]), &(acc[0]), &(mag[0]), &(quaternion[0]));
		
		// format csv
		//sprintf(buffer, "%+010.4f,%+010.4f,%+010.4f,,%+010.4f,%+010.4f,%+010.4f,,%+010.4f,%+010.4f,%+010.4f,,%+010.4f,%+010.4f,%+010.4f,%+010.4f,,,,\r\n", 
			//acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2], mag[0], mag[1], mag[2], quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
			
		// format serial
		//sprintf(buffer, "%+010.4f    %+010.4f    %+010.4f  |  %+010.4f    %+010.4f    %+010.4f  |  %+010.4f    %+010.4f    %+010.4f  |  %+010.4f    %+010.4f    %+010.4f    %+010.4f\r\n",
			//acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2], mag[0], mag[1], mag[2], quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
		
		// format raw data
		//sprintf(buffer, "%8d\t%8d\t%8d\r\n",raw_m[0], raw_m[1], raw_m[2]);
		//
		 //UART_printString(buffer);
		
		

		//USART_Transmit('S');
		for(dp = (uint8_t*)&(sdt.acc[0]); dp < (uint8_t*)&(sdt.dataEnd); dp++)
		{				 
			USART_Transmit(*dp);
		}
		//USART_Transmit('E');
		
		_delay_ms(10);
	} 

CONNECT_FAIL:
INIT_FAIL:
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
