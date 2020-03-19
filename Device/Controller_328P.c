#define F_CPU 16000000UL

#include <avr/io.h> 
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include <stdio.h>

//#include "MadgwickAHRS.h"
#include "MahonyAHRS.h"
#include "UART.h"
#include "mpu9250.h"
#include "base85.h"

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


#define ADC_READY		0x01
#define ADC_COMPLETE	0x02
#define ADC_READING		0x04


// Debug string format
// check project options for using sprintf
//#define _DEBUG	// Uncomment this line to use serial debug
#define FORM_CSV	"%+010.4f,%+010.4f,%+010.4f,"	\
					"%+010.4f,%+010.4f,%+010.4f,"	\
					"%+010.4f,%+010.4f,%+010.4f,"	\
					"%+010.4f,%+010.4f,%+010.4f,%+010.4f,,\r\n"
					
#define FORM_SERIAL	"%+010.4f    %+010.4f    %+010.4f  |  "	\
					"%+010.4f    %+010.4f    %+010.4f  |  "	\
					"%+010.4f    %+010.4f    %+010.4f  |  "	\
					"%+010.4f    %+010.4f    %+010.4f    %+010.4f\r\n"
					
#define FORM_FREEIMU	"%+010.4f\t%+010.4f\t%+010.4f\t"	\
						"%+010.4f\t%+010.4f\t%+010.4f"		


volatile uint16_t timer0_millis = 0;
volatile uint16_t timer0_micros = 0;
volatile uint8_t idxFinger = 0;
volatile uint16_t timeStampFlex = 0, timeStampEnc = 0, timeStampMPU9250 = 0, curtime = 0;
volatile uint8_t adc_flag = 1;

typedef struct {
	float acc[3];
	float gyro[3];
	float mag[3];
	uint8_t encData[5];
	uint8_t flexData[5];
}SENS_DATA_T, * PSENS_DATA_T;
volatile SENS_DATA_T sdt = {
	.encData[0] = 128,
	.encData[1] = 128,
	.encData[2] = 128,
	.encData[3] = 128,
	.encData[4] = 128,
};

inline uint16_t millis();
void TIMER0_INIT();
void ADC_INIT();
void ENC_INIT(volatile uint8_t* ppb);
void readEncoder(uint8_t ppb, uint8_t cpb, uint8_t idx);
void readFlex(unsigned char channel);

int main()
{  
	uint8_t i = 0;
	uint16_t ret = 0;
	
	float swap;
	uint8_t mpuData[19] = {0,};
	int16_t raw_a[3], raw_g[3], raw_m[3];
	
	uint8_t encIdx = 0;	
	volatile uint8_t ppb[4], cpb;	// save previous/current state of encoder PIND 4,5
	
#ifdef _DEBUG
	float quaternion[4] = {1.0f, };
	char buffer[256];
	
#else
	uint8_t reportDataBuffer[62] = { '[', }; // 2 more space for Start( [ ) / End( ] )
	reportDataBuffer[61] = ']';
#endif
	// flex
	ADC_INIT();
	
	// enc
	ENC_INIT(&ppb[0]);

	//UART
	UART_INIT(115200);

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

	while(1)
	{
		// Read Encoder every single loop
		for(encIdx = 0; encIdx <= 3; encIdx++)
		{
			PORTB = encIdx;;
			_delay_us(1);
			cpb = PIND&0x30;
			if((ppb[encIdx] & 0x20) != (cpb & 0x20)){
				readEncoder(ppb[encIdx], cpb, encIdx);
				ppb[encIdx] = cpb;
			}
		}
		
		// Read Flex if it's ready
		if(adc_flag & ADC_READY)
		{
			readFlex(idxFinger);
			adc_flag = ADC_READING;
		}
		
		// Read Quaternion every 10ms
		curtime = millis();
		if(curtime < timeStampMPU9250 + 10)
			continue;
		
		timeStampMPU9250 = curtime;
READ_RETRY:
		ret = readAll(&(mpuData[0]));
		if(ret & ERR_DATA_READ_FAIL)
			goto READ_RETRY;
		
		

		for(i = 0; i < 3; i++)
		{
			raw_a[i] = ((int16_t)mpuData[i*2] << 8) | mpuData[i*2 + 1];
			raw_g[i] = ((int16_t)mpuData[i*2 + 6] << 8) | mpuData[i*2 + 6 + 1];
			raw_m[i] = ((int16_t)mpuData[i*2 + 12] << 8) | mpuData[i*2 + 12 + 1];
		}
		
		if(!(ret & (ERR_ACC | ERR_DATA_NOT_READY(_ACC))))
		{
			sdt.acc[0] = (float)raw_a[2] * A_RES - accelBias[2];
			sdt.acc[1] = (float)raw_a[1] * A_RES - accelBias[1];
			sdt.acc[2] = 0.0f - ((float)raw_a[0] * A_RES - accelBias[0]);
		}
		//else if(ret & 0x12)
			//UART_printString("ACC Not Ready\r\n");
		
		if(!(ret & (ERR_GYRO | ERR_DATA_NOT_READY(_GYRO))))
		{
			sdt.gyro[0] = (float)raw_g[2] * G_RES;
			sdt.gyro[1] = (float)raw_g[1] * G_RES;
			sdt.gyro[2] = 0.0f - (float)raw_g[0] * G_RES;
			
			sdt.gyro[0] = sdt.gyro[0] * (3.141592f / 180.0f);
			sdt.gyro[1] = sdt.gyro[1] * (3.141592f / 180.0f);
			sdt.gyro[2] = sdt.gyro[2] * (3.141592f / 180.0f);

		}
		//else  if(ret & 0x24)
			//UART_printString("GYRO Not Ready\r\n");
		
		if(!(ret & (ERR_MAG | ERR_DATA_NOT_READY(_MAG))))
		{
			sdt.mag[0] = (float)raw_m[0] * M_RES - magBias[0];
			sdt.mag[1] = (float)raw_m[1] * M_RES - magBias[1];
			sdt.mag[2] = (float)raw_m[2] * M_RES - magBias[2];
			
			sdt.mag[0] *= magScale[0];
			sdt.mag[1] *= magScale[1];
			sdt.mag[2] *= magScale[2];

			// Change magnetometer axis direction to fit acc/gyro axis
			// See MPU9250 data sheet's assembly part
			swap = sdt.mag[1];
			sdt.mag[1] = sdt.mag[0];
			sdt.mag[0] = 0.0f - sdt.mag[2];
			sdt.mag[2] = swap;
		}
		

#ifdef _DEBUG
		MahonyAHRSupdate(&(sdt.gyro[0]), &(sdt.acc[0]), &(sdt.mag[0]), &(quaternion[0]));

		sprintf(buffer, 
			FORM_SERIAL,	// or FORM_CSV
			sdt.acc[0], sdt.acc[1], sdt.acc[2],
			sdt.gyro[0], sdt.gyro[1], sdt.gyro[2],
			sdt.mag[0], sdt.mag[1], sdt.mag[2],
			quaternion[0], quaternion[1], quaternion[2], quaternion[3]);

		UART_printString(buffer);
#else

		btob85(reportDataBuffer+1, (uint8_t*)&sdt, 46);
		UART_printBin(reportDataBuffer, 62);

#endif
	} 
//READ_FAIL:
CONNECT_FAIL:
INIT_FAIL:
	UART_printString("ShutDown\r\n");
	while(1);
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

void ADC_INIT()
{
	ADMUX |= 0x40;
	ADCSRA |= 0x07;
	ADCSRA |= (1 << ADEN);
	ADCSRA |= (1 << ADIE);
	adc_flag = ADC_READY;
}

void ENC_INIT(volatile uint8_t* ppb)
{
	uint8_t cpb, i;
	DDRB |= 0x03;
	DDRD &= ~0x30;
	
	for(i = 0; i < 4; i++)
	{
		PORTB &= ~0x03;
		PORTB |= i;
		_delay_us(5);
		cpb = PIND & 0x30;
		ppb[i] = cpb;
	}
}

void readEncoder(uint8_t ppb, uint8_t cpb, uint8_t idx)
{
	uint8_t a = 0,b = 0;
	if(cpb & 0x20)
	a = 1;
	if(cpb & 0x10)
	b = 1;
	
	if(a == b)
	sdt.encData[idx+1]--;
	else if(a != b)
	sdt.encData[idx+1]++;
}

void readFlex(unsigned char channel)
{
	ADMUX = ((ADMUX & 0xE0) | channel);
	ADCSRA |= (1 << ADSC);
	return ;
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
	
	if(timer0_millis >= timeStampFlex + 1)
	adc_flag = 1;
}

ISR(ADC_vect)
{
	uint16_t adc_value;
	adc_value = ADC;
	
	// finger grip threshold
	//			     thumb  idx  mid ring pinky  
	uint16_t l1[5] = { 60,  70,  80,  30,  50};	// straight
	uint16_t l2[5] = {180, 170, 250, 140, 160};
	uint16_t l3[5] = {230, 260, 320, 250, 220};
	uint16_t l4[5] = {260, 420, 400, 380, 280};
	uint16_t l5[5] = {300, 530, 480, 490, 380}; // grip

	if( 0 <= adc_value && adc_value < l1[idxFinger] )
	sdt.flexData[idxFinger] = 1;
	else if( l1[idxFinger] <= adc_value && adc_value < l2[idxFinger] )
	sdt.flexData[idxFinger] = 2;
	else if( l2[idxFinger] <= adc_value && adc_value < l3[idxFinger] )
	sdt.flexData[idxFinger] = 3;
	else if( l3[idxFinger] <= adc_value && adc_value < l4[idxFinger])
	sdt.flexData[idxFinger] = 4;
	else if( l4[idxFinger] <= adc_value && adc_value < l5[idxFinger])
	sdt.flexData[idxFinger] = 5;
	
	idxFinger++;
	
	if(idxFinger >= 4)	// We can't read pinky. ADC4/5 is equivalent of SDA/SCL
	idxFinger = 0;
	
	timeStampFlex = millis();
	adc_flag = ADC_COMPLETE;
	
}
