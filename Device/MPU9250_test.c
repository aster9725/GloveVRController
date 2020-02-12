


#define F_CPU 16000000UL

#define FS_SEL 131

#include<avr/io.h> 
#include<avr/interrupt.h>
#include<util/delay.h>
#include<math.h>
#include <stdio.h>
#include "MadgwickAHRS.h"
#include "UART.h"
#include "twi.h"
 
void twi_write(unsigned char address,unsigned char data);
unsigned char twi_read(char addressr);
void USART_Transmit(unsigned char tx_data);
void USART_Transmit_init4(int data);
void get_raw_data();
void calibrate();
void MPU9250_init();
void UART_printString(char *str);


volatile double dt = 0.000;
volatile int temp;
volatile unsigned char a_x_l,a_x_h,a_y_l,a_y_h,a_z_l,a_z_h;
volatile unsigned char g_x_l,g_x_h,g_y_l,g_y_h,g_z_l,g_z_h;
volatile unsigned char m_x_l,m_x_h,m_y_l,m_y_h,m_z_l,m_z_h;
volatile int bas_g_x,bas_g_y,bas_g_z;
volatile int a_x,a_y,a_z;
volatile int g_x,g_y,g_z;
volatile int m_x,m_y,m_z;
volatile float ASAX,ASAY,ASAZ;

#define I2C_SCL PC5
#define I2C_SDA PC4

int main()
{  

	//UART
	UCSR0A |= _BV(U2X0);
	
	UBRR0H = 0x00;
	UBRR0L = 207;				//9600
	
	UCSR0C |= 0x06;

	UCSR0B |= _BV(TXEN0);

	//TWI(I2C)
	DDRC |= (1 << I2C_SCL); // SCL 핀을 출력으로 설정
	DDRC |= (1 << I2C_SDA); // SDA 핀을 출력으로 설정
	TWBR = 12; // I2C 클록 주파수 설정 400KHz
	TWCR = (1 << TWEN) | (1 << TWEA); // I2C 활성화, ACK 허용

	MPU9250_init();

	while(1)
	{ 

		get_raw_data();
		
		temp = (a_x_h<<8) | a_x_l;
		a_x = temp;
		temp = (a_y_h<<8) | a_y_l;
		a_y = temp;
		temp = (a_z_h<<8) | a_z_l;
		a_z = temp;
		
		temp = (g_x_h<<8) | g_x_l;
		g_x = temp;
		temp = (g_y_h<<8) | g_y_l;
		g_y = temp;
		temp = (g_z_h<<8) | g_z_l;
		g_z = temp;

		temp = (m_x_h<<8) | m_x_l;
		m_x = temp;
		temp = (m_y_h<<8) | m_y_l;
		m_y = temp;
		temp = (m_z_h<<8) | m_z_l;
		m_z = temp;
		
		USART_Transmit('S');
		USART_Transmit(a_x_h);
		USART_Transmit(a_x_l);
		USART_Transmit(a_y_h);
		USART_Transmit(a_y_l);
		USART_Transmit(a_z_h);
		USART_Transmit(a_z_l);
		
		USART_Transmit(g_x_h);
		USART_Transmit(g_x_l);
		USART_Transmit(g_y_h);
		USART_Transmit(g_y_l);
		USART_Transmit(g_z_h);
		USART_Transmit(g_z_l);
		
		USART_Transmit(m_x_h);
		USART_Transmit(m_x_l);
		USART_Transmit(m_y_h);
		USART_Transmit(m_y_l);
		USART_Transmit(m_z_h);
		USART_Transmit(m_z_l);
		USART_Transmit('E');
		
		/*
		USART_Transmit_init4(g_x);
		USART_Transmit_init4(g_y);
		USART_Transmit_init4(g_z);
		
		USART_Transmit_init4(g_x);
		USART_Transmit_init4(g_y);
		USART_Transmit_init4(g_z);
		
		USART_Transmit_init4(m_x);
		USART_Transmit('\t');
		USART_Transmit_init4(m_y);
		USART_Transmit('\t');
		USART_Transmit_init4(m_z);
		USART_Transmit('\t');
		USART_Transmit('E');
		*/
		/*temp = (m_x_h<<8) | m_x_l;
		m_x = (float)temp*((ASAX+128.000)/256.000)-380;
		temp = (m_y_h<<8) | m_y_l;
		m_y = (float)temp*((ASAY+128.000)/256.000)+85;
		temp = (m_z_h<<8) | m_z_l;
		m_z = (float)temp*((ASAZ+128.000)/256.000)-325;
		
		USART_Transmit_init4(m_x);
		USART_Transmit('\t');
		USART_Transmit_init4(m_y);
		USART_Transmit('\t');
		USART_Transmit_init4(m_z);
		
		USART_Transmit('\n');*/
		
		//_delay_ms(1000);

	} 

} 


void calibrate()	//초기값 읽기 
{
	int cal = 10;

	for(int i=0; i<cal; i++)	//평균 
	{
		get_raw_data();
		temp = (g_x_h<<8) | g_x_l;
		g_x += temp;
		temp = (g_y_h<<8) | g_y_l;
		g_y += temp;
		temp = (g_z_h<<8) | g_z_l;
		g_z += temp;

		_delay_ms(100);
	}	
	
	bas_g_x = g_x;
	bas_g_y = g_y;
	bas_g_z = g_z;

}

void get_raw_data()
{
	a_x_h = twi_read(0x3B);		//x축 가속도
	a_x_l = twi_read(0x3C);
	a_y_h = twi_read(0x3D);		//y축 가속도 
	a_y_l = twi_read(0x3E);		
	a_z_h = twi_read(0x3F);		//z축 가속도 
	a_z_l = twi_read(0x40);	
		
	g_x_h = twi_read(0x43);		//x축 각속도 
	g_x_l = twi_read(0x44);		
	g_y_h = twi_read(0x45);		//y축 각속도 
	g_y_l = twi_read(0x46);		
	g_z_h = twi_read(0x47);		//z축 각속도 
	g_z_l = twi_read(0x48);		

	m_x_h = twi_read(0x49);		//x축 콤파스
	m_x_l = twi_read(0x4A);
	m_y_h = twi_read(0x4B);		//y축 콤파스
	m_y_l = twi_read(0x4C);
	m_z_h = twi_read(0x4D);		//z축 콤파스
	m_z_l = twi_read(0x4E);

}

