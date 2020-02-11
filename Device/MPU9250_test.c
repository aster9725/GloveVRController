


#define F_CPU 16000000UL

#define FS_SEL 131//16.384
//#define RAD2DEG 57.29578
//#define DEG2RAD 0.01745

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
volatile float bas_a_x,bas_a_y,bas_a_z;
volatile float bas_g_x,bas_g_y,bas_g_z;
volatile float a_x,a_y,a_z;
volatile float g_x,g_y,g_z;
volatile float m_x,m_y,m_z;
volatile float mx,my,mz;
volatile float r_mx,r_my;
volatile float angle_ax,angle_ay,angle_az;
volatile float angle_gx,angle_gy,angle_gz;
volatile float las_angle_gx,las_angle_gy,las_angle_gz;
volatile float yaw_g,yaw_m;
volatile float ASAX,ASAY,ASAZ;
volatile float quaternion[4];

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

		// H데이터 값과 L데이터 값을 합침 16bit
		temp = (a_x_h<<8) | a_x_l;
		//https://m.blog.naver.com/PostView.nhn?blogId=gravity981&logNo=221436232673&proxyReferer=https%3A%2F%2Fwww.google.com%2F
		a_x = (float)(temp / 8192.0);
		temp = (a_y_h<<8) | a_y_l;
		a_y = (float)(temp / 8192.0);
		temp = (a_z_h<<8) | a_z_l;
		a_z = (float)(temp / 8192.0);
		
		temp = (g_x_h<<8) | g_x_l;
		g_x = (float)temp;
		temp = (g_y_h<<8) | g_y_l;
		g_y = (float)temp;
		temp = (g_z_h<<8) | g_z_l;
		g_z = (float)temp;
		
		g_x = (g_x - bas_g_x) /FS_SEL;
		g_y = (g_y - bas_g_y) /FS_SEL;
		g_z = (g_z - bas_g_z) /FS_SEL;
		
		temp = (m_x_h<<8) | m_x_l;
		m_x = (float)temp*((ASAX+128.000)/256.000)-380;
		temp = (m_y_h<<8) | m_y_l;
		m_y = (float)temp*((ASAY+128.000)/256.000)+85;
		temp = (m_z_h<<8) | m_z_l;
		m_z = (float)temp*((ASAZ+128.000)/256.000)-325;
		
		/*temp = (a_x_h<<8) | a_x_l;
		a_x = - temp;
		temp = (a_y_h<<8) | a_y_l;
		a_y = - temp;
		temp = (a_z_h<<8) | a_z_l;
		a_z = temp;
		temp = (g_x_h<<8) | g_x_l;
		g_x = temp;
		temp = (g_y_h<<8) | g_y_l;
		g_y = temp;
		temp = (g_z_h<<8) | g_z_l;
		g_z = temp;

		temp = (m_x_h<<8) | m_x_l;
		m_x = (double)temp*((ASAX+128.000)/256.000);
		temp = (m_y_h<<8) | m_y_l;
		m_y = (double)temp*((ASAY+128.000)/256.000);
		temp = (m_z_h<<8) | m_z_l;
		m_z = (double)temp*((ASAZ+128.000)/256.000);

		g_x = (g_x - bas_g_x)/FS_SEL;
		g_y = (g_y - bas_g_y)/FS_SEL;
		g_z = (g_z - bas_g_z)/FS_SEL;
		
		angle_ax = atan(-1.000*a_y/sqrt(pow(a_x,2) + pow(a_z,2)))*RAD2DEG;
		angle_ay = atan(a_x/sqrt(pow(a_y,2) + pow(a_z,2)))*RAD2DEG;

		angle_gx = g_x*dt + las_angle_gx;
		angle_gy = g_y*dt + las_angle_gy;
		angle_gz = g_z*dt + las_angle_gz;

		dt = 0.000;

		alpha = 0.96;
		roll = alpha*angle_gx + (1.000 - alpha)*angle_ax;
		pitch = alpha*angle_gy + (1.000 - alpha)*angle_ay;
		
		mx = (m_x-15)*0.9;
		my = m_y+65;
		mz = (m_z+125)*0.53;
		r_mx = (cos(70.000*DEG2RAD)*mx + sin(70.000*DEG2RAD)*mz) - 141;
		r_my = (cos(-70.000*DEG2RAD)*my - sin(-70.000*DEG2RAD)*mz) - 143;

		yaw_g = angle_gz;
		yaw_m = -atan2(r_mx,r_my)*RAD2DEG;		//+- 10도 안에서만 사용가능
		
		beta = 0.92;
		if( (roll<10) && (roll>-10) && (pitch<10) && (pitch>-10) )
		yaw = beta*yaw_g + (1.000 - beta)*yaw_m;
		else
		yaw = yaw_g;*/
		
		char s1[30] = {0,};
		
		sprintf(s1, "%.06f\t", a_x);
		UART_printString(s1);
		USART_Transmit(',');
		sprintf(s1, "%.06f\t", a_y);
		UART_printString(s1);
		USART_Transmit(',');
		sprintf(s1, "%.06f\t", a_z);
		UART_printString(s1);
		USART_Transmit(',');
		
		USART_Transmit_init4(g_x);
		USART_Transmit(',');
		USART_Transmit_init4(g_y);
		USART_Transmit(',');
		USART_Transmit_init4(g_z);
		USART_Transmit(',');
		
		USART_Transmit_init4(m_x);
		USART_Transmit(',');
		USART_Transmit_init4(m_y);
		USART_Transmit(',');
		USART_Transmit_init4(m_z);
		USART_Transmit(',');
		
		
		MadgwickAHRSupdate(g_x, g_y, g_z, a_x, a_y, a_z, m_x, m_y, m_z);
		/*quaternion = MadgwickAHRSupdate(g_x, g_y, g_z, a_x, a_y, a_z, m_x, m_y, m_z);
		USART_Transmit_init4(quaternion[0]);
		USART_Transmit(',');
		USART_Transmit_init4(quaternion[1]);
		USART_Transmit(',');
		USART_Transmit_init4(quaternion[2]);
		USART_Transmit(',');
		USART_Transmit_init4(quaternion[3]);
		USART_Transmit('\r');
		USART_Transmit('\n');*/
		
		_delay_ms(10);

	} 

} 


void calibrate()	//초기값 읽기 
{
	int cal = 10;

	for(int i=0; i<cal; i++)	//평균 
	{
		get_raw_data();
	
		temp = (a_x_h<<8) | a_x_l;
		a_x += - temp;
		temp = (a_y_h<<8) | a_y_l;
		a_y += - temp;
		temp = (a_z_h<<8) | a_z_l;
		a_z += temp;
		temp = (g_x_h<<8) | g_x_l;
		g_x += temp;
		temp = (g_y_h<<8) | g_y_l;
		g_y += temp;
		temp = (g_z_h<<8) | g_z_l;
		g_z += temp;

		_delay_ms(1000);
	}	
	
	a_x /= cal;
	a_y /= cal;
	a_z /= cal;
	g_x /= cal;
	g_y /= cal;
	g_z /= cal;

	bas_a_x = a_x;	//초기 값으로 저장 
	bas_a_y = a_y;
	bas_a_z = a_z;
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

