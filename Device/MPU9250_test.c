


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
 
void calibrate();


volatile int temp;
volatile int bas_g_x,bas_g_y,bas_g_z;
volatile int a_x,a_y,a_z;
volatile int g_x,g_y,g_z;
volatile int m_x,m_y,m_z;
volatile float ASAX,ASAY,ASAZ;

#define I2C_SCL PC5
#define I2C_SDA PC4

int main()
{  

	UART_INIT();

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
		
		USART_Transmit(m_y_h);
		USART_Transmit(m_y_l);
		USART_Transmit(m_z_h);
		USART_Transmit(m_z_l);
		
		USART_Transmit(m_x_h);
		USART_Transmit(m_x_l);
		USART_Transmit(ASAX);
		USART_Transmit(ASAY);
		USART_Transmit(ASAZ);
		
		USART_Transmit('E');
		
		_delay_ms(2);
	} 

} 
