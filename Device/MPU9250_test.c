


#define F_CPU 16000000UL

#define FS_SEL 65.5
#define RAD2DEG 57.29578
#define DEG2RAD 0.01745

#include<avr/io.h> 
#include<avr/interrupt.h>
#include<util/delay.h>
#include<math.h>
#include <stdio.h>
#include "MadgwickAHRS.h"
#include "UART.h"
 
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
volatile float roll,pitch,yaw;
//volatile double alpha,beta;
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

	//TIMER0
	//TCCR0B = (1<<CS02)|(1<<CS01);	//256 분주 수정
	//TCNT0 = 256-125;				//125 번 => 0.002s
	//TIMSK0 = (1<<TOIE0);			//수정


	MPU9250_init();
	calibrate();

	//SREG = 0x80; 

	while(1)
	{ 

		get_raw_data();

		//las_angle_gx = roll;	//최근값 누적
		//las_angle_gy = pitch;
		//las_angle_gz = yaw;

		// H데이터 값과 L데이터 값을 합침 16bit
		temp = (a_x_h<<8) | a_x_l;
		//https://m.blog.naver.com/PostView.nhn?blogId=gravity981&logNo=221436232673&proxyReferer=https%3A%2F%2Fwww.google.com%2F
		a_x = (float)(temp / 8192.0)+0.5;
		temp = (a_y_h<<8) | a_y_l;
		a_y = (float)(temp / 8192.0)+0.5;
		temp = (a_z_h<<8) | a_z_l;
		a_z = (float)(temp / 8192.0)-1;
		
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
		m_x = (float)temp*((ASAX+128.000)/256.000)-375;
		temp = (m_y_h<<8) | m_y_l;
		m_y = (float)temp*((ASAY+128.000)/256.000)+75;
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
		
		//UART_printString("a_x = ");	
		sprintf(s1, "%f", a_x);
		UART_printString(s1);
		USART_Transmit('\t');
		
		//UART_printString("a_y = ");
		sprintf(s1, "%f", a_y);
		UART_printString(s1);
		USART_Transmit('\t');
		
		//UART_printString("a_z = ");
		sprintf(s1, "%f", a_z);
		UART_printString(s1);
		USART_Transmit('\t');
		
		//UART_printString("g_x = ");
		USART_Transmit_init4(g_x);
		USART_Transmit('\t');
		//UART_printString("g_y = ");
		USART_Transmit_init4(g_y);
		USART_Transmit('\t');
		//UART_printString("g_z = ");
		USART_Transmit_init4(g_z);
		USART_Transmit('\t');
		
		//UART_printString("m_x = ");
		USART_Transmit_init4(m_x);
		USART_Transmit('\t');
		//UART_printString("m_y = ");
		USART_Transmit_init4(m_y);
		USART_Transmit('\t');
		//UART_printString("m_z = ");
		USART_Transmit_init4(m_z);
		USART_Transmit('\t');
		//USART_Transmit('\r');
		//USART_Transmit('\n');
		
		/*USART_Transmit_init4(roll);
		USART_Transmit('\t');
		USART_Transmit_init4(pitch);
		USART_Transmit('\t');
		USART_Transmit_init4(yaw);
		USART_Transmit('\r');
		USART_Transmit('\n');*/
		
		MadgwickAHRSupdate(g_x, g_y, g_z, a_x, a_y, a_z, m_x, m_y, m_z);
		
		_delay_ms(100);

	} 

} 


ISR(TIMER0_OVF_vect)	//0.002s
{
	dt += 0.002;

	TCNT0 = 256-125;
}




void calibrate()	//초기값 읽기 
{
	int cal = 10;

	for(int i=0; i<cal; i++)	//평균 
	{
		get_raw_data();
	
		temp = (a_x_h<<8) | a_x_l;
		a_x += - temp - 16383;
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

		_delay_ms(100);
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


void MPU9250_init()
{
	/*
	1. 마스터 모드 enable
	2. I2C클럭설정
	3. 마스터모드 딜레이 설정
	4. slave3를 쓰기 모드로 설정하고 지자기센서에 16bit output,Single measurement mode로 설정
	5. 모든 slave샘플 갯수 설정
	6. 지자기 센서의 ASAX, ASAY, ASAZ 값을 읽어옴 (이떄 딜레이는 100ms가 권장사항)
	7. 외부센서 register에 저장된 지자기 센서 데이터 없어짐
	https://blog.naver.com/speedprinse/221192173624
	*/
	
	twi_write(0x6B, 0x03); //[PWR_MGMT_1] sleep 끔, Auto PLL
	twi_write(0x1A, 0x05); //[CONFIG] DLPF 10Hz

	twi_write(0x6A, 0x20); //[USER_CTRL] ,master enable
	twi_write(0x24, 0x0D); //[I2C_MST_CTRL] ,내부 i2c clock 400Hz
	twi_write(0x67, 0x0F); //[I2C_MST_DELAY_CTRL]

	twi_write(0x2E, 0x0C); //[I2C_SLV3_ADDR] AK8963 write
    twi_write(0x2F, 0x0A); //[I2C_SLV3_REG]
    twi_write(0x30, 0x81); //[I2C_SLV3_CTRL]
    twi_write(0x66, 0x11); //[I2C_SLV3_DO]

    twi_write(0x34, 0x13); //[I2C_SLV4_CTRL] 모든 slave 19+1번 샘플
	
	twi_write(0x25, 0x8C); //[I2C_SLV0_ADDR] AK8963 read.
    twi_write(0x26, 0x10); //[I2C_SLV0_REG]
    twi_write(0x27, 0x81); //[I2C_SLV0_CTRL]

	twi_write(0x28, 0x8C); //[I2C_SLV1_ADDR] AK8963 read.
    twi_write(0x29, 0x11); //[I2C_SLV1_REG] 
    twi_write(0x2A, 0x81); //[I2C_SLV1_CTRL]

	twi_write(0x2B, 0x8C); //[I2C_SLV2_ADDR] AK8963 read.
    twi_write(0x2C, 0x12); //[I2C_SLV2_REG]
    twi_write(0x2D, 0x81); //[I2C_SLV2_CTRL]

	_delay_ms(1000);

	ASAX = twi_read(0x49);
	ASAY = twi_read(0x4A);
	ASAZ = twi_read(0x4B);

	_delay_ms(1000);


	twi_write(0x25, 0x8C); //[I2C_SLV0_ADDR] AK8963 read.
    twi_write(0x26, 0x03); //[I2C_SLV0_REG]
    twi_write(0x27, 0xD2); //[I2C_SLV0_CTRL]

	twi_write(0x28, 0x8C); //[I2C_SLV1_ADDR] AK8963 read.
    twi_write(0x29, 0x05); //I2C_SLV1_REG]
    twi_write(0x2A, 0xD2); //[I2C_SLV1_CTRL]

	twi_write(0x2B, 0x8C); //[I2C_SLV2_ADDR] AK8963 read.
    twi_write(0x2C, 0x07); //[I2C_SLV2_REG]
    twi_write(0x2D, 0xD2); //[I2C_SLV2_CTRL]
	
	//**add ysh
	twi_write(0x1b, 0X04); //Gyro +500dps Scale Select: 14 of 55
	twi_write(0x1C, 0X04); //Accel 4G Scale Select: 14 of 55
	//

}








void twi_write(unsigned char address,unsigned char data)
{ 
	TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN) | _BV(TWEA);	//START

	while(!(TWCR & (1<<TWINT))); //TWINT flag 기다림 
	//while((TWSR&0xF8) != 0x08);  //START 상태(08) 기다림  

	TWDR = 0b11010000;			 //AD(1101000)+W(0) 
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
	while( !(TWCR & (1 << TWINT)) ); // 전송 완료 대기
	//while((TWSR&0xF8) != 0x18);  //SLA+W ACK 상태(18) 기다림

	TWDR = address; 			 //register address
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
	while( !(TWCR & (1 << TWINT)) ); // 전송 완료 대기
	//while((TWSR&0xF8) != 0x28);  //Data ACK 상태(28) 기다림 

	TWDR = data; 				 //data
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
	while( !(TWCR & (1 << TWINT)) ); // 전송 완료 대기
	//while((TWSR&0xF8) != 0x28);

	TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN) | _BV(TWEA); //STOP
} 

 

unsigned char twi_read(char address)
{ 
	unsigned char data;

	TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN) | _BV(TWEA);
	while( !(TWCR & (1 << TWINT)) ); // 시작 완료 대기
	
	//while((TWSR&0xF8) != 0x08);  //START 상태(08) 기다림  

	TWDR = 0b11010000;			 //AD(1101000)+W(0) 
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
	while( !(TWCR & (1 << TWINT)) ); // 전송 완료 대기
	
	//while((TWSR&0xF8) != 0x18);  //SLA+W ACK 상태(18) 기다림

	TWDR = address; 			 //register address
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
	while( !(TWCR & (1 << TWINT)) ); // 전송 완료 대기
	
	//while((TWSR&0xF8) != 0x28);  //Data ACK 상태(28) 기다림 

	//TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);	//Repeat START

	TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN) | _BV(TWEA);
	while( !(TWCR & (1 << TWINT)) ); // 시작 완료 대기
	
	//while((TWSR&0xF8) != 0x10);  //Repeat START 상태(08) 기다림

	TWDR = 0b11010001;			 //AD(1101000)+R(1) 
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
	while( !(TWCR & (1 << TWINT)) ); // 전송 완료 대기
	
	//while((TWSR&0xF8) != 0x40);  //SLA+R ACK 상태(40) 기다림 

	TWCR = _BV(TWINT) | _BV(TWEN);
	while( !(TWCR & (1 << TWINT)) ); // 수신 완료 대기
	
	//while((TWSR&0xF8) != 0x58);  //ACK 상태(58) 기다림 

	data = TWDR; 

	TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN) | _BV(TWEA);

	return data; 
}
