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

//#define _DEBUG

// Debug string format
// check project options for using sprintf
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
	uint8_t i2cReadRtyCnt = 0;
	uint16_t ret = 0;
	volatile uint16_t timeStampMPU9250 = 0, curtime = 0;
	
	float swap;
	uint8_t mpuData[19] = {0,};
	int16_t raw_a[3], raw_g[3], raw_m[3];
	SENS_DATA_T sdt = {0,};

	//uint16_t runCnt = 0;
	//uint16_t runTimeStamp = 0;

	sdt.encData[4] = 5;
	sdt.flexData[4] = 5;
	
#ifdef _DEBUG
	float quaternion[4] = {1.0f, };
	char buffer[256];
	
#else
	uint8_t reportDataBuffer[62] = { '[', }; // 2 more space for Start( [ ) / End( ] )
	reportDataBuffer[61] = ']';
#endif
	
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
	//_delay_ms(1000);

	//runTimeStamp = millis();
	while(1)
	{
		curtime = millis();
		//if(curtime - runTimeStamp > 10000)
			//goto INIT_FAIL;
		if(curtime < timeStampMPU9250 + 10)
			continue;
		timeStampMPU9250 = curtime;
READ_RETRY:
		ret = readAll(&(mpuData[0]));
		//runCnt++;
		if(ret & ERR_DATA_READ_FAIL)
		{
			if(i2cReadRtyCnt > 10)
			{
				UART_printString("\r\nRead MPU 9250 Fail");
				goto READ_FAIL;
			}
			++i2cReadRtyCnt;
			goto READ_RETRY;
		}
		i2cReadRtyCnt = 0;

		for(i = 0; i < 3; i++)
		{
			raw_a[i] = ((int16_t)mpuData[i*2] << 8) | mpuData[i*2 + 1];
			raw_g[i] = ((int16_t)mpuData[i*2 + 6] << 8) | mpuData[i*2 + 6 + 1];
			raw_m[i] = ((int16_t)mpuData[i*2 + 12] << 8) | mpuData[i*2 + 12 + 1];
		}
		
		if(!(ret & (ERR_ACC | ERR_DATA_NOT_READY(_ACC))))
		{
			sdt.acc[0] = (float)raw_a[0] * A_RES - accelBias[0];
			sdt.acc[1] = (float)raw_a[1] * A_RES - accelBias[1];
			sdt.acc[2] = (float)raw_a[2] * A_RES - accelBias[2];
			
			sdt.acc[0] = 0 - sdt.acc[0];
		}
		//else if(ret & 0x12)
			//UART_printString("ACC Not Ready\r\n");
		
		if(!(ret & (ERR_GYRO | ERR_DATA_NOT_READY(_GYRO))))
		{
			sdt.gyro[0] = (float)raw_g[0] * G_RES;
			sdt.gyro[1] = (float)raw_g[1] * G_RES;
			sdt.gyro[2] = (float)raw_g[2] * G_RES;
			
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
			sdt.mag[2] = 0.0f - sdt.mag[2];
			sdt.mag[0] = swap;
		}
		//else  if(ret & 0x48)
			//UART_printString("MAG Not Ready\r\n");
		
		//Set Encoder & Flex
		for(i = 0; i < 5; i++)
		{
			sdt.encData[i]++;
			sdt.flexData[i]++;
		}
		

#ifdef _DEBUG
		//MahonyAHRSupdate(&(sdt.gyro[0]), &(sdt.acc[0]), &(sdt.mag[0]), &(quaternion[0]));

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
READ_FAIL:
CONNECT_FAIL:
INIT_FAIL:
	//UART_printString("ShutDown\r\nrunCnt: ");
	//UART_printUINT(runCnt);
	//UART_printString("\r\n");
	//while(1);
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

/*
[zK4JO J9q#P Kh2H) z7;FX z!xe` J%K}S !~(iA #df*^ MK%0u B`001 p0]
[zKB2k yaE6G KhC}W z6wJ{ z!(}y J$@<N !v9M( #n2_^ MB&nI DJdx_ DJdx~ 001s2]
[zK1{j yQBa3 Kh0hK z7*yB z!zs^ J&d=O M1*zd #h<ep MIQD9 Dk>@} Dk>^3 001v4]
[zK8$( yGQ@< Kh6#Q z6`gP z!(}y J$w6G M1*zd #h<ep MIQD9 D=RB2 D=RB7 001y6]
[zKQ?* yVL*p Kh53# z8}tl z!wX{ J!UFR M77>d #q@ju MW7LA EG#T6 EG#TB 001#8]
[zJ<U1 JDdOr Kh2H) z7AS< z!&?z J%2$R M77>d #q@ju MW7LA EiElA EiElF 001&A]
[zKK8l JInwG Kh8b= z6>k& z!%|= J&Y^% !~(iA #oR)g MPvy# E-o%E E-o%J 001*C]
[zJ<U1 y3haM KhF*R z6t$x z!yY( J(m=H !~(iA #oR)g MPvy# FE1}I FE1}N 001;E]
[zKB2k x{v?h Kg^x} z7WpI z!#Te J%U=W !u&L^ #l#6T MK%0u FfcGM FfcGR 001>G]
[zKXy6 yC?tf Kh915 z7lb& z!z)5 J%Z*? !u&L^ #l#6T MK%0u F)=YQ F)=YV 001^I]
[zK4JO yGQ@< KhBN* z7WpI z!%uo J%G2* MCV4d #n2_^ MNI<H GBPqU GBPqZ 001{K]
[zJve# x*z|* Kg*r| z7x*7 z!#Te Jy2Od MCV4d #n2_^ MNI<H Gcz+Y Gcz+d 001~M]
[zJ~w& xc~kD Kg?bJ z6`gP z!#)? J##@- L?#;q #q@ju MF;Pm G&D3c G&D3h 0022O]
[zIp%n yb=HW Kg|99 z75XT z!qVb J%`To L?#;q #q@ju MF;Pm H8nLg H8nLl 0025Q]
[zJtH~ JL~`m KhDkm z6^23 z!(iO Jza1} #AS;B #n2_^ MSBmP Ha0dk Ha0dp 0028S]
[zJovf yR-lJ KhN#{ z7vS+ z!lZ^ J&sy; #AS;B #n2_^ MSBmP H#avo H#avt 002BU]
[zIgxm JgWc! Kgj+5 z7FOX z!^gR J*l6_ !~(iA #s*rn MPvy# I5;>s I5;>x 002EW]
[zKQ?* JInwG KhN#{ z6o*E z!=1J J$(|K !~(iA #s*rn MPvy# IXO8w IXO8# 002HY]
[zJve# I~V{E Kh7=w z7AS< z!-W+ J&_cd #RGp@ #u9gD MK%0u IyyQ! IyyQ( 002Ka]
[zJmY! yGQ@< Kg<38 z7Z6e z!$>> J&_cd #RGp@ #u9gD MK%0u J3Bi& J3Bi- 002Nc]
[zJ~w& yOaO; Kh2%~ z7Rtx z!#|3 J%+cl MGRMt #l#6T MPvy# JUl!+ JUl!> 002Qe]
[zK8$( yMzDu Kg-?z z7qXP z!yl_ J#4kn MGRMt #l#6T MPvy# Jv}`= Jv}`_ 002Tg]
[zKQ?* yGQ@< Kg@mp z6DyM z!uXA J&xuW Lxz-Y #kcH% !^)wo K0ZD^ K0ZD} 002Wi]
[zJ)*h JT3qP Kg`|! z68$# z!<k) J%2$R Lxz-Y #kcH% !^)wo KR-V| KR-W2 002Zk]
[zKnnT yGQ@< Kh915 z6+pM z!&bP J(m=H Lxz-Y #u$ab MSBmP KtMo1 KtMo6 002cm]
[zKVbR yYv6} KhHh> z7$$p z!!z@ J$Ig1 Lxz-Y #u$ab MSBmP K|w)5 K|w)A 002fo]
[zJtH~ yjlPF Kh8b= z7FOX z!zSs J%jy` MCV4d #sExO MB&nI LPA19 LPA1E 002iq]
[zKK8l yW{`( Kg@0Z z6>k& z!vQ| J)!*s MCV4d #sExO MB&nI LqkJD LqkJI 002ls]
[zJ|a2 yMzDu Kg~V< z6hXX z!&B1 J(m=H !~(iA #q@ju MYk8v L_|bH L_|bM 002ou]
[zJ@>i JT?FX Kg|99 z7ggM z!z2U J(h^w !~(iA #q@ju MYk8v MMXtL MMXtQ 002rw]
[zKcKn y9fX9 Kg`|! z6j<t z!&OD J&5Ks !~(iA #q@ju M6<=A Mn*<P Mn*<U 002uy]
[zJ|a2 JPrT^ Kg^B( z84gI z!$NR J%>Y6 !~(iA #q@ju M6<=A M@L6T M@L6Y 002x!]
[zK4JO yMzDu Kh16a z7WpI z!#)? J&sy; M1*zd #kcH% !^)wo NJvOX NJvOc 002!$]
[zJ>q% yYv6} Kg=Ee z7WpI z!%uo J#OSv M1*zd #kcH% !^)wo Nl8gb Nl8gg 002%&]
[zKuWp yMzDu Kg}Kf z6`gP z!v0w J&xuW !~(iA #l#6T MF;Pm N=iyf N=iyk 002))]
[zKFm4 JO%&+ Kg~`4 z6hXX z!vD+ J$Ig1 !~(iA #l#6T MF;Pm OG`^j OG`^o 002-+]
[zJkB} Ilusb Kh7=w z6e^B z!yz6 J(<p* #T+w? #tdl< MU;*; OiWBn OiWBs 002=;]
[zJ!1L yMzDu KhLfH z7bk! z!#|3 J#iA$ #T+w? #tdl< MU;*; O-)Tr O-)Tw 002@=]
[zJR~{ yW{`( Kg|vP z6+pM z!yY( J%P^< #MPEf #pqv7 MIQD9 PEJlv PEJl! 002`?]
[zJ&k$ xc~kD Kg`|! z77<p z!!9T J&AGD #MPEf #pqv7 MIQD9 Pft%z Pft%& 002}^]
[zJ|a2 I$!`m Kh2H) z6^23 z!*wN J&$p? L?#;q #q@ju MF;Pm P*6}% P*6}+ 0031`]
[zJ-7M Ilusb Kh3@V z7=tt z!(va J&K7H L?#;q #q@ju MF;Pm QBhG* QBhG= 0034|]
[zK4JO I<NpB KhACb z77<p z!)@m J%B7P !u&L^ #l#6T MF;Pm Qc_Y< Qc_Y^ 0037~]
[zK8$( Ilusb Kh3TF z6NpQ z!%HE J%`To !u&L^ #l#6T MF;Pm Q&Uq@ Q&Uq| 003B1]
[zJ)*h xc~kD Kh3TF z6hXX z!$!# J(#y% !u&L^ #l#6T MF;Pm R8&+{ R8&-1 003E3]
[zJ@>i Ilusb Kh3TF z6||m z!z)5 J%tp~ L?#;q #jDTG MK%0u RaI40 RaI45 003H5]
[zKQ?* JFEZ* Kg_-U z7WpI z!w7v J&K7H L?#;q #jDTG MK%0u R#sM4 R#sM9 003K7]

*/