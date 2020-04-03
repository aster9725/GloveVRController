#define F_CPU 16000000UL

#include <avr/io.h> 
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

//#include "MadgwickAHRS.h"
#include "MahonyAHRS.h"
#include "UART.h"
#include "mpu9250.h"
#include "base85.h"

float   gyroBias[3] = {0.96, -0.21, 0.12}, accelBias[3] = {0.00299, -0.00916, 0.00952};
float   magBias[3] = {71.04, 122.43, -36.90}, magScale[3]  = {1.01, 1.03, 0.96};

volatile uint8_t encCnt[4] = {128, 128, 128, 128};
volatile uint8_t count = 0;
volatile uint8_t idxFinger = 0;
volatile uint16_t timeStampFlex = 0;
volatile uint8_t adc_flag = 1;
volatile uint16_t curtime;
volatile float swap;


#define PRESCALE_VALUE				64
#define CLOCKS_PER_MICRO			(F_CPU / 1000000L)
#define CLOCKS_TO_MICROSECONDS(a)	((a) / CLOCKS_PER_MICRO)
#define MICROSECONDS_PER_TIMER0_OVERFLOW	\
(CLOCKS_TO_MICROSECONDS(PRESCALE_VALUE * 256))
#define MILLIS_INCREMENT_PER_OVERFLOW		\
(MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)
#define MICROS_INCREMENT_PER_OVERFLOW		\
(MICROSECONDS_PER_TIMER0_OVERFLOW % 1000)

#define _DEBUG

// Debug string format
// check project options for using sprintf
#define FORM_CSV	"%+010.4f,%+010.4f,%+010.4f,"	\
					"%+010.4f,%+010.4f,%+010.4f,"	\
					"%+010.4f,%+010.4f,%+010.4f,"	\
					"%+010.4f,%+010.4f,%+010.4f,%+010.4f,,\r\n"
					
#define FORM_SERIAL	"DEL:10.0"	\
					"#ENC:%d,%d,%d,%d,"	\
					"#FLEX:%d,%d,%d,%d,%d"	\
					"#FILL:%f,%f,%f,%f\r\n"
					
#define FORM_FREEIMU	"%+010.4f\t%+010.4f\t%+010.4f\t"\
						"%+010.4f\t%+010.4f\t%+010.4f"		


volatile uint16_t timer0_millis = 0;
volatile uint16_t timer0_micros = 0;

typedef struct {
	volatile float acc[3];
	volatile float gyro[3];
	volatile float mag[3];
	volatile uint8_t encData[5];
	volatile uint8_t flexData[5];
}SENS_DATA_T, * PSENS_DATA_T;
volatile SENS_DATA_T sdt = {0,};

inline uint16_t millis();
void TIMER0_INIT();


ISR(ADC_vect)
{
	uint16_t adc_value;
	adc_value = ADC;
					
	uint16_t l1[5] = {60, 70, 80, 30, 50};
	uint16_t l2[5] = {180, 170, 250, 140, 160};
	uint16_t l3[5] = {230, 260, 320, 250, 220};
	uint16_t l4[5] = {260, 420, 400, 380, 280};
	uint16_t l5[5] = {300, 530, 480, 490, 380};

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
			
	if(idxFinger >= 4)
		idxFinger = 0;
	
	timeStampFlex = millis();
	adc_flag = 0x02;
	
}



void ADC_INIT()
{
	ADMUX |= 0x40;
	ADCSRA |= 0x07;
	ADCSRA |= (1 << ADEN);
	ADCSRA |= (1 << ADIE);
}


void readEncoder(uint8_t ppb, uint8_t cpb, uint8_t idx)
{
	uint8_t a = 0,b = 0;
	if(cpb & 0x08)
	a = 1;
	if(cpb & 0x04)
	b = 1;
	
	if(a == b)
	encCnt[idx]--;
	else if(a != b)
	encCnt[idx]++;
}

void read_flex(unsigned char channel)
{
	ADMUX = ((ADMUX & 0xE0) | channel);
	ADCSRA |= (1 << ADSC);
	return ;
}

int main()
{  
	uint8_t i = 0;
	//uint8_t i2cReadRtyCnt = 0;
	uint16_t ret = 0;
	//volatile uint16_t timeStampMPU9250 = 0; //, curtime = 0;
	//volatile uint16_t timeStampEnc = 0;
	volatile uint8_t ppb[4], cpb;

	//volatile float swap;
	uint8_t encIdx = 0;	
	uint8_t mpuData[19] = {0,};
	int16_t raw_a[3], raw_g[3], raw_m[3];
	

	//uint16_t runCnt = 0;
	//uint16_t runTimeStamp = 0;
	
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
	
	
	//////////////////////////////////////////////////////////////////////////
	// flex
	ADC_INIT();
	
	// enc
	

	DDRB |= 0x03;
	DDRD &= ~0x0C;
	
	cpb = PIND & 0x0C;
	ppb[0] = ppb[1] = ppb[2] = ppb[3] = cpb;	
	
	//////////////////////////////////////////////////////////////////////////
	sei();
	
	TWI_ReadReg(MPU9250_ADDRESS, WHO_AM_I_MPU9250, &i, 1);
	
	if(i == 0x73)
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
			
		for(encIdx = 0; encIdx <= 3; encIdx++)
		{
			PORTB = encIdx; //<< 1;
			_delay_us(1);
			cpb = PIND&0x0C;
			if((ppb[encIdx] & 0x08) != (cpb & 0x08)){
				
				readEncoder(ppb[encIdx], cpb, encIdx);
				ppb[encIdx] = cpb;
				sdt.encData[encIdx+1] = encCnt[encIdx];
				
			}
		}
	
		
		if(adc_flag & 0x01)
		{
			read_flex(idxFinger);
			adc_flag = 0x00;
		}
		//if(curtime - runTimeStamp > 10000)
			//goto INIT_FAIL;
			
		
			

//READ_RETRY:
		ret = readAll(&(mpuData[0]));
		//runCnt++;
		//if(ret & ERR_DATA_READ_FAIL)
		//{
			//if(i2cReadRtyCnt > 10)
			//{
				//UART_printString("\r\nRead MPU 9250 Fail");
				//goto READ_FAIL;
			//}
			//++i2cReadRtyCnt;
			//goto READ_RETRY;
		//}
		//i2cReadRtyCnt = 0;

		for(i = 0; i < 3; i++)
		{
			raw_a[i] = ((int16_t)mpuData[i*2] << 8) | mpuData[i*2 + 1];
			raw_g[i] = ((int16_t)mpuData[i*2 + 6] << 8) | mpuData[i*2 + 6 + 1];
			raw_m[i] = ((int16_t)mpuData[i*2 + 12] << 8) | mpuData[i*2 + 12 + 1];
		}

		if(!(ret & (ERR_ACC | ERR_DATA_NOT_READY(_ACC))))
		{
			sdt.acc[0] = (float)raw_a[0] / 16384;
			sdt.acc[1] = (float)raw_a[1] / 16384;
			sdt.acc[2] = (float)raw_a[2] / 16384;

		}
		//else if(ret & 0x12)
		//UART_printString("ACC Not Ready\r\n");
		
		if(!(ret & (ERR_GYRO | ERR_DATA_NOT_READY(_GYRO))))
		{
			sdt.gyro[0] = (float)raw_g[0] / 131;
			sdt.gyro[1] = (float)raw_g[1] / 131;
			sdt.gyro[2] = (float)raw_g[2] / 131;
			
			sdt.gyro[0] *= (3.141592f / 180.0f);
			sdt.gyro[1] *= (3.141592f / 180.0f);
			sdt.gyro[2] *= (3.141592f / 180.0f);
			
		}
		//else  if(ret & 0x24)
		//UART_printString("GYRO Not Ready\r\n");
		
		if(!(ret & (ERR_MAG | ERR_DATA_NOT_READY(_MAG))))
		{
			uint8_t ASA[3];
			
			TWI_ReadReg(AK8963_ADDRESS, AK8963_ASAX, &(ASA[0]), 3);
			
			sdt.mag[0] = (float)raw_m[0]*((ASA[0]+128.000)/256.000)-1230;
			sdt.mag[1] = (float)raw_m[1]*((ASA[1]+128.000)/256.000)+169;
			sdt.mag[2] = (float)raw_m[2]*((ASA[2]+128.000)/256.000)-699;
			
			sdt.mag[0] *= magScale[0];
			sdt.mag[1] *= magScale[1];
			sdt.mag[2] *= magScale[2];
			
			swap = sdt.mag[0];
			sdt.mag[0] = sdt.mag[1];
			sdt.mag[1] = swap;
			sdt.mag[2] = 0.0f-sdt.mag[2];
		}
		//else  if(ret & 0x48)
			//UART_printString("MAG Not Ready\r\n");
		
		//Set Encoder & Flex

		

#ifdef _DEBUG

		//MahonyAHRSupdate(&(sdt.gyro[0]), &(sdt.acc[0]), &(sdt.mag[0]), &(quaternion[0]));
		MahonyAHRSupdateIMU(&(sdt.gyro[0]), &(sdt.acc[0]),&(quaternion[0]));

		memset(buffer, 0, sizeof(buffer));
		UART_printString("DEL:10.0");
		
		UART_printString("#ENC:");
		sprintf(buffer,"%d,%d,%d,%d,",sdt.encData[1], sdt.encData[2], sdt.encData[3], sdt.encData[4]);
		UART_printString(buffer);
		memset(buffer, 0, sizeof(buffer));
		_delay_ms(10);

		UART_printString("#FLEX:");
		sprintf(buffer,"%d,%d,%d,%d,%d,",sdt.flexData[0], sdt.flexData[1], sdt.flexData[2], sdt.flexData[3], sdt.flexData[4]);
		UART_printString(buffer);
		memset(buffer, 0, sizeof(buffer));
		
		UART_printString("#FILL:");
		sprintf(buffer,"%f,%f,%f,%f\r\n",quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
		UART_printString(buffer);
		memset(buffer, 0, sizeof(buffer));
		_delay_ms(10);

		
#else

		btob85(reportDataBuffer+1, (uint8_t*)&sdt, 46);
		UART_printBin(reportDataBuffer, 62);

#endif
	} 
//READ_FAIL:
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
	
	if(timer0_millis >= timeStampFlex + 1)
		adc_flag = 1;
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