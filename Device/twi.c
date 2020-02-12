/*
 * twi.c
 *
 * Created: 2020-02-11 오전 11:40:08
 *  Author: bitcamp
 */ 
#include "twi.h"


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
    twi_write(0x2F, 0x0A); //[I2C_SLV3_REG]		내부 레지스터 주소
    twi_write(0x30, 0x81); //[I2C_SLV3_CTRL]	인터럽트 신호 발생 완료되면 비활성화 레지스터 54에서 확인가능	& 속도 감소 수식 : 1 / (1 + I2C_MST_DLY(17)) samples 
    twi_write(0x66, 0x11); //[I2C_SLV3_DO]		슬레이브장치에 기록	DI는 읽기
	
	/*
	I2C_SLV4_EN When set to 1, this bit enables Slave 4 for data transfer operations.
	When cleared to 0, this bit disables Slave 4 from data transfer operations.
	I2C_SLV4_INT_EN When set to 1, this bit enables the generation of an interrupt signal upon
	completion of a Slave 4 transaction.
	When cleared to 0, this bit disables the generation of an interrupt signal
	upon completion of a Slave 4 transaction.
	The interrupt status can be observed in Register 54.
	I2C_SLV4_REG_DIS When set to 1, the transaction will read or write data.
	When cleared to 0, the transaction will read or write a register address.
	I2C_MST_DLY Configures the decreased access rate of slave devices relative to the
	Sample Rate.
	
	I2C_MST_DLY configures the reduced access rate of I
	2C slaves relative to the Sample Rate. When
	a slave’s access rate is decreased relative to the Sample Rate, the slave is accessed every
	1 / (1 + I2C_MST_DLY) samples
	This base Sample Rate in turn is determined by SMPLRT_DIV (register 25) and DLPF_CFG
	(register 26). Whether a slave’s access rate is reduced relative to the Sample Rate is determined by
	I2C_MST_DELAY_CTRL (register 103).
	*/

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

	_delay_ms(100);

	ASAX = twi_read(0x49);
	ASAY = twi_read(0x4A);
	ASAZ = twi_read(0x4B);

	_delay_ms(100);


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
	//twi_write(0x1B, 0X18); //Gyro +2000dps Scale Select: 00 of 18
	twi_write(0x1C, 0X08); //Accel 4G Scale Select: 00 of 18
	//

}



void twi_write(unsigned char address,unsigned char data)
{ 
	TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN) | _BV(TWEA);	//START

	while(!(TWCR & (1<<TWINT))); //TWINT flag 기다림 

	TWDR = 0b11010000;			 //AD(1101000)+W(0) 
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
	while( !(TWCR & (1 << TWINT)) ); // 전송 완료 대기

	TWDR = address; 			 //register address
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
	while( !(TWCR & (1 << TWINT)) ); // 전송 완료 대기

	TWDR = data; 				 //data
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
	while( !(TWCR & (1 << TWINT)) ); // 전송 완료 대기

	TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN) | _BV(TWEA); //STOP
} 

 

unsigned char twi_read(char address)
{ 
	unsigned char data;

	TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN) | _BV(TWEA);
	while( !(TWCR & (1 << TWINT)) ); // 시작 완료 대기

	TWDR = 0b11010000;			 //AD(1101000)+W(0) 
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
	while( !(TWCR & (1 << TWINT)) ); // 전송 완료 대기

	TWDR = address; 			 //register address
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
	while( !(TWCR & (1 << TWINT)) ); // 전송 완료 대기

	TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN) | _BV(TWEA);
	while( !(TWCR & (1 << TWINT)) ); // 시작 완료 대기

	TWDR = 0b11010001;			 //AD(1101000)+R(1) 
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
	while( !(TWCR & (1 << TWINT)) ); // 전송 완료 대기

	TWCR = _BV(TWINT) | _BV(TWEN);
	while( !(TWCR & (1 << TWINT)) ); // 수신 완료 대기

	data = TWDR; 

	TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN) | _BV(TWEA);

	return data; 
}
