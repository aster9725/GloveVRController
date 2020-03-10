#include "mpu9250.h"

void MPU9250_init(void)
{
/*
*	1. 마스터 모드 enable
*	2. I2C클럭설정
*	3. 마스터모드 딜레이 설정
*	4. slave3를 쓰기 모드로 설정하고 지자기센서에 16bit output,Single measurement mode로 설정
*	5. 모든 slave샘플 갯수 설정
*	6. 지자기 센서의 ASAX, ASAY, ASAZ 값을 읽어옴 (이떄 딜레이는 100ms가 권장사항)
*	7. 외부센서 register에 저장된 지자기 센서 데이터 없어짐
*	https://blog.naver.com/speedprinse/221192173624
*/
			
	writeByte(MPU9250_ADDRESS, 0x6B, 0x03); //[PWR_MGMT_1] sleep 끔, Auto PLL
	writeByte(MPU9250_ADDRESS, 0x1A, 0x05); //[CONFIG] DLPF 10Hz

	writeByte(MPU9250_ADDRESS, 0x6A, 0x20); //[USER_CTRL] ,master enable
	writeByte(MPU9250_ADDRESS, 0x24, 0x0D); //[I2C_MST_CTRL] ,내부 i2c clock 400Hz
	writeByte(MPU9250_ADDRESS, 0x67, 0x0F); //[I2C_MST_DELAY_CTRL]

	writeByte(MPU9250_ADDRESS, 0x25, 0x8C); //[I2C_SLV0_ADDR] AK8963 read.
	writeByte(MPU9250_ADDRESS, 0x26, 0x03); //[I2C_SLV0_REG]
	writeByte(MPU9250_ADDRESS, 0x27, 0xD2); //[I2C_SLV0_CTRL]

	//**add ysh

	writeByte(MPU9250_ADDRESS, 0x1B, 0X18); //Gyro +2000dps Scale Select: 00 of 18
	writeByte(MPU9250_ADDRESS, 0x1C, 0X08); //Accel 4G Scale Select: 00 of 18
}

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{ 
	i2c_start_condition();
	i2c_send_byte(address);
	i2c_send_byte(subAddress);
	i2c_send_byte(data);
	i2c_stop_condition();
} 

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data;
	
	i2c_start_condition();
	i2c_send_byte(address);
	i2c_send_byte(subAddress);
	i2c_start_condition();
	i2c_send_byte(address+1);
	data = i2c_recv_nak();
	i2c_stop_condition();

	return data; 
}


//uint8_t readBytes(uint8_t address, uint8_t subAddress, uint8_t cnt, uint8_t* dst)
//{
	//uint8_t data, i;
	//i2c_start_condition(address);
	//i2c_send_byte(subAddress);
	//i2c_stop_condition();
	//
	//i2c_start_condition(address+1);
	//data = i2c_recv_last_byte();
	//i2c_stop_condition();
//
	//return data;
//}

void get_raw_data(void)
{
	/*
	a_x_h = readByte(0x3B);		//x축 가속도
	a_x_l = readByte(0x3C);
	a_y_h = readByte(0x3D);		//y축 가속도
	a_y_l = readByte(0x3E);
	a_z_h = readByte(0x3F);		//z축 가속도
	a_z_l = readByte(0x40);
								
	g_x_h = readByte(0x43);		//x축 각속도
	g_x_l = readByte(0x44);
	g_y_h = readByte(0x45);		//y축 각속도
	g_y_l = readByte(0x46);
	g_z_h = readByte(0x47);		//z축 각속도
	g_z_l = readByte(0x48);

	m_x_h = readByte(0x49);		//x축 콤파스
	m_x_l = readByte(0x4A);
	m_y_h = readByte(0x4B);		//y축 콤파스
	m_y_l = readByte(0x4C);
	m_z_h = readByte(0x4D);		//z축 콤파스
	m_z_l = readByte(0x4E);
	*/
}

