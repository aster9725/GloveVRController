/*
 * twi.c
 *
 * Created: 2020-02-11 오전 11:40:08
 *  Author: YounSeok Shim
 */ 
#include "twi.h"
#include <util/twi.h>
//#include <string.h>

#define F_SCL 400000UL // SCL frequency
#define Prescaler 1
#define TWBR_val ((((F_CPU / F_SCL) / Prescaler) - 16 ) / 2)

//#define SIZE_TWI_BUFFER	64
//static volatile uint8_t TWIBuffer[SIZE_TWI_BUFFER] = {0, };
//static volatile uint8_t idxTwi = 0;
//static volatile uint8_t lenTwiData = 0;
//static volatile uint8_t isTWIBusy = 0;


void TWI_INIT(void)
{
	DDRC = 0x00;
	PORTC = 0x30;
	TWBR = (uint8_t)TWBR_val;
	//TWSR = 0x00;


	//TWCR = _BV(TWEN);	// Enable I2C Interrupt	
	//isTWIBusy = 0;
	//idxTwi = 0;
	//lenTwiData = 0;
}

void TWI_Stop(void)
{
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);
}


//////////////////////////////////////////////////////////////////////////
//			Non-Blocking (notwork)
//////////////////////////////////////////////////////////////////////////

//void TWI_Wait(void)
//{
	//while (isTWIBusy);
//}
//
//void TWI_Start(void)
//{
	//TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWIE) | _BV(TWSTA);
//}
//

//
//void TWI_Ack(void)
//{
	//TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
//}
//
//void TWI_Nack(void)
//{
	//TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWIE);
//}
//
//void TWI_Transmit(uint8_t data)
//{
	//TWDR = data;
//}
//
//void TWI_Receive()
//{
	//TWIBuffer[idxTwi] = TWDR;
//}
//
//void TWI_Reply(void)
//{
	//if(idxTwi < lenTwiData - 1)
		//TWI_Ack();
	//else
		//TWI_Nack();
//}
//
//void TWI_Done(void)
//{
	//isTWIBusy = 0;
//}
//
//uint8_t TWI_WriteReg(uint8_t devaddr, uint8_t regaddr, uint8_t* data, uint8_t length)
//{
	//TWI_Wait();
	//
	//isTWIBusy = 1;
	//
	//TWIBuffer[0] = (devaddr << 1) | TW_WRITE;
	//TWIBuffer[1] = regaddr;
	//lenTwiData = length + 2;
	//idxTwi = 0;
	//
	//memcpy(&TWIBuffer[2], data, length);
	//
	//TWI_Start();
	//
	//return 0;
//}
//
//void TWI_Write(uint8_t address, uint8_t* data, uint8_t length)
//{
	//TWI_Wait();
	//
	//isTWIBusy = 1;
	//
	//TWIBuffer[0] = (address << 1) | TW_WRITE;
	//lenTwiData = (length + 1);
	//idxTwi = 0;
	//
	//memcpy(&TWIBuffer[1], data, length);
	//
	//TWI_Start();
//}
//
//uint8_t TWI_ReadReg(uint8_t devaddr, uint8_t regaddr, uint8_t* data, uint8_t length)
//{
	//TWI_Wait();
	//
	//isTWIBusy = 1;
	//
	//TWIBuffer[0] = (devaddr << 1) | TW_READ;
	//TWIBuffer[1] = regaddr;
	//lenTwiData = length + 2;
	//idxTwi = 0;
	//
	//TWI_Start();
	//
	//return 0;
//}
//
//void TWI_Read(uint8_t address, uint8_t* data, uint8_t length)
//{
	//TWI_Wait();
	//
	//isTWIBusy = 1;
	//
	//TWIBuffer[0] = (address << 1) | TW_READ;
	//lenTwiData = length + 1;
	//idxTwi = 0;
	//
	//TWI_Start();
//}
//
//ISR(TWI_vect)
//{
	//switch (TW_STATUS) {
		//case TW_START:
		//case TW_REP_START:
		//case TW_MT_SLA_ACK:
		//case TW_MT_DATA_ACK:
			//if (idxTwi < lenTwiData) 
			//{
				//TWI_Transmit(TWIBuffer[idxTwi++]);
				//TWI_Nack();
			//} 
			//else 
			//{
				//TWI_Stop();
				//TWI_Done();
			//}
		//break;
//
		//case TW_MR_DATA_ACK:
		//TWI_Receive();
		//TWI_Reply();
		//break;
//
		//case TW_MR_SLA_ACK:
		//TWI_Reply();
		//break;
//
		//case TW_MR_DATA_NACK:
		//TWI_Receive();
		//TWI_Stop();
		//TWI_Done();
		//break;
//
		//case TW_MT_SLA_NACK:
		//case TW_MR_SLA_NACK:
		//case TW_MT_DATA_NACK:
		//default:
		//TWI_Stop();
		//TWI_Done();
		//break;
	//}
//}


//////////////////////////////////////////////////////////////////////////
//			Blocking I2C
//////////////////////////////////////////////////////////////////////////

uint8_t TWI_Start(uint8_t address)
{
	// reset control register
	TWCR = 0;
	// START
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	// wait until transmission end
	while( !(TWCR & (1<<TWINT)) );
	
	// check start successfully transmitted
	uint8_t twsr = TWSR & 0xF8;
	if(twsr != TW_START) return twsr;
	
	// load slave address
	TWDR = address;
	// transmit slave address
	TWCR = (1<<TWINT) | (1<<TWEN);
	// wait until transmission end
	while( !(TWCR & (1<<TWINT)) );
	
	// check ACK
	twsr = TW_STATUS & 0xF8;
	if ( (twsr != TW_MT_SLA_ACK) && (twsr != TW_MR_SLA_ACK)) return twsr;
	
	return 0;
}

uint8_t TWI_Write(uint8_t data)
{
	// load data
	TWDR = data;
	// transmit data
	TWCR = (1<<TWINT) | (1<<TWEN);
	// wait until transmission end
	while( !(TWCR & (1<<TWINT)) );

	uint8_t twsr = TWSR & 0xF8;
	if( twsr != TW_MT_DATA_ACK && (twsr != TW_MR_DATA_ACK) ){ return twsr; }
	
	return 0;
}

uint8_t TWI_ReadACK(void)
{
	// set ACK bit for continuous reading
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	while( !(TWCR & (1<<TWINT)) );

	return TWDR;
}

uint8_t TWI_ReadNACK(void)
{
	
	// clear ACK bit for stop reading
	TWCR = (1<<TWINT) | (1<<TWEN);
	while( !(TWCR & (1<<TWINT)) );

	return TWDR;
}

uint8_t TWI_Transmit(uint8_t address, uint8_t* data, uint16_t length)
{
	uint8_t twsr;
	if (twsr = TWI_Start(address | 0x00)) return twsr | 0x01;
	
	for (uint16_t i = 0; i < length; i++)
	{
		if (twsr = TWI_Write(data[i])) return twsr | 0x02;
	}
	
	TWI_Stop();
	
	return 0;
}

uint8_t TWI_Receive(uint8_t address, uint8_t* data, uint16_t length)
{
	uint8_t twsr;
	if (twsr = TWI_Start(address | 0x01)) return twsr | 0x01;

	for (uint16_t i = 0; i < (length-1); i++)
	{
		data[i] = TWI_ReadACK();
	}
	data[(length-1)] = TWI_ReadNACK();
	
	TWI_Stop();
	
	return 0;
}

uint8_t TWI_WriteReg(uint8_t devaddr, uint8_t regaddr, uint8_t* data, uint16_t length)
{
	uint8_t twsr;
	if (twsr = TWI_Start(devaddr<<1)) return twsr | 0x01;

	if(twsr = TWI_Write(regaddr)) return twsr | 0x02;

	for (uint16_t i = 0; i < length; i++)
	{
		if (twsr = TWI_Write(data[i])) return twsr | 0x03;
	}

	TWI_Stop();

	return 0;
}

uint8_t TWI_ReadReg(uint8_t devaddr, uint8_t regaddr, uint8_t* data, uint16_t length)
{
	uint8_t twsr;
	if (twsr = TWI_Start(devaddr<<1)) return twsr | 0x01;

	if(twsr = TWI_Write(regaddr)) return twsr | 0x02;

	if (twsr = TWI_Start((devaddr<<1) + 1)) return twsr | 0x03;

	for (uint16_t i = 0; i < (length-1); i++)
	{
		*(data + i) = TWI_ReadACK();
	}
	*(data + (length-1)) = TWI_ReadNACK();

	TWI_Stop();

	return 0;
}

