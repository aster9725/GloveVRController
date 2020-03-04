/*
 * twi.c
 *
 * Created: 2020-02-11 오전 11:40:08
 *  Author: YounSeok Shim
 */ 
#include "twi.h"
#include <util/twi.h>

#define F_SCL 400000UL // SCL frequency
#define Prescaler 1
#define TWBR_val ((((F_CPU / F_SCL) / Prescaler) - 16 ) / 2)


void TWI_INIT(void)
{
	DDRC = 0x00;
	PORTC = 0x30;
	TWBR = (uint8_t)TWBR_val;
}

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

void TWI_Stop(void)
{
	// STOP
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
}