#include "i2c.h"

void i2c_init(uint32_t scl, uint8_t prescale)
{
	uint8_t prescaler = prescale;
	uint8_t twps = 0;
	
	if(!prescale)
		prescaler = 1;
	
	for(prescale; prescale > 1; prescale/=4)
		++twps;

	TWSR |= twps;
	TWBR = ((((F_CPU / scl) / prescaler) - 16 ) / 2);
	TWCR = (1 << TWEN); // I2C È°¼ºÈ­
}

uint8_t i2c_start_condition(void)
{
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while ((TWCR & (1 << TWINT)) == 0);
	
	return (TWSR & 0xF8);
}

void i2c_stop_condition(void)
{
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

uint8_t i2c_send_byte(unsigned char byte)
{
	TWDR = byte;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while ((TWCR & (1 << TWINT)) == 0);
	
	return (TWSR & 0xF8);
}

void i2c_set_address(uint8_t address)
{
	TWAR = address;
}

uint8_t i2c_recv_ack(void)
{
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while ((TWCR & (1 << TWINT)) == 0);
    return TWDR;
}

uint8_t i2c_recv_nak(void)
{
	TWCR = (1 << TWINT) | (1 << TWEN);
    while ((TWCR & (1 << TWINT)) == 0);
    return TWDR;
}