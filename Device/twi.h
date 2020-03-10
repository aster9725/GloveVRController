/*
 * twi.h
 *
 * Created: 2020-02-11 오전 11:39:58
 *  Author: YounSeok Shim
 */ 


#ifndef TWI_H_
#define TWI_H_

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>


void TWI_INIT(void);
void TWI_Stop(void);

// Nonblock I2C
//void TWI_Wait(void);
//void TWI_Start(void);
//void TWI_Ack(void);
//void TWI_Nack(void);
//void TWI_Transmit(uint8_t data);
//void TWI_Receive(void);
//void TWI_Reply(void);
//void TWI_Done(void);
//uint8_t TWI_WriteReg(uint8_t devaddr, uint8_t regaddr, uint8_t* data, uint8_t length);
//void TWI_Write(uint8_t address, uint8_t* data, uint8_t length);
//uint8_t TWI_ReadReg(uint8_t devaddr, uint8_t regaddr, uint8_t* data, uint8_t length);
//void TWI_Read(uint8_t address, uint8_t* data, uint8_t length);

// Blocking I2C
uint8_t TWI_Start(uint8_t address);
uint8_t TWI_Write(uint8_t data);
uint8_t TWI_ReadACK(void);
uint8_t TWI_ReadNACK(void);
uint8_t TWI_Transmit(uint8_t address, uint8_t* data, uint16_t length);
uint8_t TWI_Receive(uint8_t address, uint8_t* data, uint16_t length);
uint8_t TWI_WriteReg(uint8_t devaddr, uint8_t regaddr, uint8_t* data, uint16_t length);
uint8_t TWI_ReadReg(uint8_t devaddr, uint8_t regaddr, uint8_t* data, uint16_t length);


#endif /* TWI_H_ */