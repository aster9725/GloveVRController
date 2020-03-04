/*
 * twi.h
 *
 * Created: 2020-02-11 오전 11:39:58
 *  Author: YounSeok Shim
 */ 


#ifndef TWI_H_
#define TWI_H_


#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 16000000UL

void TWI_INIT(void);
uint8_t TWI_Start(uint8_t address);
uint8_t TWI_Write(uint8_t data);
uint8_t TWI_ReadACK(void);
uint8_t TWI_ReadNACK(void);
uint8_t TWI_Transmit(uint8_t address, uint8_t* data, uint16_t length);
uint8_t TWI_Receive(uint8_t address, uint8_t* data, uint16_t length);
uint8_t TWI_WriteReg(uint8_t devaddr, uint8_t regaddr, uint8_t* data, uint16_t length);
uint8_t TWI_ReadReg(uint8_t devaddr, uint8_t regaddr, uint8_t* data, uint16_t length);
void TWI_Stop(void);

#endif /* TWI_H_ */