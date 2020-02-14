/*
 * twi.h
 *
 * Created: 2020-02-11 오전 11:39:58
 *  Author: bitcamp
 */ 


#ifndef TWI_H_
#define TWI_H_

#define F_CPU 16000000UL
#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>
#include<math.h>
#include "twi.h"

volatile float ASAX,ASAY,ASAZ;

void MPU9250_init();
void twi_write(unsigned char address,unsigned char data);
unsigned char twi_read(char address);


#endif /* TWI_H_ */