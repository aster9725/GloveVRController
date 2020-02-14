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
volatile unsigned char a_x_l,a_x_h,a_y_l,a_y_h,a_z_l,a_z_h;
volatile unsigned char g_x_l,g_x_h,g_y_l,g_y_h,g_z_l,g_z_h;
volatile unsigned char m_x_l,m_x_h,m_y_l,m_y_h,m_z_l,m_z_h;

void MPU9250_init();
void twi_write(unsigned char address,unsigned char data);
unsigned char twi_read(char address);
void get_raw_data();


#endif /* TWI_H_ */