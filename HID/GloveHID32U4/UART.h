/*
 * UART.h
 *
 * Created: 2020-01-08 오후 12:37:01
 *  Author: bitcamp
 */ 


#ifndef UART_H_
#define UART_H_

#include <avr/io.h>

#define START_CHAR	0x53 // 'S'
#define END_CHAR	0x45 // 'E'


void UART_INIT(int baud);
unsigned char UART_receive(unsigned char block);
void UART_transmit(unsigned char data);
void UART_printString(char *str);
void UART_printUINT(uint32_t n);
void UART_printf(char *fmt, ...);

#endif /* UART_H_ */