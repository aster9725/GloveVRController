/*
 * UART.h
 *
 * Created: 2020-01-08 오후 12:37:01
 *  Author: bitcamp
 */ 


#ifndef UART_H_
#define UART_H_

#include <avr/io.h>

#define RXUART_BUFF_SIZE		64
#define RXUART_BUFF_SIZE_MASK	0x3F

#define FRD_READY		(1<<0)
#define FRD_READ		(1<<1)
#define FRD_SEND		(1<<2)

extern volatile uint8_t flagReportData;	// FRD
extern volatile uint8_t rxUART[RXUART_BUFF_SIZE];


void UART_INIT(uint32_t baud);
unsigned char UART_receive(unsigned char block);
void UART_transmit(unsigned char data);
void UART_printString(char *str);
void UART_printUINT(uint32_t n);
void UART_printf(char *fmt, ...);

#endif /* UART_H_ */