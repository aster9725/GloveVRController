/*
 * UART.h
 *
 * Created: 2020-02-07 오후 12:00:22
 *  Author: bitcamp
 */ 

#define F_CPU 16000000UL

#include<avr/io.h>
#include <stdio.h>

void UART_INIT(int baud);
void USART_Transmit(unsigned char tx_data);
void UART_printString(char *str);
void USART_Transmit_init4(int data);
void UART_printUINT(uint32_t n);