/*
 * UART.h
 *
 * Created: 2020-02-07 오후 12:00:22
 *  Author: bitcamp
 */ 

#define F_CPU 16000000UL

#include <avr/io.h>
#include <stdio.h>

void UART_INIT(uint32_t baud);
void USART_Transmit(uint8_t tx_data);
void UART_printString(char *str);
void USART_Transmit_int4(int data);
void UART_printUINT(uint32_t n);

void UART_EncodeAscii85(uint8_t* data, uint8_t cnt);
void UART_DecodeAscii85(uint8_t* data, uint8_t cnt);