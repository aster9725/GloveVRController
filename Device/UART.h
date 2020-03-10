/*
 * UART.h
 *
 * Created: 2020-02-07 오후 12:00:22
 *  Author: bitcamp
 */ 

#define F_CPU 16000000UL

#include <avr/io.h>

void UART_INIT(uint32_t baud);
void UART_printBin(uint8_t* pData, uint8_t cnt);
void UART_printChar(uint8_t tx_data);
void UART_printString(char *str);
void UART_printUINT(uint32_t n);

