/*
 * UART.c
 *
 * Created: 2020-01-08 오후 12:36:49
 *  Author: bitcamp
 */ 

#include "UART.h"
#include <avr/interrupt.h>

volatile uint8_t flagReportData = FRD_SEND;	// FRD
volatile uint8_t rxUART[RXUART_BUFF_SIZE] = {0, };

void UART_INIT(uint32_t baud)
{
	UCSR1A |= _BV(U2X1);
		
	UBRR1H = 0x00;
	UBRR1L = F_CPU / 8 / baud - 1;
		
	UCSR1C |= 0x06;
		
	UCSR1B |= _BV(RXEN1);	// Enable RX
	UCSR1B |= _BV(TXEN1);	// Enable TX
	UCSR1B |= _BV(RXCIE1);	// Enable ISR RX_Complete
	
	flagReportData = FRD_SEND;
}

unsigned char UART_receive(unsigned char block)
{
	if(!block)
	{
		if(!(UCSR1A & (1<<RXC1)))
			return 0;
		else
			return UDR1;
	}
	else
	{
		while(!(UCSR1A & (1<<RXC1)));
		return UDR1;
	}
	return 0;
}

void UART_transmit(unsigned char data)
{
	while(!(UCSR1A & (1<<UDRE1)));
	UDR1 = data;
}

void UART_printString(char *str)
{
	int i = 0;
	while(str[i])
	{
		while(!(UCSR1A & (1<<UDRE1)));
		UDR1 = str[i++];
	}
}

void UART_printUINT(uint32_t n)
{
	int i = 0;
	char str[11] = {'0',};
		
	if(n > 0)
	{
		for(; n > 0; i++)
		{
			str[i] = n % 10 + '0';
			n = n / 10;
		}
		i--;
	}
	
	for(; i >= 0; i--)
		UART_transmit(str[i]);
}
