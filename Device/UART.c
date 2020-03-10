/*
 * UART.c
 *
 * Created: 2020-02-07 오후 12:00:04
 *  Author: bitcamp
 */ 
#include "UART.h"
#include <avr/interrupt.h>

#define SIZE_UART_BUFFER		255
static volatile uint8_t txUARTBuffer[SIZE_UART_BUFFER] = {0, };
static volatile uint8_t idxStart, idxEnd;

void UART_INIT(uint32_t baud)
{
	UCSR0A |= _BV(U2X0);
	
	UBRR0H = 0x00;
	UBRR0L = F_CPU / 8 / baud - 1;
	
	UCSR0C |= 0x06;
	
	UCSR0B |= _BV(RXEN0);		// Enable RX
	UCSR0B |= _BV(TXEN0);		// Enable TX
	//UCSR0B |= _BV(RXCIE0);	// Enable ISR RX_Complete
	
	idxStart = 0;
	idxEnd = 0;
}

inline static void UART_Transmit(uint8_t tx_data)
{
	txUARTBuffer[idxEnd++] = tx_data;
}

void UART_printBin(uint8_t* pData, uint8_t cnt)
{
	uint8_t i = 0;

	UCSR0B |= _BV(UDRIE0);	// Enable UDR0 Ready
	for(i = 0; i < cnt; i++)
		UART_Transmit(pData[i]);
}

void UART_printChar(uint8_t tx_data)
{
	UCSR0B |= _BV(UDRIE0);	// Enable UDR0 Ready
	UART_Transmit(tx_data);
}

void UART_printString(char *str)
{
	UCSR0B |= _BV(UDRIE0);	// Enable UDR0 Ready
	for(int i = 0; str[i];i++)
	{
		UART_Transmit(str[i]);
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
	UCSR0B |= _BV(UDRIE0);	// Enable UDR0 Ready
	for(; i >= 0; i--)
	{
		UART_Transmit(str[i]);
	}
}

ISR(USART_UDRE_vect)
{
	if(idxStart != idxEnd)
	{
		UDR0 = txUARTBuffer[idxStart++];
	}
	else
	{
		UCSR0B &= ~_BV(UDRIE0);	// Disable UDR0 Ready
	}
}