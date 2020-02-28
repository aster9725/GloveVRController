/*
 * UART.c
 *
 * Created: 2020-02-07 오후 12:00:04
 *  Author: bitcamp
 */ 
#include "UART.h"

void UART_INIT(int baud)
{
	UCSR0A |= _BV(U2X0);
	
	UBRR0H = 0x00;
	UBRR0L = F_CPU / 8 / baud - 1;
	//UBRR0L = 19;	// test for 100000 baud
	
	UCSR0C |= 0x06;
	
	UCSR0B |= _BV(RXEN0);	// Enable RX
	UCSR0B |= _BV(TXEN0);	// Enable TX
	UCSR0B |= _BV(RXCIE0);	// Enable ISR RX_Complete
}

void USART_Transmit(unsigned char tx_data)
{
	while( !(UCSR0A & (1 <<UDRE0)));
	UDR0 = tx_data;
}

void UART_printString(char *str)
{
	for(int i = 0; str[i];i++)
	USART_Transmit(str[i]);
}

void USART_Transmit_init4(int data)
{
	if(data < 0)
	{
		data = -data;
		USART_Transmit('-');
	}
	else
	USART_Transmit(' ');

	int temp = 0;
	temp = data/10000;
	USART_Transmit(temp+48);
	temp = (data%10000)/1000;
	USART_Transmit(temp+48);
	temp = (data%1000)/100;
	USART_Transmit(temp+48);
	temp = (data%100)/10;
	USART_Transmit(temp+48);
	temp = data%10;
	USART_Transmit(temp+48);
	
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
	USART_Transmit(str[i]);
}