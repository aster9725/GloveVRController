#define F_CPU 16000000L
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include "UART.h"

volatile int count;
int8_t encoder = 0;

ISR(TIMER0_OVF_vect) // TCNT0 값 넘침(오버플로우) 발생시 여기로 점프
{
	if(count>5)
	count=0;
	else
	count++;
	
	TCNT0=0x64; // TCMT0값을 10진수 100으로 초기화
}

void timer_delay(void)
{
	count=0;
	while(count<=5){} // 50이 될 때까지 동작x
}

void Set(void)
{
	PORTD |= (1 << PORTD2) | (1 << PORTD3);
	DDRD &= ~(1 << DDD2) & ~(1 << DDD3);

	PCICR |= (1 << PCIE0); //  핀 활성화
	PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);
	sei(); // interrupt enable
}

void ADC_INIT()
{
	ADMUX |= 0x40;  // 전압 설정
	
	ADCSRA |= 0x07;  //	 분주율 선택
	ADCSRA |= (1 << ADEN);	// ADC 활성화
	ADCSRA |= (1 << ADATE);	// 트리거 신호 자동화 실행 (실행될 때마다 업데이트)
	

}

void read_Val_a(void)
{
	
	if(PIND & (1 << PIND2)) // HIGH
	{
		if(PIND & (0 << PIND3)) // HIGH
		{
			encoder++;
		}
		else
		{
			encoder--;
		}
	}
	else
	{
		if(PIND & (0 << PIND3))
		{
			encoder--;
		}
		else
		{
			encoder++;
		}
	}
	
	if(PIND & (1 << PIND3))
	{
		if(PIND & (0 << PIND2))
		{
			encoder--;
		}
		else
		{
			encoder++;
		}
	}
	else
	{
		if(PIND & (0 << PIND2))
		{
			encoder++;
		}
		else
		{
			encoder--;
		}
	}
}


int read_ADC(unsigned char channel)
{
	ADMUX |= 0x40;
	ADMUX = ((ADMUX & 0xE0) | channel); 
	ADCSRA |= (1 << ADSC);	// AD(Analog -> digital)변환을 시작하도록 하는 비트
	while (!(ADCSRA & (1 << ADIF))); // 한번 실행 될 때 까지
	
	return ADC;	// ADC 값 리턴
}

int read_ADC_a(unsigned char channel)
{
	ADMUX |= 0x40;
	ADMUX = ((ADMUX & 0xE0) | channel);
	ADCSRA |= (1 << ADSC);	// AD(Analog -> digital)변환을 시작하도록 하는 비트	
	while (!(ADCSRA & (1 << ADIF))); // 한번 실행 될 때 까지
	
	return ADC;	// ADC 값 리턴
}

int read_ADC_b(unsigned char channel)
{
	ADMUX |= 0x40;
	ADMUX = ((ADMUX & 0xE0) | channel); 
	ADCSRA |= (1 << ADSC);	// AD(Analog -> digital)변환을 시작하도록 하는 비트
	while (!(ADCSRA & (1 << ADIF))); // 한번 실행 될 때 까지
	
	return ADC;	// ADC 값 리턴
}

int read_ADC_c(unsigned char channel)
{
	ADMUX |= 0x40;
	ADMUX = ((ADMUX & 0xE0) | channel);
	ADCSRA |= (1 << ADSC);	// AD(Analog -> digital)변환을 시작하도록 하는 비트
	while (!(ADCSRA & (1 << ADIF))); // 한번 실행 될 때 까지
	
	return ADC;	// ADC 값 리턴
}

int read_ADC_d(unsigned char channel)
{
	ADMUX |= 0x40;
	ADMUX = ((ADMUX & 0xE0) | channel);
	ADCSRA |= (1 << ADSC);	// AD(Analog -> digital)변환을 시작하도록 하는 비트
	while (!(ADCSRA & (1 << ADIF))); // 한번 실행 될 때 까지
	
	return ADC;	// ADC 값 리턴
}

void int_to_string(int n, char* buffer)
{
	sprintf(buffer, "%04d", n);	// string을 출력
	buffer[4] = '\0'; //	string 출력이므로 제일 마지막에  NULL 값 넣어줌
}

int main(void)
{
	int read;
	int read_a;
	int read_b, read_c, read_d;
	char buffer[5];
	
	TCCR0A=0x00; // 타이머 인터럽트 설정 레지스터A의 값을 일반모드로(오버플로우)
	TCCR0B=0x05; // 타이머 인터럽트 설정 레이스터 B의 값을 1024 분주비로 설정
	TCNT0=0x64;
	
	TIMSK0 = 0x01; // 타이머 인터럽트의 TOIE0를 설정 오버플로우를 허용
	
	sei();
	
	UART_INIT();
	ADC_INIT();
	Set();
	int8_t result = encoder;
	while(1)
	{
		read = read_ADC(1); // ADC 값을 read에 대입
	
		UART_printString("A0 : ");
		int_to_string(read,buffer); // int 값을 string으로
		UART_printString(buffer);	// buffer 수 만큼
		UART_printString("\t");
		
		timer_delay();

		read_a = read_ADC_a(2);
		UART_printString("A1 : ");
		int_to_string(read_a,buffer); // int 값을 string으로
		UART_printString(buffer);	// buffer 수 만큼
		UART_printString("\t");
		
		timer_delay();
		
		read_b = read_ADC_b(3);
		UART_printString("A2 : ");
		int_to_string(read_b,buffer); // int 값을 string으로
		UART_printString(buffer);	// buffer 수 만큼
		UART_printString("\t");
		
		timer_delay();
		
		read_c = read_ADC_c(4);
		UART_printString("A3 : ");
		int_to_string(read_c,buffer); // int 값을 string으로
		UART_printString(buffer);	// buffer 수 만큼
		UART_printString("\t");
		
		timer_delay();
		
		read_d = read_ADC_d(0);
		UART_printString("A4 : ");
		int_to_string(read_d,buffer); // int 값을 string으로
		UART_printString(buffer);	// buffer 수 만큼
		UART_printString("\t");
		
		timer_delay();
		
		read_Val_a();
		if(result!=encoder)
		{
			result = encoder;
			int_to_string(result/2,buffer);
			UART_printString("encoder : ")
			UART_printString(buffer);
			UART_printString("\n");
			
			timer_delay();
			
			//UART_transmit('0'+ result/10);
			//UART_transmit('0'+ result%10);
		}
		
	}
	return 0;
}