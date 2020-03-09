/*
 * base85.h
 *
 * Created: 2020-03-09 오전 9:36:49
 *  Author: bitcamp
 */ 


#ifndef BASE85_H_
#define BASE85_H_

#include "avr/io.h"

void btob85(uint8_t* dest, uint8_t* src, uint8_t size);
void b85tob(uint8_t* dest, volatile uint8_t* src);


#endif /* BASE85_H_ */