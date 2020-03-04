/* I2C (TwoWire) AVR library
 *
 * Copyright (C) 2015-2017 Sergey Denisov.
 * Rewritten by Sergey Denisov aka LittleBuster (DenisovS21@gmail.com)
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public Licence
 * as published by the Free Software Foundation; either version 3
 * of the Licence, or (at your option) any later version.
 */

#ifndef __I2C_H__
#define __I2C_H__

#include <avr/io.h>

void	i2c_init(uint32_t scl, uint8_t prescale);
uint8_t i2c_start_condition(void);
void	i2c_stop_condition(void);
uint8_t i2c_send_byte(unsigned char byte);
void	i2c_set_address(uint8_t address);
uint8_t i2c_recv_ack(void);
uint8_t i2c_recv_nak(void);


#endif