/*
  TwoWire.h - TWI/I2C library for Arduino & Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified 2012 by Todd Krein (todd@krein.org) to implement repeated starts
  Modified 2017 by Michael Mayer to plain C for use with Sduino
*/

#ifndef __TWOWIRE_H
#define __TWOWIRE_H

#include "stm8s.h"

#define BUFFER_LENGTH 32

// WIRE_HAS_END means Wire has end()
#define WIRE_HAS_END 1

/* only this minimal interface is currently implemented: */
void	Wire_begin(void);
void	Wire_end(void);
void	Wire_setClock(uint32_t);
void	Wire_setTimeout(uint16_t);

void		Wire_beginTransmission(uint8_t);
uint8_t 	Wire_endTransmission1(uint8_t sendStop);
inline uint8_t	Wire_endTransmission(void)
{
  return Wire_endTransmission1(TRUE);
}

uint32_t	Wire_write(uint8_t);
uint32_t	Wire_write_sn(const uint8_t *, uint32_t);
int	Wire_available(void);
int	Wire_read(void);
int	Wire_peek(void);
void	Wire_flush(void);

uint8_t	Wire_requestFrom2(uint8_t address, uint8_t quantity);
uint8_t	Wire_requestFrom3(uint8_t address, uint8_t quantity, uint8_t sendStop);
uint8_t Wire_requestFrom5(uint8_t address, uint8_t quantity, uint32_t iaddress, uint8_t isize, uint8_t sendStop);

#endif
