/*
** mem.h
**
** Memory image header
*/

/*
** capipesim is free software; you can redistribute it and/or modify it
** under the terms of the GNU General Public License as published by the
** Free Software Foundation; either version 3, or (at your option) any later
** version.
**
** capipesim is distributed in the hope that it will be useful, but WITHOUT
** ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
** FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
** for more details.
** 
** You should have received a copy of the GNU General Public License
** along with capipesim; see the file COPYING3.  If not see
** <http://www.gnu.org/licenses/>.
*/

#ifndef MEM_H
#define MEM_H

#include <stdint.h>

typedef struct {

	uint32_t address, size, big_endian_flag;
	uint8_t data[0];

} MEM32_IMAGE;

uint32_t mem32_read_byte(MEM32_IMAGE *mem_image, uint32_t address);
uint32_t mem32_read_halfword(MEM32_IMAGE *mem_image, uint32_t address);
uint32_t mem32_read_word(MEM32_IMAGE *mem_image, uint32_t address);
void mem32_write_byte(MEM32_IMAGE *mem_image, uint32_t address, uint8_t value);
void mem32_write_halfword(MEM32_IMAGE *mem_image, uint32_t address, uint16_t value);
void mem32_write_word(MEM32_IMAGE *mem_image, uint32_t address, uint32_t value);
void mem32_write_block(MEM32_IMAGE *mem_image, uint32_t address, uint32_t size, void *buffer);
MEM32_IMAGE *mem32_init(uint32_t address, uint32_t size, int big_endian_flag);

#endif

