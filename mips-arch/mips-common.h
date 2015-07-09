
/*
** mips-common.h
**
** Info about the MIPS instruction set shared across simulators.
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

#ifndef _MIPS_COMMON_H
#define _MIPS_COMMON_H

/* Location of register fields in opcode. */

#define RS_SHIFTS 21
#define RT_SHIFTS 16
#define RD_SHIFTS 11
#define SA_SHIFTS 6

void MIPS_disasm(uint32_t addr, uint32_t opcode, char *buf);

#endif
