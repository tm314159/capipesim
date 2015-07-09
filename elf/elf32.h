/*
** elf.h
**
** ELF file loader header
**
** by Toshiyasu Morita
**
** Started: 11/26/2013 @ 1:22 pm
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

#ifndef ELF_H
#define ELF_H

#include <stdint.h>

typedef struct {

	uint32_t sh_name;
	uint32_t sh_type;
	uint32_t sh_flags;
	uint32_t sh_addr;
	uint32_t sh_offset;
	uint32_t sh_size;
	uint32_t sh_link;
	uint32_t sh_info;
	uint32_t sh_addralign;
	uint32_t sh_entsize;

} ELF32_Shdr;

/* p_type values */
#define PT_NULL    0
#define PT_LOAD    1
#define PT_DYNAMIC 2
#define PT_INTERP  3
#define PT_NOTE    4
#define PT_SHLIB   5
#define PT_PHDR    6
#define PT_LOPROC  0x70000000
#define PT_HIPROC  0x7fffffff

typedef struct {

	uint32_t p_type;
	uint32_t p_offset;
	uint32_t p_vaddr;
	uint32_t p_paddr;
	uint32_t p_filesz;
	uint32_t p_memsz;
	uint32_t p_flags;
	uint32_t p_align;

} ELF32_Phdr;

/* e_ident offsets */
#define EI_MAG0    0
#define EI_MAG1    1
#define EI_MAG2    2
#define EI_MAG3    3
#define EI_CLASS   4
#define EI_DATA    5
#define EI_VERSION 6
#define EI_PAD     7
#define EI_NIDENT  16

typedef struct {

	uint8_t	e_ident[EI_NIDENT];
	uint16_t e_type;
	uint16_t e_machine;
	uint32_t e_version;
	uint32_t e_entry;
	uint32_t e_phoff;
	uint32_t e_shoff;
	uint32_t e_flags;
	uint16_t e_ehsize;
	uint16_t e_phentsize;
	uint16_t e_phnum;
	uint16_t e_shentsize;
	uint16_t e_shnum;
	uint16_t e_shstrndx;

} ELF32_Ehdr;

typedef struct {

	FILE *infile;
	ELF32_Ehdr *ehdr;
	ELF32_Phdr *phdr;
	ELF32_Shdr *shdr;
	char *string_table;

} ELF_FILE;

/* sh_flags bits */

#define SHF_WRITE	1
#define SHF_ALLOC	2
#define SHF_EXECINSTR	4
#define SHF_MASKPROC	0xf00000000

extern int big_endian_flag;

ELF_FILE *elf32_fopen(char *filename);
int elf32_load(ELF_FILE *elf_file, MEM32_IMAGE *mem_image);

#endif
