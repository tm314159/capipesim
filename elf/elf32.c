/*
** elf.c
**
** ELF file loader
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

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "mem.h"
#include "elf32.h"

int big_endian_flag;

uint16_t fget16(FILE *infile)
{
	uint32_t value;

	if (big_endian_flag) {
		value  = fgetc(infile) << 8;
		value |= fgetc(infile);
	} else {
		value  = fgetc(infile);
		value |= fgetc(infile) << 8;
	}

	return value;	
}

uint32_t fget32(FILE *infile)
{
	uint32_t value;

	if (big_endian_flag) {
		value  = fgetc(infile) << 24;
		value |= fgetc(infile) << 16;
		value |= fgetc(infile) << 8;
		value |= fgetc(infile);
	} else {
		value  = fgetc(infile);
		value |= fgetc(infile) << 8;
		value |= fgetc(infile) << 16;
		value |= fgetc(infile) << 24;
	}

	return value;
}

void elf32_ehdr_dump(ELF32_Ehdr *ehdr)
{
	printf("e_ident[EI_MAG0]   : 0x%02x ('%c')\n",  ehdr->e_ident[EI_MAG0], ehdr->e_ident[EI_MAG0]);
	printf("e_ident[EI_MAG1]   : 0x%02x ('%c'))\n", ehdr->e_ident[EI_MAG1], ehdr->e_ident[EI_MAG1]);
	printf("e_ident[EI_MAG2]   : 0x%02x ('%c'))\n", ehdr->e_ident[EI_MAG2], ehdr->e_ident[EI_MAG2]);
	printf("e_ident[EI_MAG3]   : 0x%02x ('%c'))\n", ehdr->e_ident[EI_MAG3], ehdr->e_ident[EI_MAG3]);
	printf("e_ident[EI_CLASS ] : 0x%02x ", ehdr->e_ident[EI_CLASS]);

	switch (ehdr->e_ident[EI_CLASS]) {

		case 0:
			printf("(ELFCLASSNONE)\n");
			break;

		case 1:
			printf("(ELFCLASS32)\n");
			break;

		case 2:
			printf("(ELFCLASS64)\n");
			break;
	}

	printf("e_ident[EI_DATA]   : 0x%02x ", ehdr->e_ident[EI_DATA]);

	switch (ehdr->e_ident[EI_DATA]) {

		case 0:
			printf("(ELFDATANONE)\n");
			break;

		case 1:
			printf("(ELFDATA2LSB)\n");
			break;

		case 2:
			printf("(ELFDATA2MSB)\n");
			break;
	}

	printf("e_ident[EI_VERSION]: 0x%02x\n", ehdr->e_ident[EI_VERSION]);
	printf("e_ident[EI_PAD]    : 0x%02x\n", ehdr->e_ident[EI_PAD]);

	printf("e_type     : 0x%04x\n", ehdr->e_type);
	printf("e_machine  : 0x%04x\n", ehdr->e_machine);
	printf("e_version  : 0x%08x\n", ehdr->e_version);
	printf("e_entry    : 0x%08x\n", ehdr->e_entry);
	printf("e_phoff    : 0x%08x\n", ehdr->e_phoff);
	printf("e_shoff    : 0x%08x\n", ehdr->e_shoff);
	printf("e_flags    : 0x%08x\n", ehdr->e_flags);
	printf("e_ehsize   : 0x%04x\n", ehdr->e_ehsize);
	printf("e_phentsize: 0x%04x\n", ehdr->e_phentsize);
	printf("e_phnum    : 0x%04x\n", ehdr->e_phnum);
	printf("e_shentsize: 0x%04x\n", ehdr->e_shentsize);
	printf("e_shnum    : 0x%04x\n", ehdr->e_shnum);
	printf("e_shstrndx : 0x%04x\n", ehdr->e_shstrndx);
}

void elf32_ehdr_read(FILE *infile, ELF32_Ehdr *ehdr)
{
	ehdr->e_type      = fget16(infile);
	ehdr->e_machine   = fget16(infile);
	ehdr->e_version   = fget32(infile);
	ehdr->e_entry     = fget32(infile);
	ehdr->e_phoff     = fget32(infile);
	ehdr->e_shoff     = fget32(infile);
	ehdr->e_flags     = fget32(infile);
	ehdr->e_ehsize    = fget16(infile);
	ehdr->e_phentsize = fget16(infile);
	ehdr->e_phnum     = fget16(infile);
	ehdr->e_shentsize = fget16(infile);
	ehdr->e_shnum     = fget16(infile);
	ehdr->e_shstrndx  = fget16(infile);
}

void elf32_read_shdrs(FILE *infile, ELF_FILE *elf_file)
{
	int i;
	ELF32_Ehdr *ehdr = elf_file->ehdr;
	ELF32_Shdr *shdr;

	elf_file->shdr = malloc(ehdr->e_shnum * sizeof(ELF32_Shdr));

	for (i=0; i<ehdr->e_shnum; i++) {

		fseek(infile, ehdr->e_shoff + (i * ehdr->e_shentsize), SEEK_SET);

		shdr = elf_file->shdr + i;

		shdr->sh_name   = fget32(infile);
		shdr->sh_type   = fget32(infile);
		shdr->sh_flags  = fget32(infile);
		shdr->sh_addr   = fget32(infile);
		shdr->sh_offset = fget32(infile);
		shdr->sh_size   = fget32(infile);
		shdr->sh_link   = fget32(infile);
		shdr->sh_info   = fget32(infile);
		shdr->sh_addralign = fget32(infile);
		shdr->sh_entsize = fget32(infile);
	}
}

void elf32_shdr_dump(ELF_FILE *elf_file, ELF32_Shdr *shdr)
{
	if (elf_file->ehdr && elf_file->string_table)
		printf("sh_name     : 0x%08x (%s)\n", shdr->sh_name, elf_file->string_table + shdr->sh_name);
	else
		printf("sh_name     : 0x%08x\n", shdr->sh_name);

	printf("sh_type     : 0x%08x\n", shdr->sh_type);
	printf("sh_flags    : 0x%08x\n", shdr->sh_flags);
	printf("sh_addr     : 0x%08x\n", shdr->sh_addr);
	printf("sh_offset   : 0x%08x\n", shdr->sh_offset);
	printf("sh_size     : 0x%08x\n", shdr->sh_size);
	printf("sh_link     : 0x%08x\n", shdr->sh_link);
	printf("sh_info     : 0x%08x\n", shdr->sh_info);
	printf("sh_addralign: 0x%08x\n", shdr->sh_addralign);
	printf("sh_entsize  : 0x%08x\n", shdr->sh_entsize);
}

void elf32_read_phdrs(FILE *infile, ELF_FILE *elf_file)
{
	int i;
	ELF32_Ehdr *ehdr = elf_file->ehdr;
	ELF32_Phdr *phdr;

	elf_file->phdr = malloc(ehdr->e_phnum * sizeof(ELF32_Phdr));

	for (i=0; i<ehdr->e_phnum; i++) {

		fseek(infile, ehdr->e_phoff + (i * ehdr->e_phentsize), SEEK_SET);

		phdr = elf_file->phdr + i;

		phdr->p_type   = fget32(infile);
		phdr->p_offset = fget32(infile);
		phdr->p_vaddr  = fget32(infile);
		phdr->p_paddr  = fget32(infile);
		phdr->p_filesz = fget32(infile);
		phdr->p_memsz  = fget32(infile);
		phdr->p_flags  = fget32(infile);
		phdr->p_align  = fget32(infile);
	}
}

ELF_FILE *elf32_fopen(char *filename)
{
	char *data, buf[4];
	int c, i, size;
	FILE *infile;
	ELF32_Ehdr *ehdr;
	ELF32_Shdr *shdr;
	ELF_FILE *elf_file;

	ehdr = calloc(1, sizeof(*ehdr));

	if (!(infile = fopen(filename, "rb"))) {
		free(ehdr);
		return 0;
	}

	/* Verify the ELF signature */

	if (!fread(ehdr, 1, EI_NIDENT, infile))
		return 0;

	if ((ehdr->e_ident[EI_MAG0] != 0x7f) ||
	    (ehdr->e_ident[EI_MAG1] != 'E') ||
	    (ehdr->e_ident[EI_MAG2] != 'L') ||
	    (ehdr->e_ident[EI_MAG3] != 'F'))
		return 0;

	/* Set the endianness */

	switch (ehdr->e_ident[EI_DATA]) {

		case 1:

			big_endian_flag = 0;
			break;
	
		case 2:
			
			big_endian_flag = 1;
			break;

		default:

			printf("elf32_fopen: Unknown endian\n");
			abort();
	}

	/* Create struct to return */

	elf_file = calloc(1, sizeof(*elf_file));

	elf_file->infile = infile;
	elf_file->ehdr = ehdr;

	/* Read rest of ELF header */

	elf32_ehdr_read(infile, ehdr);

	elf32_ehdr_dump(ehdr);

	/* Read the program headers */

	elf32_read_phdrs(infile, elf_file);

	/* Read the section headers */

	elf32_read_shdrs(infile, elf_file);

	/* Read the string table */

	shdr = elf_file->shdr + ehdr->e_shstrndx;

	elf_file->string_table = malloc(shdr->sh_size);

	fseek(infile, shdr->sh_offset, SEEK_SET);

	fread(elf_file->string_table, 1, shdr->sh_size, infile);

	return elf_file;
}

int elf32_load(ELF_FILE *elf_file, MEM32_IMAGE *mem_image)
{
	void *buf;
	int i, pheaders_num, buf_size;
	ELF32_Phdr *phdr;

	pheaders_num = elf_file->ehdr->e_phnum;

	buf = 0;
	buf_size = 0;

	for (i=0; i<pheaders_num; i++) {

		phdr = elf_file->phdr + i;

		printf("pheader %d: p_paddr 0x%08x, p_memsz 0x%08x p_filesz 0x%08x\n",
			i, phdr->p_paddr, phdr->p_memsz, phdr->p_filesz);

		if (phdr->p_type != PT_LOAD)
			continue;

		if (buf_size < phdr->p_memsz)
			buf = realloc(buf, buf_size = phdr->p_memsz);

		fseek(elf_file->infile, phdr->p_offset, SEEK_SET);

		fread(buf, 1, phdr->p_filesz, elf_file->infile);

		/* ELF spec: "If the segment's memory size (p_memsz) is larger than the
		   file size (p_filesz), the "extra" bytes are defined to hold the value 0
		   and to follow the segment's initialized area." */

		if (phdr->p_filesz < phdr->p_memsz)
			memset(buf + phdr->p_filesz, 0, phdr->p_memsz - phdr->p_filesz);

		mem32_write_block(mem_image, phdr->p_paddr, phdr->p_memsz, buf);
	}

	free(buf);
}
