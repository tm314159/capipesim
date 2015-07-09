/*
** mips_ioe5.c
**
** MIPS in-order execution pipeline with 5 stages
**
** by Toshiyasu Morita
**
** Started: 10/3/2013 @ 1:08 pm
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

/*
** Most of the technical information about the MIPS M4k has been sourced from:
** MIPS32 M4K(tm) Processor Core Software User's Manual, Revision 02.03.
**
** The MIPS M4K pipeline has five stages:
**
** I stage:
**   I-SRAM read
** 
** E stage:
**   Instruction decode,
**   Register file read,
**   ALU/shift ops part 1
**   Data address calculation
**
** M stage:
**   ALU/shift ops part 2
**   D-SRAM read
**
** A stage:
**   Load data aligner
**
** W stage:
**   Register file write
**
** For the purposes of simulation, the following changes are made:
**
** 1. The ALU/shift ops part 1 are moved into the M stage.
**
**    This is functionally equivalent to having the ALU/shift ops part 1 in the E stage because
**    the ALU result is not available until the end of the M stage in both cases.
*/

/*
** TODO list
**
** Split memory routines into separate file
** Fix LWL/LWR to align in A stage
*/


/*
** Notes:
**
** Coprocessor instructions (BCzF, etc) are not supported.
** 64-bit instructions are not supported.
** Multiprocessor instructions (LL/SC) are not supported.
** Only big-endian configuration currently supported.
*/

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../elf/mem.h"
#include "../elf/elf32.h"

#include "../mips-arch/mips-common.h"

#define GENERAL_REGS_NUM 32

typedef enum {

	ALU_OP_NOP,	/* Must be first. */
	ALU_OP_ADD,	/* Add, result may cause overflow exception. */
	ALU_OP_ADDU,	/* Add, result will not cause overflow exception. */
	ALU_OP_AND,
	ALU_OP_DIV,	/* Signed divide */
	ALU_OP_DIV_CALC,
	ALU_OP_DIVU,	/* Unsigned divide */
	ALU_OP_DIVU_CALC,
	ALU_OP_MOVE,	/* Move op1 to a register. */
	ALU_OP_MULT0,	/* Signed multiply */
	ALU_OP_MULT1,
	ALU_OP_MULTU0,	/* Unsigned multiply */
	ALU_OP_MULTU1,
	ALU_OP_NOR,
	ALU_OP_OR,
	ALU_OP_SLL,	/* Shift left logical. */
	ALU_OP_SRA,	/* Shift right arithmetic. */
	ALU_OP_SRL,	/* Shift right logical. */
	ALU_OP_SLTS,	/* operand1 < operand2, signed compare */
	ALU_OP_SLTU,	/* operand1 < operand2, unsigned compare */
	ALU_OP_SUB,
	ALU_OP_SUBU,
	ALU_OP_TEQ,
	ALU_OP_TGE,
	ALU_OP_TGEU,
	ALU_OP_TLT,
	ALU_OP_TLTU,
	ALU_OP_TNE,
	ALU_OP_XOR,
	ALU_OP_LAST,

} ALU_OP;

char alu_op_name[ALU_OP_LAST][32] = {
	"ALU_OP_NOP",
	"ALU_OP_ADD",
	"ALU_OP_ADDU",
	"ALU_OP_AND",
	"ALU_OP_DIV",
	"ALU_OP_DIV_CALC",
	"ALU_OP_DIVU",
	"ALU_OP_DIVU_CALC",
	"ALU_OP_MOVE",
	"ALU_OP_MULT0",
	"ALU_OP_MULT1",
	"ALU_OP_MULTU0"
	"ALU_OP_MULTU1",
	"ALU_OP_NOR",
	"ALU_OP_OR",
	"ALU_OP_SLL",
	"ALU_OP_SRA",
	"ALU_OP_SRL",
	"ALU_OP_SLTS",
	"ALU_OP_SLTU",
	"ALU_OP_SUB",
	"ALU_OP_SUBU",
	"ALU_OP_TEQ",
	"ALU_OP_TGE",
	"ALU_OP_TGEU",
	"ALU_OP_TLT",
	"ALU_OP_TLTU",
	"ALU_OP_TNE",
	"ALU_OP_XOR",
};

typedef enum {

	MDU_OP_NONE,
	MDU_OP_DIV,
	MDU_OP_DIVU,
	MDU_OP_MULT,
	MDU_OP_MULTU,
	MDU_OP_LAST,

} MDU_OP;

typedef enum {

	MEM_OP_NOP,
	MEM_OP_LOAD_BYTE_SIGNED,
	MEM_OP_LOAD_BYTE_UNSIGNED,
	MEM_OP_LOAD_HALFWORD_SIGNED,
	MEM_OP_LOAD_HALFWORD_UNSIGNED,
	MEM_OP_LOAD_WORD_SIGNED,
	MEM_OP_LOAD_WORD_UNSIGNED,
	MEM_OP_LOAD_WORD_LEFT,
	MEM_OP_LOAD_WORD_RIGHT,

	MEM_OP_STORE_BYTE,
	MEM_OP_STORE_HALFWORD,
	MEM_OP_STORE_WORD,
	MEM_OP_STORE_WORD_LEFT,
	MEM_OP_STORE_WORD_RIGHT,
	MEM_OP_LAST,

} MEM_OP;

char mem_op_name[MEM_OP_LAST][32] = {
	"MEM_OP_NOP",
	"MEM_OP_LOAD_BYTE_SIGNED",
	"MEM_OP_LOAD_BYTE_UNSIGNED",
	"MEM_OP_LOAD_HALFWORD_SIGNED",
	"MEM_OP_LOAD_HALFWORD_UNSIGNED",
	"MEM_OP_LOAD_WORD_SIGNED",
	"MEM_OP_LOAD_WORD_UNSIGNED",
	"MEM_OP_LOAD_WORD_LEFT",
	"MEM_OP_LOAD_WORD_RIGHT",

	"MEM_OP_STORE_BYTE",
	"MEM_OP_STORE_HALFWORD",
	"MEM_OP_STORE_WORD",
	"MEM_OP_STORE_WORD_LEFT",
	"MEM_OP_STORE_WORD_RIGHT",
};

typedef enum {

	ALIGN_OP_NOP,
	ALIGN_OP_LEFT,
	ALIGN_OP_RIGHT,

} ALIGN_OP;

typedef enum {

	WB_OP_NOP,
	WB_OP_WRITE_GENERAL_REG,
	WB_OP_WRITE_HI,
	WB_OP_WRITE_LO,
	WB_OP_WRITE_HI_LO,
	WB_OP_LAST,

} WB_OP;

char wb_op_name[WB_OP_LAST][32] = {

	"WB_OP_NOP",
	"WB_OP_WRITE_GENERAL_REG",
	"WB_OP_WRITE_HI",
	"WB_OP_WRITE_LO",
	"WB_OP_WRITE_HI_LO",
};

typedef enum {

	LO = 32,
	HI = 33,
	HI_LO = 34,

} REGNUM;

typedef struct {

	int valid_flag;
	uint32_t address, value;

} OPCODE_INFO;

typedef struct {

	ALU_OP op;			/* ALU operator */
	uint32_t operand1_value;	/* ALU operand1 value */
	uint32_t operand2_value;	/* ALU operand2 value */

} ALU_INFO;

typedef struct {

	MEM_OP op;
	uint32_t address;

} MEM_INFO;

typedef struct {

	int valid_flag;
	WB_OP op;
	int32_t reg_num;
	uint32_t orig_rt_value, value;
	uint32_t hi_value;		/* Only used when writing HI/LO pair */

} WB_INFO;

/* Data latched between I and E pipeline stages */

typedef struct {

	OPCODE_INFO opcode;

} I_E_LATCH;

/* Data latched between E and M pipeline stages */

typedef struct {

	OPCODE_INFO opcode;	/* For debug only */
	ALU_INFO alu;
	MEM_INFO mem;
	WB_INFO wb;

} E_M_LATCH;

/* Data latched between M and A pipeline stages */

 typedef struct {

	OPCODE_INFO opcode;	/* For debug only */
	MEM_INFO mem;
	WB_INFO wb;

} M_A_LATCH;

/* Data latched between A and W pipeline stages */

typedef struct {

	OPCODE_INFO opcode;	/* For debug only */
	WB_INFO wb;

} A_W_LATCH;

typedef struct {

	OPCODE_INFO opcode;	/* For debug only */
	ALU_INFO alu;
	WB_INFO wb;

} E_MDU_LATCH;

typedef struct {

	OPCODE_INFO opcode;	/* For debug only */
	WB_INFO wb;

} MDU_A_LATCH;

typedef struct {

	char *name;
	int valid_flag;
	uint32_t reg_num, value;

} BYPASS_STATE;

typedef struct {

	uint32_t value;
	int valid_flag;

} REG_WITH_VALID_FLAG;

typedef struct {

	/* Global state */

	uint32_t cycle_num;

	uint32_t pc, next_pc;
	REG_WITH_VALID_FLAG general_reg[GENERAL_REGS_NUM];	/* General registers are implemented as a dual-port register file. */
	REG_WITH_VALID_FLAG hi, lo;				/* hi and lo are implemented as flops. */

	/* MDU state */

	uint32_t mdu_operand1_value, mdu_operand2_value;
	int div_counter;

	/* Latches between pipeline stages */

	I_E_LATCH I_E_latch;
	E_M_LATCH E_M_latch;
	M_A_LATCH M_A_latch;
	A_W_LATCH A_W_latch;

	E_MDU_LATCH E_MDU_latch;
	MDU_A_LATCH MDU_A_latch;

	/* Bypasses */

	BYPASS_STATE M_E_bypass;
	BYPASS_STATE A_E_bypass;

	/* Memory image */

	MEM32_IMAGE *mem_image;

} CPU_STATE;

int debug_flag = 0;

int debug_address = 0x45a8;

void bypass_init(BYPASS_STATE *bypass, char *name)
{
	memset(bypass, 0, sizeof(*bypass));

	bypass->name = name;
}

void bypass_clear(BYPASS_STATE *bypass)
{
	bypass->valid_flag = 0;

	if (debug_flag)
		printf("  bypass %s cleared\n", bypass->name);
}

int bypass_check(BYPASS_STATE *bypass, int reg_num, uint32_t *value)
{
	if (bypass->valid_flag && (bypass->reg_num == reg_num)) {

		*value = bypass->value;

		if (debug_flag)
			printf("  bypass %s: $%d found, value: 0x%08x\n", bypass->name, reg_num, *value);
		return 1;
	}

	if (debug_flag)
		printf("  bypass %s: reg $%d not found\n", bypass->name, reg_num);

	return 0;
}

void bypass_update(BYPASS_STATE *bypass, WB_INFO *wb)
{
	bypass->valid_flag = wb->valid_flag;

	if (wb->valid_flag && (wb->op == WB_OP_WRITE_GENERAL_REG)) {

		bypass->reg_num = wb->reg_num;
		bypass->value   = wb->value;

		if (debug_flag)
			printf("  bypass %s: wrote reg $%d, value: 0x%08x\n", bypass->name, wb->reg_num, wb->value);

	} else 
		if (debug_flag)
			printf("  bypass %s cleared\n", bypass->name);
}

/*************************************
* Start memory code
*************************************/

uint32_t mem32_read_byte(MEM32_IMAGE *mem_image, uint32_t address)
{
	uint8_t *data = mem_image->data;
	uint32_t offset = address - mem_image->address;
	uint32_t value;

	if (offset >= mem_image->size) {
		printf("  Read from illegal address 0x%08x\n", address);
		abort();
	}

	value = data[offset];

	if (debug_flag)
		printf("  Read memory 0x%x (0x%02x)\n", address, value);

	return value;
}

uint32_t mem32_read_halfword(MEM32_IMAGE *mem_image, uint32_t address)
{
	uint8_t *data = mem_image->data;
	uint32_t offset = address - mem_image->address;
	uint32_t value;

	if (offset >= mem_image->size) {
		printf("  Read from illegal address 0x%08x\n", address);
		abort();
	}

	if (address & 1) {
		printf("  Read from unaligned address 0x%08x\n", address);
		abort();
	}

	if (mem_image->big_endian_flag) {
		value  = data[offset] << 8;
		value |= data[offset + 1];
	} else {
		value  = data[offset + 1];
		value |= data[offset] << 8;
	}

	if (debug_flag)
		printf("  Read memory 0x%x (0x%04x)\n", address, value);

	return value;
}

uint32_t mem32_read_word(MEM32_IMAGE *mem_image, uint32_t address)
{
	uint8_t *data = mem_image->data;
	uint32_t offset = address - mem_image->address;
	uint32_t value;

	if (offset >= mem_image->size) {
		printf("  Read from illegal address 0x%08x\n", address);
		abort();
	}

	if (address & 3) {
		printf("  Read from unaligned address 0x%08x\n", address);
		abort();
	}

	if (mem_image->big_endian_flag) {
		value  = data[offset] << 24;
		value |= data[offset + 1] << 16;
		value |= data[offset + 2] << 8;
		value |= data[offset + 3];
	} else {
		value  = data[offset + 3];
		value |= data[offset + 2] << 8;
		value |= data[offset + 1] << 16;
		value |= data[offset] << 24;
	}

	if (debug_flag)
		printf("  Read memory 0x%x (0x%08x)\n", address, value);

	return value;
}

void mem32_write_byte(MEM32_IMAGE *mem_image, uint32_t address, uint8_t value)
{
	uint8_t *data = mem_image->data;
	uint32_t offset = address - mem_image->address;

	if (offset >= mem_image->size) {
		printf("  Write to illegal address 0x%08x\n", address);
		abort();
	}

	data[offset] = value;

	if (debug_flag)
		printf("  Wrote memory 0x%x (0x%02x)\n", address, value & 0xff);
}

void mem32_write_halfword(MEM32_IMAGE *mem_image, uint32_t address, uint16_t value)
{
	uint8_t *data = mem_image->data;
	uint32_t offset = address - mem_image->address;

	if (offset >= mem_image->size) {
		printf("  Write to illegal address 0x%08x\n", address);
		abort();
	}

	if (address & 1) {
		printf("  Write to unaligned address 0x%08x\n", address);
		abort();
	}

	if (mem_image->big_endian_flag) {
		data[offset]     = value >> 8;
		data[offset + 1] = value;
	} else {
		data[offset]     = value;
		data[offset + 1] = value >> 8;
	}

	if (debug_flag)
		printf("  Wrote memory 0x%x (0x%04x)\n", address, value & 0xffff);
}

void mem32_write_word(MEM32_IMAGE *mem_image, uint32_t address, uint32_t value)
{
	uint8_t *data = mem_image->data;
	uint32_t offset = address - mem_image->address;

	if (offset >= mem_image->size) {
		printf("  Write to illegal address 0x%08x\n", address);
		abort();
	}

	if (address & 3) {
		printf("  Write to unaligned address 0x%08x\n", address);
		abort();
	}

	if (mem_image->big_endian_flag) {
		data[offset]     = value >> 24;
		data[offset + 1] = value >> 16;
		data[offset + 2] = value >> 8;
		data[offset + 3] = value;
	} else {
		data[offset]     = value;
		data[offset + 1] = value >> 8;
		data[offset + 2] = value >> 16;
		data[offset + 3] = value >> 24;
	}

	if (debug_flag && (debug_address == address))
		printf("   *** DEBUG: wrote to address 0x%08x (0x%08x)\n", address, value);

	if (debug_flag)
		printf("  Wrote memory 0x%x (0x%08x)\n", address, value);
}

void *mem32_get_pointer(MEM32_IMAGE *mem_image, uint32_t address)
{
	if ((address < mem_image->address) || (address >= (mem_image->address + mem_image->size))) {
		printf("  tried to get pointer illegal address 0x%08x which is outside memory image\n", address);
		abort();
	}

	return mem_image->data + (address - mem_image->address);
}

void mem32_write_block(MEM32_IMAGE *mem_image, uint32_t address, uint32_t size, void *buffer)
{
	uint32_t offset;
	if ((address < mem_image->address) || ((address + size) > (mem_image->address + mem_image->size))) {
		printf("mem32_write_block(): tried to load data outside memory image\n");
		printf("                     memory block: 0x%08x-0x%08x\n", mem_image->address, mem_image->address + mem_image->size - 1);
		printf("                     write  block: 0x%08x-0x%08x\n", address, address + size - 1);
		abort();
	}

	offset = address - mem_image->address;

	if (debug_flag)
		printf("  Wrote memory block 0x%08x - 0x%08x\n", address, address + size - 1);

	memcpy(mem_image->data + offset, buffer, size);
}


MEM32_IMAGE *mem32_init(uint32_t address, uint32_t size, int big_endian_flag)
{
	MEM32_IMAGE *mem_image;

	if (address & 3) {
		printf("mem32_init(): unaligned address 0x%08x\n", address);
		abort();
	}

	mem_image = calloc(1, sizeof(MEM32_IMAGE) + size);

	mem_image->address = address;
	mem_image->size    = size;
	mem_image->big_endian_flag = big_endian_flag;

	return mem_image;
}

/*************************************
* End memory code
*************************************/
/*************************************
* Start register code
*************************************/

void reg_init(CPU_STATE *cpu_state)
{
	int i;

	for (i=0; i<GENERAL_REGS_NUM; i++) {
		cpu_state->general_reg[i].value = i << 8;
		cpu_state->general_reg[i].valid_flag = 1;
	}

	cpu_state->hi.value = 32;
	cpu_state->lo.value = 33;

	cpu_state->hi.valid_flag = cpu_state->lo.valid_flag = 1;
}

int reg_read(CPU_STATE *cpu_state, uint32_t reg_num, uint32_t *value)
{
	REG_WITH_VALID_FLAG *reg;

	if (reg_num >= GENERAL_REGS_NUM)
		abort();

	reg = &cpu_state->general_reg[reg_num];

	*value = reg->value;

	if (debug_flag)
		printf("  Read $%d (0x%08x)\n", reg_num, *value);

	return reg->valid_flag;
}

void reg_write(CPU_STATE *cpu_state, uint32_t reg_num, uint32_t value)
{
	REG_WITH_VALID_FLAG *reg;

	if (reg_num >= GENERAL_REGS_NUM)
		abort();

	if (!reg_num)		/* r0 is always zero */
		return;

	if (debug_flag)
		printf("  Wrote $%d (0x%08x)\n", reg_num, value);

	reg = &cpu_state->general_reg[reg_num];

	reg->value = value;
	reg->valid_flag = 1;
}

void reg_clear_valid_flag(CPU_STATE *cpu_state, uint32_t reg_num)
{
	REG_WITH_VALID_FLAG *reg;

	if (reg_num >= GENERAL_REGS_NUM)
		abort();

	reg = &cpu_state->general_reg[reg_num];

	reg->valid_flag = 0;

	if (debug_flag)
		printf("  Cleared valid bit on $%d\n", reg_num);
}

/*************************************
* End register code
*************************************/

void m4k_delayed_jump(CPU_STATE *cpu_state, uint32_t value)
{
	cpu_state->next_pc = value;

	if (debug_flag)
		printf("  PC set to 0x%08x\n", value);
}

void  I_stage(CPU_STATE *cpu_state)
{
	uint32_t opcode;
	I_E_LATCH *output = &cpu_state->I_E_latch;

	if (debug_flag) {
		printf("I stage:\n");
		printf("  PC: 0x%08x, next_PC: 0x%08x\n", cpu_state->pc, cpu_state->next_pc);
	}

	if (output->opcode.valid_flag)
		return;

	opcode = mem32_read_word(cpu_state->mem_image, cpu_state->pc);

	output->opcode.address = cpu_state->pc;
	output->opcode.value   = opcode;
	output->opcode.valid_flag = 1;

	cpu_state->pc = cpu_state->next_pc;
	cpu_state->next_pc += 4;
}

int is_conversion_specifier(char c)
{
	switch (c) {

		case 'c':
		case 'd':
		case 'i':
		case 's':
		case 'x':

			return 1;
	}

	return 0;
}

typedef struct {

	int arg_num;
	CPU_STATE *cpu_state;

} MIPS_VA_LIST;

void mips_va_init(MIPS_VA_LIST *va_list, CPU_STATE *cpu_state)
{
	va_list->arg_num = 0;
	va_list->cpu_state = cpu_state;
}

int mips_va_arg(MIPS_VA_LIST *va_list)
{
	switch (va_list->arg_num++) {

		case 0:

			return va_list->cpu_state->general_reg[5].value;

		case 1:

			return va_list->cpu_state->general_reg[6].value;

		case 2:

			return va_list->cpu_state->general_reg[7].value;
	}

	return -1;
}

void m4k_handle_printf(MEM32_IMAGE *mem_image, CPU_STATE *cpu_state, uint32_t arg0)
{
	void *target_pointer;
	char c, *format, substring[256];
	int i;
	uint32_t value;
	MIPS_VA_LIST va_list;

	/* We split the format string into multiple format strings with only one conversion specifier each,
           then print each format string. */

	format = mem32_get_pointer(mem_image, arg0);

	mips_va_init(&va_list, cpu_state);

	for (;;) {

		i = 0;

		do
			c = format[i++];
		while (c && (c != '%'));

		if (!c) {
			printf("%s", format);
			return;
		}

		do
			c = format[i++];
		while (c && !is_conversion_specifier(c));

		if (!c) {
			printf("printf(): bad format string\n");
			abort();
		}

		strncpy(substring, format, i);
		substring[i] = 0;
		format += i;

		switch (c) {

			case 'c':
			case 'd':
				value = mips_va_arg(&va_list);
				printf(substring, value);
				break;

			case 's':
				value = mips_va_arg(&va_list);
				target_pointer = mem32_get_pointer(cpu_state->mem_image, value);
				printf(substring, target_pointer);
				break;
		}
	}
}

#define SYSCALL_PUTC 1
#define SYSCALL_PUTS 2
#define SYSCALL_PRINTF 3
#define SYSCALL_EXIT 4

void m4k_handle_syscall(CPU_STATE *cpu_state)
{
	int syscall_num, arg0;
	char *data;

	syscall_num = cpu_state->general_reg[2].value;
	arg0 = cpu_state->general_reg[4].value;

	switch (syscall_num) {

		case SYSCALL_PUTC:

			putc(arg0, stdout);
			break;

		case SYSCALL_PUTS:

			data = mem32_get_pointer(cpu_state->mem_image, arg0);
			puts(data);
			break;

		case SYSCALL_PRINTF:

			m4k_handle_printf(cpu_state->mem_image, cpu_state, arg0);
			break;

		case SYSCALL_EXIT:

			printf("*** Program exited\n");
			printf("%d clock cycles executed\n", cpu_state->cycle_num);
			fflush(stdout);
			exit(0);
			break;

		default:
			printf("*** UNHANDLED SYSCALL %d\n", syscall_num);
			break;
	}
	fflush(stdout);
}

void E_stage(CPU_STATE *cpu_state)
{
	I_E_LATCH *input = &cpu_state->I_E_latch;
	E_M_LATCH *E_M_output = &cpu_state->E_M_latch;
	E_MDU_LATCH *E_MDU_output = &cpu_state->E_MDU_latch;
	uint32_t opcode = input->opcode.value;
	uint32_t major_op, rs_reg_num, rt_reg_num, rd_reg_num, immediate_bits;
	uint32_t rs_value, rt_value, hi_value, lo_value;
	uint32_t branch_flag, jump_26bit_flag, jump_rs_flag;
	int rs_valid_flag, rt_valid_flag, hi_valid_flag, lo_valid_flag;
	char buffer[80];

	if (debug_flag)
		printf("E stage:\n");

	MIPS_disasm(input->opcode.address, opcode, buffer);

	/* Defaults to simplify each individual case. */
	E_M_output->alu.op = ALU_OP_NOP;
	E_M_output->mem.op = MEM_OP_NOP;
	E_M_output->wb.op = WB_OP_NOP;
	E_M_output->wb.valid_flag = 0;

	/* Don't clear MDU input if MDU is in the middle of an operation. */
	if (!E_MDU_output->opcode.valid_flag) {
		E_MDU_output->alu.op = ALU_OP_NOP;
		E_MDU_output->wb.op = WB_OP_NOP;
		E_MDU_output->wb.valid_flag = 0;
	}

	if (!input->opcode.valid_flag)
		return;

	/* Split opcode into fields to simplify following code. */
	major_op = opcode >> 26;
	rs_reg_num = (opcode >> RS_SHIFTS) & 0x1f;
	rt_reg_num = (opcode >> RT_SHIFTS) & 0x1f;
	rd_reg_num = (opcode >> RD_SHIFTS) & 0x1f;
	immediate_bits = opcode & 0xffff;

	/* Standard register reads. */

	rs_valid_flag = bypass_check(&cpu_state->M_E_bypass, rs_reg_num, &rs_value);

	if (!rs_valid_flag)
		rs_valid_flag = bypass_check(&cpu_state->A_E_bypass, rs_reg_num, &rs_value);

	if (!rs_valid_flag)
		rs_valid_flag = reg_read(cpu_state, rs_reg_num, &rs_value);

	rt_valid_flag = bypass_check(&cpu_state->M_E_bypass, rt_reg_num, &rt_value);

	if (!rt_valid_flag)
		rt_valid_flag = bypass_check(&cpu_state->A_E_bypass, rt_reg_num, &rt_value);

	if (!rt_valid_flag)
		rt_valid_flag = reg_read(cpu_state, rt_reg_num, &rt_value);

	/* rt is the output register for all non-SPECIAL instructions. */
	E_M_output->wb.reg_num = rt_reg_num;

	/* Original value of rt is required for LWL, LWR, etc instructions in the A pipeline stage. */
	E_M_output->wb.orig_rt_value = rt_value;

	E_M_output->alu.operand1_value = rs_value;
	E_M_output->alu.operand2_value = rt_value;

	E_MDU_output->alu.operand1_value = rs_value;
	E_MDU_output->alu.operand2_value = rt_value;

	hi_value = cpu_state->hi.value;
	lo_value = cpu_state->lo.value;
	hi_valid_flag = cpu_state->hi.valid_flag;
	lo_valid_flag = cpu_state->lo.valid_flag;

	/* Most instructions are not branches. */
	branch_flag = jump_26bit_flag = jump_rs_flag = 0;

	if (!major_op) { /* SPECIAL */

		switch (opcode & 0x3f) {

			case 0x00: /* SLL rd,rt,sa */

				if (!rt_valid_flag)
					goto rt_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->alu.op = ALU_OP_SLL;
				E_M_output->alu.operand1_value = (opcode >> SA_SHIFTS) & 0x1f;
				E_M_output->wb.op = WB_OP_WRITE_GENERAL_REG;
				E_M_output->wb.reg_num = rd_reg_num;
				break;

			case 0x02: /* SRL rd,rt,sa */

				if (!rt_valid_flag)
					goto rt_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->alu.op = ALU_OP_SRL;
				E_M_output->alu.operand1_value = (opcode >> SA_SHIFTS) & 0x1f;
				E_M_output->wb.op = WB_OP_WRITE_GENERAL_REG;
				E_M_output->wb.reg_num = rd_reg_num;
				break;

			case 0x03: /* SRA rd,rt,sa */

				if (!rt_valid_flag)
					goto rt_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->alu.op = ALU_OP_SRA;
				E_M_output->alu.operand1_value = (opcode >> SA_SHIFTS) & 0x1f;
				E_M_output->wb.op = WB_OP_WRITE_GENERAL_REG;
				E_M_output->wb.reg_num = rd_reg_num;
				break;

			case 0x04: /* SLLV rd,rt,rs */

				if (!rt_valid_flag)
					goto rt_invalid_stall;

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->alu.op = ALU_OP_SLL;
				E_M_output->wb.op = WB_OP_WRITE_GENERAL_REG;
				E_M_output->wb.reg_num = rd_reg_num;
				break;

			case 0x06: /* SRLV rd,rt,rs */

				if (!rt_valid_flag)
					goto rt_invalid_stall;

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->alu.op = ALU_OP_SRL;
				E_M_output->wb.op = WB_OP_WRITE_GENERAL_REG;
				E_M_output->wb.reg_num = rd_reg_num;
				break;

			case 0x07: /* SRAV rd,rt,rs */

				if (!rt_valid_flag)
					goto rt_invalid_stall;

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->alu.op = ALU_OP_SRA;
				E_M_output->wb.op  = WB_OP_WRITE_GENERAL_REG;
				E_M_output->wb.reg_num = rd_reg_num;
				break;

			case 0x08: /* JR rs */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				jump_rs_flag = 1;
				break;

			case 0x09: /* JALR rd,rs */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->wb.op = WB_OP_WRITE_GENERAL_REG;
				E_M_output->wb.reg_num = rd_reg_num;
				E_M_output->wb.value = input->opcode.address + 8;
				E_M_output->wb.valid_flag = 1;
				jump_rs_flag = 1;
				break;

			case 0x0c: /* SYSCALL */

				m4k_handle_syscall(cpu_state);
				break;

			case 0x0d: /* BREAK */

				break;

			case 0x0f: /* SYNC */

				break;

			case 0x10: /* MFHI rd */

				if (!hi_valid_flag)
					goto hi_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->alu.op = ALU_OP_MOVE;
				E_M_output->alu.operand1_value = hi_value;
				E_M_output->wb.op = WB_OP_WRITE_GENERAL_REG;
				E_M_output->wb.reg_num = rd_reg_num;
				break;

			case 0x11: /* MTHI rs */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->alu.op = ALU_OP_MOVE;
				E_M_output->wb.reg_num = HI;
				E_M_output->wb.op = WB_OP_WRITE_HI;
				break;

			case 0x12: /* MFLO rd */

				if (!lo_valid_flag)
					goto lo_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->alu.op = ALU_OP_MOVE;
				E_M_output->alu.operand1_value = lo_value;
				E_M_output->wb.op = WB_OP_WRITE_GENERAL_REG;
				E_M_output->wb.reg_num = rd_reg_num;
				break;

			case 0x13: /* MTLO rs */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->alu.op = ALU_OP_MOVE;
				E_M_output->wb.reg_num = LO;
				E_M_output->wb.op = WB_OP_WRITE_LO;
				break;

			case 0x18: /* MULT rs,rt */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (!rt_valid_flag)
					goto rt_invalid_stall;

				if (E_MDU_output->opcode.valid_flag)
					return;

				E_MDU_output->opcode = input->opcode;
				E_MDU_output->alu.op = ALU_OP_MULT0;
				E_MDU_output->wb.op = WB_OP_WRITE_HI_LO;
				break;

			case 0x19: /* MULTU rs,rt */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (!rt_valid_flag)
					goto rt_invalid_stall;

				if (E_MDU_output->opcode.valid_flag)
					return;

				E_MDU_output->opcode = input->opcode;
				E_MDU_output->alu.op = ALU_OP_MULTU0;
				E_MDU_output->wb.op = WB_OP_WRITE_HI_LO;
				break;

			case 0x1a: /* DIV rs,rt */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (!rt_valid_flag)
					goto rt_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_MDU_output->opcode = input->opcode;
				E_MDU_output->alu.op = ALU_OP_DIV;
				E_MDU_output->wb.op = WB_OP_WRITE_HI_LO;
				break;

			case 0x1b: /* DIVU rs,rt */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (!rt_valid_flag)
					goto rt_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_MDU_output->opcode = input->opcode;
				E_MDU_output->alu.op = ALU_OP_DIVU;
				E_MDU_output->wb.op = WB_OP_WRITE_HI_LO;
				break;

			case 0x20: /* ADD rd, rs, rt */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (!rt_valid_flag)
					goto rt_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->alu.op = ALU_OP_ADD;
				E_M_output->wb.op = WB_OP_WRITE_GENERAL_REG;
				E_M_output->wb.reg_num = rd_reg_num;
				break;

			case 0x21: /* ADDU rd,rs,rt */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (!rt_valid_flag)
					goto rt_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->alu.op = ALU_OP_ADDU;
				E_M_output->wb.op = WB_OP_WRITE_GENERAL_REG;
				E_M_output->wb.reg_num = rd_reg_num;
				break;

			case 0x22: /* SUB rd,rs,rt */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (!rt_valid_flag)
					goto rt_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->alu.op = ALU_OP_SUB;
				E_M_output->wb.op = WB_OP_WRITE_GENERAL_REG;
				E_M_output->wb.reg_num = rd_reg_num;
				break;

			case 0x23: /* SUBU rd,rs,rt */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (!rt_valid_flag)
					goto rt_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->alu.op = ALU_OP_SUBU;
				E_M_output->wb.op  = WB_OP_WRITE_GENERAL_REG;
				E_M_output->wb.reg_num = rd_reg_num;
				break;

			case 0x24: /* AND rd,rs,rt */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (!rt_valid_flag)
					goto rt_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->alu.op = ALU_OP_AND;
				E_M_output->wb.op  = WB_OP_WRITE_GENERAL_REG;
				E_M_output->wb.reg_num = rd_reg_num;
				break;

			case 0x25: /* OR rd,rs,rt */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (!rt_valid_flag)
					goto rt_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->alu.op = ALU_OP_OR;
				E_M_output->wb.op  = WB_OP_WRITE_GENERAL_REG;
				E_M_output->wb.reg_num = rd_reg_num;
				break;

			case 0x26: /* XOR rd,rs,rt */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (!rt_valid_flag)
					goto rt_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->alu.op = ALU_OP_XOR;
				E_M_output->wb.op  = WB_OP_WRITE_GENERAL_REG;
				E_M_output->wb.reg_num = rd_reg_num;
				break;

			case 0x27: /* NOR rd,rs,rt */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (!rt_valid_flag)
					goto rt_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->alu.op = ALU_OP_NOR;
				E_M_output->wb.op  = WB_OP_WRITE_GENERAL_REG;
				E_M_output->wb.reg_num = rd_reg_num;
				break;

			case 0x2a: /* SLT rd,rs,rt */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (!rt_valid_flag)
					goto rt_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->alu.op = ALU_OP_SLTS;
				E_M_output->wb.op  = WB_OP_WRITE_GENERAL_REG;
				E_M_output->wb.reg_num = rd_reg_num;
				break;

			case 0x2b: /* SLTU rd,rs,rt */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (!rt_valid_flag)
					goto rt_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->alu.op = ALU_OP_SLTU;
				E_M_output->wb.op  = WB_OP_WRITE_GENERAL_REG;
				E_M_output->wb.reg_num = rd_reg_num;
				break;

			case 0x30: /* TGE rs,rt */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (!rt_valid_flag)
					goto rt_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->alu.op = ALU_OP_TGE;
				break;

			case 0x31: /* TGEU rs,rt */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (!rt_valid_flag)
					goto rt_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->alu.op = ALU_OP_TGE;
				break;

			case 0x32: /* TLT rs,rt */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (!rt_valid_flag)
					goto rt_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->alu.op = ALU_OP_TLT;
				break;

			case 0x33: /* TLTU rs,rt */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (!rt_valid_flag)
					goto rt_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->alu.op = ALU_OP_TLTU;
				break;

			case 0x34: /* TEQ rs,rt */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (!rt_valid_flag)
					goto rt_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->alu.op = ALU_OP_TEQ;
				break;

			case 0x36: /* TNE rs,rt */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (!rt_valid_flag)
					goto rt_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->alu.op = ALU_OP_TNE;
				break;
		}

	} else if (major_op == 1) { /* REGIMM */

		/* REGIMM has a weird minor op. */

		switch ((opcode >> 16) & 0x1f) {

			case 0x00: /* BLTZ rs,offset */
			case 0x02: /* BLTZL rs,offset */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				branch_flag = rs_value & 0x80000000;
				break;

			case 0x01: /* BGEZ rs,offset */
			case 0x03: /* BGEZL rs,offset */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				branch_flag = !(rs_value & 0x80000000);
				break;

			case 0x08: /* TGEI rs,immediate */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->alu.op = ALU_OP_TGE;
				E_M_output->alu.operand2_value = immediate_bits;
				break;

			case 0x09: /* TGEIU rs,immediate */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->alu.op = ALU_OP_TGEU;
				E_M_output->alu.operand2_value = immediate_bits;
				break;

			case 0x0a: /* TLTI rs,immediate */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->alu.op = ALU_OP_TLT;
				E_M_output->alu.operand2_value = immediate_bits;
				break;

			case 0x0b: /* TLTIU rs,immediate */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->alu.op = ALU_OP_TLTU;
				E_M_output->alu.operand2_value = immediate_bits;
				break;

			case 0x0c: /* TEQI rs,immediate */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->alu.op = ALU_OP_TEQ;
				E_M_output->alu.operand2_value = immediate_bits;
				break;

			case 0x0e: /* TNEI rs,immediate */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->alu.op = ALU_OP_TNE;
				E_M_output->alu.operand2_value = immediate_bits;
				break;

			case 0x10: /* BLTZAL rs,offset */
			case 0x12: /* BLTZALL rs,offset */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				E_M_output->wb.op = WB_OP_WRITE_GENERAL_REG;
				E_M_output->wb.reg_num = 31;
				E_M_output->wb.value = input->opcode.address + 8;

				branch_flag = rs_value & 0x80000000;
				break;

			case 0x11: /* BGEZAL rs,offset */
			case 0x13: /* BGEZALL rs, offset */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				E_M_output->wb.op = WB_OP_WRITE_GENERAL_REG;
				E_M_output->wb.reg_num = 31;
				E_M_output->wb.value = input->opcode.address + 8;

				branch_flag = !(rs_value & 0x80000000);
				break;
		}

	} else {

		switch (major_op) {

			case 0x02: /* J target */

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				jump_26bit_flag = 1;
				break;

			case 0x03: /* JAL target */

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->wb.op = WB_OP_WRITE_GENERAL_REG;
				E_M_output->wb.reg_num = 31;
				E_M_output->wb.value = input->opcode.address + 8;
				E_M_output->wb.valid_flag = 1;

				jump_26bit_flag = 1;
				break;

			case 0x04: /* BEQ rs,rt,offset */
			case 0x14: /* BEQL rs,rt,offset */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (!rt_valid_flag)
					goto rt_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				branch_flag = (rs_value == rt_value);
				break;

			case 0x05: /* BNE rs,rt,offset */
			case 0x15: /* BNEL rs,rt,offset */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (!rt_valid_flag)
					goto rt_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				branch_flag = (rs_value != rt_value);
				break;

			case 0x06: /* BLEZ rs,offset */
			case 0x16: /* BLEZL rs,offset */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				branch_flag = (rs_value & 0x80000000) || !rs_value;
				break;

			case 0x07: /* BGTZ rs,offset */
			case 0x17: /* BGTZL rs,offset */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				branch_flag = !(rs_value & 0x80000000) && rs_value;
				break;

			case 0x08: /* ADDI rt,rs,immediate_bits */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->alu.op = ALU_OP_ADD;
				E_M_output->alu.operand2_value = (int32_t)(int16_t)immediate_bits;

				E_M_output->wb.op = WB_OP_WRITE_GENERAL_REG;
				E_M_output->wb.reg_num = rt_reg_num;
				break;

			case 0x09: /* ADDIU rt,rs,immediate_bits */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (!rt_valid_flag)
					goto rt_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->alu.op = ALU_OP_ADDU;
				E_M_output->alu.operand2_value = (int32_t)(int16_t)immediate_bits;

				E_M_output->wb.op = WB_OP_WRITE_GENERAL_REG;
				E_M_output->wb.reg_num = rt_reg_num;
				break;

			case 0x0a: /* SLTI rt,rs,immediate */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (!rt_valid_flag)
					goto rt_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->alu.op = ALU_OP_SLTS;
				E_M_output->alu.operand2_value = (int32_t)(int16_t)immediate_bits;

				E_M_output->wb.op = WB_OP_WRITE_GENERAL_REG;
				E_M_output->wb.reg_num = rt_reg_num;
				break;

			case 0x0b: /* SLTIU rt,rs,immediate */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (!rt_valid_flag)
					goto rt_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->alu.op = ALU_OP_SLTU;
				E_M_output->alu.operand2_value = (uint32_t)(uint16_t)immediate_bits;

				E_M_output->wb.op = WB_OP_WRITE_GENERAL_REG;
				E_M_output->wb.reg_num = rt_reg_num;
				break;

			case 0x0c: /* ANDI rt,rs,immediate */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (!rt_valid_flag)
					goto rt_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->alu.op = ALU_OP_AND;
				E_M_output->alu.operand2_value = immediate_bits;

				E_M_output->wb.op = WB_OP_WRITE_GENERAL_REG;
				E_M_output->wb.reg_num = rt_reg_num;
				break;

			case 0x0d: /* ORI rt,rs,immediate */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (!rt_valid_flag)
					goto rt_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->alu.op = ALU_OP_OR;
				E_M_output->alu.operand2_value = immediate_bits;

				E_M_output->wb.op = WB_OP_WRITE_GENERAL_REG;
				E_M_output->wb.reg_num = rt_reg_num;
				break;

			case 0x0e: /* XORI rt,rs,immediate */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (!rt_valid_flag)
					goto rt_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->alu.op = ALU_OP_XOR;
				E_M_output->alu.operand2_value = immediate_bits;

				E_M_output->wb.op = WB_OP_WRITE_GENERAL_REG;
				E_M_output->wb.reg_num = rt_reg_num;
				break;

			case 0x0f: /* LUI rt,immediate_bits */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (!rt_valid_flag)
					goto rt_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->alu.op = ALU_OP_MOVE;
				E_M_output->alu.operand1_value = immediate_bits << 16;

				E_M_output->wb.op = WB_OP_WRITE_GENERAL_REG;
				E_M_output->wb.reg_num = rt_reg_num;
				break;

			case 0x10: /* COP0 major op */

				/* Unimplemented for now. */
				break;

			case 0x1b: /* LDR rt,offset(base) */

				/* Unsupported - 64-bit operation. */
				break;

			case 0x20: /* LB rt,offset(base) */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->mem.op = MEM_OP_LOAD_BYTE_SIGNED;
				E_M_output->mem.address = rs_value + (int32_t)(int16_t)immediate_bits;

				E_M_output->wb.op = WB_OP_WRITE_GENERAL_REG;
				E_M_output->wb.reg_num = rt_reg_num;
				break;

			case 0x21: /* LH rt,offset(base) */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->mem.op = MEM_OP_LOAD_HALFWORD_SIGNED;
				E_M_output->mem.address = rs_value + (int32_t)(int16_t)immediate_bits;

				E_M_output->wb.op = WB_OP_WRITE_GENERAL_REG;
				E_M_output->wb.reg_num = rt_reg_num;
				break;

			case 0x22: /* LWL rt,offset(base) */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->mem.op = MEM_OP_LOAD_WORD_LEFT;
				E_M_output->mem.address = rs_value + (int32_t)(int16_t)immediate_bits;

				E_M_output->wb.op = WB_OP_WRITE_GENERAL_REG;
				E_M_output->wb.reg_num = rt_reg_num;
				break;

			case 0x23: /* LW rt,offset(base) */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->mem.op = MEM_OP_LOAD_WORD_SIGNED;
				E_M_output->mem.address = rs_value + (int32_t)(int16_t)immediate_bits;

				E_M_output->wb.op = WB_OP_WRITE_GENERAL_REG;
				E_M_output->wb.reg_num = rt_reg_num;
				break;

			case 0x24: /* LBU rt,offset(base) */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->mem.op = MEM_OP_LOAD_BYTE_UNSIGNED;
				E_M_output->mem.address = rs_value + (int32_t)(int16_t)immediate_bits;

				E_M_output->wb.op = WB_OP_WRITE_GENERAL_REG;
				E_M_output->wb.reg_num = rt_reg_num;
				break;

			case 0x25: /* LHU rt,offset(base) */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->mem.op = MEM_OP_LOAD_HALFWORD_UNSIGNED;
				E_M_output->mem.address = rs_value + (int32_t)(int16_t)immediate_bits;

				E_M_output->wb.op = WB_OP_WRITE_GENERAL_REG;
				E_M_output->wb.reg_num = rt_reg_num;
				break;

			case 0x26: /* LWR rt,offset(base) */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->mem.op = MEM_OP_LOAD_WORD_RIGHT;
				E_M_output->mem.address = rs_value + (int32_t)(int16_t)immediate_bits;

				E_M_output->wb.op = WB_OP_WRITE_GENERAL_REG;
				E_M_output->wb.reg_num = rt_reg_num;
				break;

			case 0x28: /* SB rt,offset(base) */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (!rt_valid_flag)
					goto rt_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->mem.op = MEM_OP_STORE_BYTE;
				E_M_output->mem.address = rs_value + (int32_t)(int16_t)immediate_bits;
				break;

	 		case 0x29: /* SH rt,offset(base) */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (!rt_valid_flag)
					goto rt_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->mem.op = MEM_OP_STORE_HALFWORD;
				E_M_output->mem.address = rs_value + (int32_t)(int16_t)immediate_bits;
				break;

	 		case 0x2a: /* SWL rt,offset(base) */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (!rt_valid_flag)
					goto rt_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->mem.op = MEM_OP_STORE_WORD_LEFT;
				E_M_output->mem.address = rs_value + (int32_t)(int16_t)immediate_bits;
				break;

	 		case 0x2b: /* SW rt,offset(base) */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (!rt_valid_flag)
					goto rt_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->mem.op = MEM_OP_STORE_WORD;
				E_M_output->mem.address = rs_value + (int32_t)(int16_t)immediate_bits;
				break;

			case 0x2c: /* SDL rt,offset(base) */

				/* Unsupported - 64-bit instruction. */
				E_M_output->wb.op = WB_OP_NOP;
				break;

			case 0x2d: /* SDR rt,offset(base) */

				/* Unsupported - 64-bit instruction. */
				E_M_output->wb.op = WB_OP_NOP;
				break;

			case 0x2e: /* SWR rt,offset(base) */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (!rt_valid_flag)
					goto rt_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->mem.op = MEM_OP_STORE_WORD_RIGHT;
				E_M_output->mem.address = rs_value + (int32_t)(int16_t)immediate_bits;
				break;

			case 0x2f: /* LWU rt,offset(base) */
				   /* Same as LW rt,offset(base) in a 32-bit implementation. */
				   /* NOTE: The MIPS R4000 Microprocessor User's Manual lists the CACHE instruction as having the same major op. */

				if (!rs_valid_flag)
					goto rs_invalid_stall;

				if (E_M_output->opcode.valid_flag)
					goto E_stage_stall;

				E_M_output->opcode = input->opcode;
				E_M_output->mem.op = MEM_OP_LOAD_WORD_UNSIGNED;
				E_M_output->mem.address = rs_value + (int32_t)(int16_t)immediate_bits;
				break;

			case 0x30: /* LL rt,offset(base) */

				/* Unsupported - multiprocessor instruction. */
				E_M_output->wb.op = WB_OP_NOP;
				break;

			case 0x32: /* LDL rt,offset(base) */

				/* Unsupported - 64-bit instruction. */
				E_M_output->wb.op = WB_OP_NOP;
				break;

			case 0x34: /* LLD rt,offset(base) */

				/* Unsupported - 64-bit multiprocessor instruction. */
				E_M_output->wb.op = WB_OP_NOP;
				break;

			case 0x37: /* LD rt,offset(base) */

				/* Unsupported - 64-bit instruction. */
				E_M_output->wb.op = WB_OP_NOP;
				break;

			case 0x38: /* SC rt,offset(base) */

				/* Unsupported - multiprocessor instruction. */
				E_M_output->wb.op = WB_OP_NOP;
				break;

			case 0x3c: /* SCD rt,offset(base) */

				/* Unsupported - 64-bit multiprocessor instruction. */
				E_M_output->wb.op = WB_OP_NOP;
				break;

			case 0x3f: /* SD rt,offset(base) */

				/* Unsupported - 64-bit instruction. */
				E_M_output->wb.op = WB_OP_NOP;
				break;
		}
	}

	/* Jump to a pc-relative address. */
	if (branch_flag) {
		immediate_bits = (int32_t)(int16_t)immediate_bits;
		m4k_delayed_jump(cpu_state, input->opcode.address + 4 + (immediate_bits << 2));
	}

	/* Jump to the addresss encoded in the opcode. */
	if (jump_26bit_flag)
		m4k_delayed_jump(cpu_state, (cpu_state->pc & 0xf0000000) | ((opcode & 0x3ffffff) << 2));

	/* Jump to the address contained in the rs register. */
	if (jump_rs_flag) {
		if (!rs_valid_flag)
			goto rs_invalid_stall;
		else
			m4k_delayed_jump(cpu_state, rs_value);
	}

	/* Clear the valid flag for the output register of the instruction. */

	switch (E_M_output->wb.op) {

		case WB_OP_NOP:
			break;

		case WB_OP_WRITE_GENERAL_REG:

			if (!E_M_output->wb.reg_num)		/* r0 is never invalid */
				break;
			
			reg_clear_valid_flag(cpu_state, E_M_output->wb.reg_num);
			break;

		default:
			printf("  *** ERROR in E stage: Unhandled WB_OP (%s) for M stage\n", wb_op_name[E_M_output->wb.op]);
			break;
	}

	switch (E_MDU_output->wb.op) {

		case WB_OP_NOP:

			break;

		case WB_OP_WRITE_HI:

			cpu_state->hi.valid_flag = 0;
			E_M_output->wb.reg_num = -1;
			break;

		case WB_OP_WRITE_LO:

			cpu_state->lo.valid_flag = 0;
			E_M_output->wb.reg_num = -1;
			break;

		case WB_OP_WRITE_HI_LO:

			cpu_state->hi.valid_flag = 0;
			cpu_state->lo.valid_flag = 0;
			E_M_output->wb.reg_num = -1;
			break;

		default:
			printf("  *** ERROR in E stage: Unhandled WB_OP (%s)for MDU stage\n", wb_op_name[E_MDU_output->wb.op]);
			break;
	}

	input->opcode.valid_flag = 0;
	return;

E_stage_stall:
	if (debug_flag)
		printf("  *** stalled - downstream stage busy\n");

	return;

rs_invalid_stall:
	if (debug_flag)
		printf("  *** RAW hazard - r%d not valid\n", rs_reg_num);

	E_M_output->opcode.address = E_M_output->opcode.value = E_M_output->opcode.valid_flag = 0;
	return;

rt_invalid_stall:
	if (debug_flag)
		printf("  *** RAW hazard - r%d not valid\n", rt_reg_num);

	E_M_output->opcode.address = E_M_output->opcode.value = E_M_output->opcode.valid_flag = 0;
	return;

hi_invalid_stall:
	if (debug_flag)
		printf("  *** RAW hazard - hi not valid\n");

	E_M_output->opcode.address = E_M_output->opcode.value = E_M_output->opcode.valid_flag = 0;
	return;

lo_invalid_stall:
	if (debug_flag)
		printf("  *** RAW hazard - lo not valid\n");

	E_M_output->opcode.address = E_M_output->opcode.value = E_M_output->opcode.valid_flag = 0;
	return;
}

void MDU_stage(CPU_STATE *cpu_state)
{
	E_MDU_LATCH *input = &cpu_state->E_MDU_latch;
	MDU_A_LATCH *output = &cpu_state->MDU_A_latch;
	uint32_t temp32;
	uint64_t temp64;

	if (debug_flag)
		printf("MDU stage:\n");

	/* Check if the A stage has consumed our output. If not, then stall for a clock. */
	if (output->opcode.valid_flag)
		return;

	switch (input->alu.op) {

		case ALU_OP_NOP:

			input->opcode.valid_flag = 0;
			break;

		case ALU_OP_DIV:

			temp32 = input->alu.operand1_value;

			if (!(temp32 & 0xffffff00))
				cpu_state->div_counter = 10;
			else if (!(temp32 & 0xffff0000))
				cpu_state->div_counter = 18;
			else if (!(temp32 & 0xffffff00))
				cpu_state->div_counter = 26;
			else
				cpu_state->div_counter = 34;

			if (debug_flag)
				printf("  Starting DIV, %d cycles left\n", cpu_state->div_counter);

			input->alu.op = ALU_OP_DIV_CALC;
			cpu_state->mdu_operand1_value = input->alu.operand1_value;
			cpu_state->mdu_operand2_value = input->alu.operand2_value;
			break;

		case ALU_OP_DIV_CALC:

			if (debug_flag)
				printf("  DIV in progress, %d cycles left\n", cpu_state->div_counter - 1);

			if (--cpu_state->div_counter)
				break;

			temp32 = input->alu.operand1_value | input->alu.operand2_value;

			output->opcode = input->opcode;
			output->wb = input->wb;

			input->opcode.valid_flag = 0;
			output->wb.hi_value = (int32_t)cpu_state->mdu_operand1_value % (int32_t)cpu_state->mdu_operand2_value;
			output->wb.value    = (int32_t)cpu_state->mdu_operand1_value / (int32_t)cpu_state->mdu_operand2_value;
			output->wb.valid_flag = 1;
			break;

		case ALU_OP_DIVU:

			temp32 = input->alu.operand1_value;

			if (!(temp32 & 0xffffff00))
				cpu_state->div_counter = 9;
			else if (!(temp32 & 0xffff0000))
				cpu_state->div_counter = 17;
			else if (!(temp32 & 0xffffff00))
				cpu_state->div_counter = 25;
			else
				cpu_state->div_counter = 33;

			if (debug_flag)
				printf("  Starting DIVU, %d cycles left\n", cpu_state->div_counter);

			input->alu.op = ALU_OP_DIVU_CALC;
			cpu_state->mdu_operand1_value = input->alu.operand1_value;
			cpu_state->mdu_operand2_value = input->alu.operand2_value;
			break;

		case ALU_OP_DIVU_CALC:

			if (debug_flag)
				printf("  DIVU in progress, %d cycles left\n", cpu_state->div_counter - 1);

			if (--cpu_state->div_counter)
				break;

			temp32 = input->alu.operand1_value | input->alu.operand2_value;

			output->opcode = input->opcode;
			output->wb = input->wb;

			input->opcode.valid_flag = 0;
			output->wb.hi_value = cpu_state->mdu_operand1_value % cpu_state->mdu_operand2_value;
			output->wb.value    = cpu_state->mdu_operand1_value / cpu_state->mdu_operand2_value;
			output->wb.valid_flag = 1;
			break;

		case ALU_OP_MULT0:

			if (input->alu.operand2_value & 0xffff0000) {
				input->alu.op = ALU_OP_MULT1;
				break;
			}

			/* Intentional fallthrough */

		case ALU_OP_MULT1:

			output->opcode = input->opcode;
			output->wb = input->wb;

			input->opcode.valid_flag = 0;
			temp64 = input->alu.operand1_value * input->alu.operand2_value;
			output->wb.value = temp64;
			output->wb.hi_value = temp64 >> 32;
			output->wb.valid_flag = 1;
			break;

		case ALU_OP_MULTU0:

			if (input->alu.operand2_value & 0xffff0000) {
				input->alu.op = ALU_OP_MULTU1;
				break;
			}

			/* Intentional fallthrough */

		case ALU_OP_MULTU1:

			output->opcode = input->opcode;
			output->wb = input->wb;

			input->opcode.valid_flag = 0;
			temp64 = (uint64_t)input->alu.operand1_value * (uint64_t)input->alu.operand2_value;
			output->wb.hi_value = temp64 >> 32;
			output->wb.value    = temp64;
			output->wb.valid_flag = 1;
			break;

		default:

			printf("  *** Error in MDU stage: Unhandled ALU operation %s\n", alu_op_name[input->alu.op]);
			abort();
	}
}

void  M_stage(CPU_STATE *cpu_state)
{
	E_M_LATCH *input = &cpu_state->E_M_latch;
	M_A_LATCH *output = &cpu_state->M_A_latch;
	uint32_t value, address;
	MEM32_IMAGE *mem_image;

	if (debug_flag) {
		printf("M stage:\n");
		printf("  ALU op: %d (%s)\n", input->alu.op, alu_op_name[input->alu.op]);
		printf("  mem op: %d (%s)\n", input->mem.op, mem_op_name[input->mem.op]);
	}

	if (output->opcode.valid_flag) {

		if (debug_flag)
			printf("  *** STALLED\n");

		return;
	}

	output->opcode = input->opcode;
	output->mem = input->mem;
	output->wb = input->wb;

	input->opcode.valid_flag = 0;

	switch (input->alu.op) {

		case ALU_OP_NOP:
			break;

		case ALU_OP_ADD:	/* Add, result may cause overflow exception. */
			output->wb.value = input->alu.operand1_value + input->alu.operand2_value;
			output->wb.valid_flag = 1;
			break;

		case ALU_OP_ADDU:	/* Add, result will not cause overflow exception. */
			output->wb.value = input->alu.operand1_value + input->alu.operand2_value;
			output->wb.valid_flag = 1;
			break;

		case ALU_OP_AND:	/* Bitwise AND. */
			output->wb.value = input->alu.operand1_value & input->alu.operand2_value;
			output->wb.valid_flag = 1;
			break;

		case ALU_OP_MOVE:	/* Move op1 to a register. */
			output->wb.value = input->alu.operand1_value;
			output->wb.valid_flag = 1;
			break;

		case ALU_OP_NOR:
			output->wb.value = (input->alu.operand1_value | input->alu.operand2_value) ^ 0xffffffff;
			output->wb.valid_flag = 1;
			break;

		case ALU_OP_OR:
			output->wb.value = input->alu.operand1_value | input->alu.operand2_value;
			output->wb.valid_flag = 1;
			break;

		case ALU_OP_SLL:	/* Shift left logical. */
			output->wb.value = input->alu.operand2_value << input->alu.operand1_value;
			output->wb.valid_flag = 1;
			break;

		case ALU_OP_SRA:	/* Shift right arithmetic. */
			output->wb.value = (int32_t)input->alu.operand2_value >> input->alu.operand1_value;
			output->wb.valid_flag = 1;
			break;

		case ALU_OP_SRL:	/* Shift right logical. */
			output->wb.value = input->alu.operand2_value >> input->alu.operand1_value;
			output->wb.valid_flag = 1;
			break;

		case ALU_OP_SLTS:	/* operand1 < operand2, signed compare */
			output->wb.value = (int32_t)input->alu.operand1_value < (int32_t)input->alu.operand2_value;
			output->wb.valid_flag = 1;
			break;

		case ALU_OP_SLTU:	/* operand1 < operand2, unsigned compare */
			output->wb.value = input->alu.operand1_value < input->alu.operand2_value;
			output->wb.valid_flag = 1;
			break;

		case ALU_OP_SUB:
			output->wb.value = input->alu.operand1_value - input->alu.operand2_value;
			output->wb.valid_flag = 1;
			break;

		case ALU_OP_SUBU:
			output->wb.value = input->alu.operand1_value - input->alu.operand2_value;
			output->wb.valid_flag = 1;
			break;

		case ALU_OP_TEQ:
			output->wb.value = input->alu.operand1_value == input->alu.operand2_value;
			output->wb.valid_flag = 1;
			break;

		case ALU_OP_TGE:
			output->wb.value = (int32_t)input->alu.operand1_value >= (int32_t)input->alu.operand2_value;
			output->wb.valid_flag = 1;
			break;

		case ALU_OP_TGEU:
			output->wb.value = input->alu.operand1_value >= input->alu.operand2_value;
			output->wb.valid_flag = 1;
			break;

		case ALU_OP_TLT:
			output->wb.value = (int32_t)input->alu.operand1_value < (int32_t)input->alu.operand2_value;
			output->wb.valid_flag = 1;
			break;

		case ALU_OP_TLTU:
			output->wb.value = input->alu.operand1_value < input->alu.operand2_value;
			output->wb.valid_flag = 1;
			break;

		case ALU_OP_TNE:
			output->wb.value = input->alu.operand1_value != input->alu.operand2_value;
			output->wb.valid_flag = 1;
			break;

		case ALU_OP_XOR:
			output->wb.value = input->alu.operand1_value ^ input->alu.operand2_value;
			output->wb.valid_flag = 1;
			break;

		default:
			printf("  *** Error in M stage: Unhandled ALU operation\n");
			abort();
	}

	switch (input->mem.op) {

		case MEM_OP_NOP:
			break;

		case MEM_OP_LOAD_BYTE_SIGNED:
			output->wb.value = (int32_t)(int8_t)mem32_read_byte(cpu_state->mem_image, input->mem.address);
			output->wb.valid_flag = 1;
			break;

		case MEM_OP_LOAD_BYTE_UNSIGNED:
			output->wb.value = (uint32_t)(uint8_t)mem32_read_byte(cpu_state->mem_image, input->mem.address);
			output->wb.valid_flag = 1;
			break;

		case MEM_OP_LOAD_HALFWORD_SIGNED:
			output->wb.value = (uint32_t)(int16_t)mem32_read_halfword(cpu_state->mem_image, input->mem.address);
			output->wb.valid_flag = 1;
			break;

		case MEM_OP_LOAD_HALFWORD_UNSIGNED:
			output->wb.value = (uint32_t)(uint16_t)mem32_read_halfword(cpu_state->mem_image, input->mem.address);
			output->wb.valid_flag = 1;
			break;

		case MEM_OP_LOAD_WORD_SIGNED:
			output->wb.value = mem32_read_word(cpu_state->mem_image, input->mem.address);
			output->wb.valid_flag = 1;
			break;

		case MEM_OP_LOAD_WORD_UNSIGNED:
			output->wb.value = mem32_read_word(cpu_state->mem_image, input->mem.address);
			output->wb.valid_flag = 1;
			break;

		case MEM_OP_LOAD_WORD_LEFT:

			/* Hardware would do this by using separate byte read enables. */
			mem_image = cpu_state->mem_image;
			address = input->mem.address;

			switch (address & 3) {

				case 0:
					output->wb.value = mem32_read_word(mem_image, address);
					break;
				case 1:
					output->wb.value = input->wb.orig_rt_value & 0xff;
					output->wb.value |= mem32_read_byte(mem_image, address) << 24;
					output->wb.value |= mem32_read_halfword(mem_image, address) << 8;
					break;
				case 2:
					output->wb.value = input->wb.orig_rt_value & 0xffff;
					output->wb.value |= mem32_read_halfword(mem_image, address) << 16;
					break;
				case 3:
					output->wb.value = input->wb.orig_rt_value & 0xffffff;
					output->wb.value |= mem32_read_byte(mem_image, address) << 8;
					break;
			}
			output->wb.valid_flag = 1;
			break;

		case MEM_OP_LOAD_WORD_RIGHT:

			/* Hardware would do this by using separate byte read enables. */
			mem_image = cpu_state->mem_image;
			address = input->mem.address;

			switch (address & 3) {

				case 0:
					break;
				case 1:
					output->wb.value = input->wb.orig_rt_value & 0xffffff00;
					output->wb.value |= mem32_read_byte(mem_image, address + 3);
					break;
				case 2:
					output->wb.value = input->wb.orig_rt_value & 0xffff0000;
					output->wb.value |= mem32_read_halfword(mem_image, address + 2);
					break;
				case 3:
					output->wb.value = input->wb.orig_rt_value & 0xff000000;
					output->wb.value |= mem32_read_halfword(mem_image, address + 1) << 8;
					output->wb.value |= mem32_read_byte(mem_image, address + 3);
					break;
			}
			output->wb.valid_flag = 1;
			break;

		case MEM_OP_STORE_BYTE:

			mem32_write_byte(cpu_state->mem_image, input->mem.address, input->wb.orig_rt_value);
			break;

		case MEM_OP_STORE_HALFWORD:
			mem32_write_halfword(cpu_state->mem_image, input->mem.address, input->wb.orig_rt_value);
			break;

		case MEM_OP_STORE_WORD:
			mem32_write_word(cpu_state->mem_image, input->mem.address, input->wb.orig_rt_value);
			break;

		case MEM_OP_STORE_WORD_LEFT:

			/* Hardware would do this by using separate byte write enables. */
			mem_image = cpu_state->mem_image;
			value   = input->wb.orig_rt_value;
			address = input->mem.address;

			switch (address & 3) {

				case 0:
					mem32_write_word(mem_image, address, value);
					break;
				case 1:
					mem32_write_byte(mem_image, address, value >> 24);
					mem32_write_halfword(mem_image, address + 1, (value >> 8) & 0xffff);
					break;
				case 2:
					mem32_write_halfword(mem_image, address, value >> 16);
					break;
				case 3:
					mem32_write_byte(cpu_state->mem_image, address, value >> 24);
					break;
			}
			break;

		case MEM_OP_STORE_WORD_RIGHT:

			/* Hardware would do this by using separate byte write enables. */
			mem_image = cpu_state->mem_image;
			value   = input->wb.orig_rt_value;
			address = input->mem.address;

			switch (address & 3) {

				case 0:
					break;
				case 1:
					mem32_write_byte(mem_image, address + 3, value & 0xff);
					break;
				case 2:
					mem32_write_halfword(mem_image, address + 2, value & 0xffff);
					break;
				case 3:
					mem32_write_halfword(mem_image, address + 1, (value >> 8) & 0xffff);
					mem32_write_byte(mem_image, address + 3, value & 0xff);
					break;
			}
			break;

		default:

			printf("M_stage(): Unhandled memory op\n");
			break;
	}

	bypass_update(&cpu_state->M_E_bypass, &output->wb);
}

void A_stage(CPU_STATE *cpu_state)
{
	M_A_LATCH *M_A_input = &cpu_state->M_A_latch;
	MDU_A_LATCH *MDU_A_input = &cpu_state->MDU_A_latch;
	A_W_LATCH *output = &cpu_state->A_W_latch;
	int bit_alignment, shifts, mask;

	if (debug_flag)
		printf("A stage:\n");

	if (output->opcode.valid_flag) {

		if (debug_flag)
			printf("  *** STALLED\n");

		return;
	}

	/* The A stage accepts input from both the M stage and the MDU stage.
           However, the MDU stage has priority, because otherwise the pipeline would lock up. */

	if (MDU_A_input->opcode.valid_flag) {

		output->opcode = MDU_A_input->opcode;
		output->wb     = MDU_A_input->wb;

		MDU_A_input->opcode.valid_flag = 0;

	} else {

		output->opcode = M_A_input->opcode;
		output->wb     = M_A_input->wb;

		M_A_input->opcode.valid_flag = 0;

		bit_alignment = (M_A_input->mem.address & 3) << 3;;

		switch (M_A_input->mem.op) {

			case MEM_OP_LOAD_WORD_LEFT:

				if (bit_alignment) {
					mask = (1 << bit_alignment) - 1;
					output->wb.value = (M_A_input->wb.value << bit_alignment) | (M_A_input->wb.orig_rt_value & mask);
				} else
					output->wb.value = M_A_input->wb.value;	/* R4000 manual explanation is hard to understand for this case. */
				break;

			case MEM_OP_LOAD_WORD_RIGHT:

				if (bit_alignment) {
					shifts = 32 - bit_alignment;
					mask = 0xffffffff ^ ((1 << bit_alignment) - 1);
					output->wb.value = (M_A_input->wb.value >> shifts) | (M_A_input->wb.orig_rt_value & mask);
				} else
					output->wb.value = M_A_input->wb.value;	/* R4000 manual explanation is hard to understand for this case. */
				break;

			default:
				break;
		}
	}

	bypass_update(&cpu_state->A_E_bypass, &output->wb);
}

void W_stage(CPU_STATE *cpu_state)
{
	A_W_LATCH *input = &cpu_state->A_W_latch;

	if (debug_flag)
		printf("W stage:\n");

	input->opcode.valid_flag = 0;

	if (!input->wb.valid_flag)
		return;

	switch (input->wb.op) {

		case WB_OP_WRITE_GENERAL_REG:

			reg_write(cpu_state, input->wb.reg_num, input->wb.value);
			break;

		case WB_OP_WRITE_HI:

			if (debug_flag)
				printf("  wrote hi=0x%x\n", input->wb.value);

			cpu_state->hi.value = input->wb.value;
			cpu_state->hi.valid_flag = 1;
			break;

		case WB_OP_WRITE_LO:

			if (debug_flag)
				printf("  wrote lo=0x%x\n", input->wb.value);

			cpu_state->lo.value = input->wb.value;
			cpu_state->lo.valid_flag = 1;
			break;

		case WB_OP_WRITE_HI_LO:

			if (debug_flag)
				printf("  wrote hi=0x%x, lo=0x%x\n", input->wb.hi_value, input->wb.value);

			cpu_state->hi.value = input->wb.hi_value;
			cpu_state->hi.valid_flag = 1;

			cpu_state->lo.value = input->wb.value;
			cpu_state->lo.valid_flag = 1;
			break;

		default:
			break;
	}
}

#if 1

void test_disasm(void)
{
	uint32_t opcode;
	char buf[80];

	printf("ADD rd,rs,rt ");
	opcode = 0x00000020 | (1 << RD_SHIFTS) | (2 << RS_SHIFTS) | (3 << RT_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("ADDI rt,rs,immediate ");
	opcode = 0x20000000 | (1 << RT_SHIFTS) | (2 << RS_SHIFTS) | 0x3000;
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("ADDIU rt,rs,immediate ");
	opcode = 0x24000000 | (1 << RT_SHIFTS) | (2 << RS_SHIFTS) | 0x3000;
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("ADDU rd,rs,rt ");
	opcode = 0x21 | (1 << RD_SHIFTS) | (2 << RS_SHIFTS) | (3 << RT_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("AND rd,rs,rt ");
	opcode = 0x24 | (1 << RD_SHIFTS) | (2 << RS_SHIFTS) | (3 << RT_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("ANDI rd,rs,rt ");
	opcode = 0x30000000 | (1 << RT_SHIFTS) | (2 << RS_SHIFTS) | 0x3000;
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("BEQ rs,rt,offset ");
	opcode = 0x10000000 | (1 << RS_SHIFTS) | (2 << RT_SHIFTS) | 0x2ffc;
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("BEQL rs,rt,offset ");
	opcode = 0x50000000 | (1 << RS_SHIFTS) | (2 << RT_SHIFTS) | 0x2ffc;
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

#define REGIMM_BITS 0x04000000
#define SPECIAL_BITS 0

	printf("BGEZ rs,offset ");
	opcode = REGIMM_BITS | 0x10000 | (1 << RS_SHIFTS) | 0x2ffc;
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("BGEZAL rs,offset ");
	opcode = REGIMM_BITS | 0x110000 | (1 << RS_SHIFTS) | 0x1ffc;
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("BGEZALL rs,offset ");
	opcode = REGIMM_BITS | 0x130000 | (1 << RS_SHIFTS) | 0x1ffc;
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("BGEZL rs,offset ");
	opcode = REGIMM_BITS | 0x030000 | (1 << RS_SHIFTS) | 0x1ffc;
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("BGTZ rs,offset ");
	opcode = 0x1c000000 | (1 << RS_SHIFTS) | 0x1ffc;
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("BGTZL rs,offset ");
	opcode = 0x5c000000 | (1 << RS_SHIFTS) | 0x1ffc;
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("BLEZ rs,offset ");
	opcode = 0x18000000 | (1 << RS_SHIFTS) | 0x1ffc;
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("BLEZL rs,offset ");
	opcode = 0x58000000 | (1 << RS_SHIFTS) | 0x1ffc;
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);
	
	printf("BLTZ rs,offset ");
	opcode = REGIMM_BITS | 0x000000 | (1 << RS_SHIFTS) | 0x1ffc;
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);
	
	printf("BLTZAL rs,offset ");
	opcode = REGIMM_BITS | 0x100000 | (1 << RS_SHIFTS) | 0x1ffc;
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);
	
	printf("BLTZALL rs,offset ");
	opcode = REGIMM_BITS | 0x120000 | (1 << RS_SHIFTS) | 0x1ffc;
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);
	
	printf("BLTZL rs,offset ");
	opcode = REGIMM_BITS | 0x020000 | (1 << RS_SHIFTS) | 0x1ffc;
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);
	
	printf("BNE rs,rt,offset ");
	opcode = 0x14000000 | (1 << RS_SHIFTS) | (2 << RT_SHIFTS) | 0x1ffc;
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);
	
	printf("BNEL rs,rt,offset ");
	opcode = 0x54000000 | (1 << RS_SHIFTS) | (2 << RT_SHIFTS) | 0x1ffc;
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);
	
	printf("BREAK code ");
	opcode = 0x0d | (0x12345 << 6);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);
	
	printf("DADD rd,rs,rt ");
	opcode = 0x2c | (1 << RD_SHIFTS) | (2 << RS_SHIFTS) | (3 << RD_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("DADDI rt,rs,immediate ");
	opcode = 0x60000000 | (1 << RT_SHIFTS) | (2 << RS_SHIFTS) | 0x1234;
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("DADDIU rt,rs,immediate ");
	opcode = 0x64000000 | (1 << RT_SHIFTS) | (2 << RS_SHIFTS) | 0x1234;
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("DADDU rd,rs,rt ");
	opcode = SPECIAL_BITS | 0x2d | (1 << RD_SHIFTS) | (2 << RS_SHIFTS) | (3 << RT_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("DDIV rs,rt ");
	opcode = SPECIAL_BITS | 0x1e | (1 << RS_SHIFTS) | (2 << RT_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("DDIVU rs,rt ");
	opcode = SPECIAL_BITS | 0x1f | (1 << RS_SHIFTS) | (2 << RT_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("DIV rs,rt ");
	opcode = SPECIAL_BITS | 0x1a | (1 << RS_SHIFTS) | (2 << RT_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("DIVU rs,rt ");
	opcode = SPECIAL_BITS | 0x1b | (1 << RS_SHIFTS) | (2 << RT_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("DMULTU rs,rt ");
	opcode = SPECIAL_BITS | 0x1d | (1 << RS_SHIFTS) | (2 << RT_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("DSLL rd,rt,sa ");
	opcode = SPECIAL_BITS | 0x38 | (1 << RD_SHIFTS) | (2 << RT_SHIFTS) | (3 << SA_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("DSLLV rd,rt,rs ");
	opcode = SPECIAL_BITS | 0x14 | (1 << RD_SHIFTS) | (2 << RT_SHIFTS) | (3 << RS_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("DSLL32 rd,rt,sa ");
	opcode = SPECIAL_BITS | 0x3c | (1 << RD_SHIFTS) | (2 << RT_SHIFTS) | (3 << SA_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("DSRA rd,rt,sa ");
	opcode = SPECIAL_BITS | 0x3b | (1 << RD_SHIFTS) | (2 << RT_SHIFTS) | (3 << SA_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("DSRAV rd,rt,rs ");
	opcode = SPECIAL_BITS | 0x17 | (1 << RD_SHIFTS) | (2 << RT_SHIFTS) | (3 << RS_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("DSRA32 rd,rt,sa ");
	opcode = SPECIAL_BITS | 0x3f | (1 << RD_SHIFTS) | (2 << RT_SHIFTS) | (3 << SA_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("DSRL rd,rt,sa ");
	opcode = SPECIAL_BITS | 0x3a | (1 << RD_SHIFTS) | (2 << RT_SHIFTS) | (3 << SA_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("DSRLV rd,rt,sa ");
	opcode = SPECIAL_BITS | 0x16 | (1 << RD_SHIFTS) | (2 << RT_SHIFTS) | (3 << RS_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("DSRL32 rd,rt,sa ");
	opcode = SPECIAL_BITS | 0x3e | (1 << RD_SHIFTS) | (2 << RT_SHIFTS) | (3 << SA_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("DSUB rd,rs,rt ");
	opcode = SPECIAL_BITS | 0x2e | (1 << RD_SHIFTS) | (2 << RT_SHIFTS) | (3 << SA_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("DSUBU rd,rs,rt ");
	opcode = SPECIAL_BITS | 0x2f | (1 << RD_SHIFTS) | (2 << RT_SHIFTS) | (3 << SA_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("J target ");
	opcode = 0x08ffffff;
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("JAL target ");
	opcode = 0x0cffffff;
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("JALR rd,rs ");
	opcode = SPECIAL_BITS | 0x09 | (1 << RD_SHIFTS) | (2 << RS_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("JR rs ");
	opcode = SPECIAL_BITS | 0x08 | (1 << RS_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("LB rt,offset(base) ");
	opcode = 0x80000000 | (1 << RT_SHIFTS) | 0x2000 | (3 << RS_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("LBU rt,offset(base) ");
	opcode = 0x90000000 | (1 << RT_SHIFTS) | 0x2000 | (3 << RS_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("LD rt,offset(base) ");
	opcode = 0xdc000000 | (1 << RT_SHIFTS) | 0x2000 | (3 << RS_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("LDL rt,offset(base) ");
	opcode = 0x68000000 | (1 << RT_SHIFTS) | 0x2000 | (3 << RS_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);
	
	printf("LDR rt,offset(base) ");
	opcode = 0x6c000000 | (1 << RT_SHIFTS) | 0x2000 | (3 << RS_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);
	
	printf("LH rt,offset(base) ");
	opcode = 0x84000000 | (1 << RT_SHIFTS) | 0x2000 | (3 << RS_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);
	
	printf("LHU rt,offset(base) ");
	opcode = 0x94000000 | (1 << RT_SHIFTS) | 0x2000 | (3 << RS_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);
	
	printf("LL rt,offset(base) ");
	opcode = 0xc0000000 | (1 << RT_SHIFTS) | 0x2000 | (3 << RS_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);
	
	printf("LLD rt,offset(base) ");
	opcode = 0xd0000000 | (1 << RT_SHIFTS) | 0x2000 | (3 << RS_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);
	
	printf("LUI rt,immediate ");
	opcode = 0x3c000000 | (1 << RT_SHIFTS) | 0x2000;
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("LW rt,offset(base) ");
	opcode = 0x8c000000 | (1 << RT_SHIFTS) | 0x2000 | (3 << RS_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("LWL rt,offset(base) ");
	opcode = 0x88000000 | (1 << RT_SHIFTS) | 0x2000 | (3 << RS_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("LWR rt,offset(base) ");
	opcode = 0x98000000 | (1 << RT_SHIFTS) | 0x2000 | (3 << RS_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("LWU rt,offset(base) ");
	opcode = 0xbc000000 | (1 << RT_SHIFTS) | 0x2000 | (3 << RS_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("MFHI rd ");
	opcode = SPECIAL_BITS | 0x10 | (1 << RD_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("MFLO rd ");
	opcode = SPECIAL_BITS | 0x12 | (1 << RD_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("MTHI rs ");
	opcode = SPECIAL_BITS | 0x11 | (1 << RS_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("MTLO rs ");
	opcode = SPECIAL_BITS | 0x13 | (1 << RS_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("MULT rs,rt ");
	opcode = SPECIAL_BITS | 0x18 | (1 << RS_SHIFTS) | (2 << RT_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("MULTU rs,rt ");
	opcode = SPECIAL_BITS | 0x19 | (1 << RS_SHIFTS) | (2 << RT_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("NOR rd,rs,rt ");
	opcode = SPECIAL_BITS | 0x27 | (1 << RS_SHIFTS) | (2 << RT_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("OR rd,rs,rt ");
	opcode = SPECIAL_BITS | 0x25 | (1 << RS_SHIFTS) | (2 << RT_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("ORI rt,rs,immediate ");
	opcode = 0x34000000 | (1 << RT_SHIFTS) | (2 << RS_SHIFTS) | 0x3000;
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("SB rt,offset(base) ");
	opcode = 0xa0000000 | (1 << RT_SHIFTS) | 0x2000 | (3 << RS_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("SC rt,offset(base) ");
	opcode = 0xe0000000 | (1 << RT_SHIFTS) | 0x2000 | (3 << RS_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("SCD rt,offset(base) ");
	opcode = 0xf0000000 | (1 << RT_SHIFTS) | 0x2000 | (3 << RS_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("SD rt,offset(base) ");
	opcode = 0xfc000000 | (1 << RT_SHIFTS) | 0x2000 | (3 << RS_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("SDL rt,offset(base) ");
	opcode = 0xb0000000 | (1 << RT_SHIFTS) | 0x2000 | (3 << RS_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("SDR rt,offset(base) ");
	opcode = 0xb4000000 | (1 << RT_SHIFTS) | 0x2000 | (3 << RS_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("SH rt,offset(base) ");
	opcode = 0xa4000000 | (1 << RT_SHIFTS) | 0x2000 | (3 << RS_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("SLL rd,rt,sa ");
	opcode = SPECIAL_BITS | 0x00 | (1 << RD_SHIFTS) | (2 << RT_SHIFTS) | (3 << SA_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("SLLV rd,rt,sa ");
	opcode = SPECIAL_BITS | 0x04 | (1 << RD_SHIFTS) | (2 << RT_SHIFTS) | (3 << SA_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("SLT rd,rs,rt ");
	opcode = SPECIAL_BITS | 0x2a | (1 << RD_SHIFTS) | (2 << RT_SHIFTS) | (3 << SA_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("SLTI rt,rs,immediate ");
	opcode = 0x28000000 | (1 << RS_SHIFTS) | (2 << RT_SHIFTS) | 0x3000;
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("SLTIU rt,rs,immediate ");
	opcode = 0x2c000000 | (1 << RS_SHIFTS) | (2 << RT_SHIFTS) | 0x3000;
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("SLTU rd,rs,rt ");
	opcode = SPECIAL_BITS | 0x2b | (1 << RD_SHIFTS) | (2 << RS_SHIFTS) | (3 << RT_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("SRA rd,rt,sa ");
	opcode = SPECIAL_BITS | 0x03 | (1 << RD_SHIFTS) | (2 << RT_SHIFTS) | (3 << SA_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("SRAV rd,rt,rs ");
	opcode = SPECIAL_BITS | 0x07 | (1 << RD_SHIFTS) | (2 << RT_SHIFTS) | (3 << RS_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("SRL rd,rt,sa ");
	opcode = SPECIAL_BITS | 0x02 | (1 << RD_SHIFTS) | (2 << RT_SHIFTS) | (3 << SA_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("SRLV rd,rt,rs ");
	opcode = SPECIAL_BITS | 0x06 | (1 << RD_SHIFTS) | (2 << RT_SHIFTS) | (3 << RS_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("SUB rd,rt,rs ");
	opcode = SPECIAL_BITS | 0x22 | (1 << RD_SHIFTS) | (2 << RT_SHIFTS) | (3 << RS_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("SUBU rd,rt,rs ");
	opcode = SPECIAL_BITS | 0x23 | (1 << RD_SHIFTS) | (2 << RT_SHIFTS) | (3 << RS_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("SW rt,offset(base) ");
	opcode = 0xac000000 | (1 << RT_SHIFTS) | 0x2000 | (3 << RS_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("SWL rt,offset(base) ");
	opcode = 0xa8000000 | (1 << RT_SHIFTS) | 0x2000 | (3 << RS_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("SWR rt,offset(base) ");
	opcode = 0xb8000000 | (1 << RT_SHIFTS) | 0x2000 | (3 << RS_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("SYNC ");
	opcode = 0xf;
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("SYSCALL ");
	opcode = 0xc;
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("TEQ rs,rt");
	opcode = 0x34 | (1 << RS_SHIFTS) | (2 << RT_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("TEQI rs,immediate ");
	opcode = REGIMM_BITS | 0xc0000 | (1 << RS_SHIFTS) | 0x2000;
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("TGE rs,rt ");
	opcode = 0x30 | (1 << RS_SHIFTS) | (2 << RT_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("TGEI rs,immediate ");
	opcode = REGIMM_BITS | 0x80000 | (1 << RS_SHIFTS) | 0x2000;
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("TGEIU rs,immediate ");
	opcode = REGIMM_BITS | 0x90000 | (1 << RS_SHIFTS) | 0x2000;
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("TGEU rs,rt ");
	opcode = 0x31 | (1 << RS_SHIFTS) | (2 << RT_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("TLT rs,rt ");
	opcode = 0x32 | (1 << RS_SHIFTS) | (2 << RT_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("TLTI rs,immediate ");
	opcode = REGIMM_BITS | 0xa0000 | (1 << RS_SHIFTS) | 0x2000;
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("TLTIU rs,immediate ");
	opcode = REGIMM_BITS | 0xb0000 | (1 << RS_SHIFTS) | 0x2000;
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("TLTU rs,rt ");
	opcode = 0x33 | (1 << RS_SHIFTS) | (2 << RT_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("TNE rs,rt ");
	opcode = 0x36 | (1 << RS_SHIFTS) | (2 << RT_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("TNEI rs,immediate ");
	opcode = REGIMM_BITS | 0xe0000 | (1 << RS_SHIFTS) | 0x2000;
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("XOR rd,rt,rs ");
	opcode = SPECIAL_BITS | 0x26 | (1 << RD_SHIFTS) | (2 << RT_SHIFTS) | (3 << RS_SHIFTS);
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

	printf("XORI rt,rs,immediate ");
	opcode = 0x38000000 | (1 << RT_SHIFTS) | (2 << RS_SHIFTS) | 0x3000;
	MIPS_disasm(0, opcode, buf);
	printf("(%s)\n", buf);

}

void m4k_init(CPU_STATE *cpu_state, MEM32_IMAGE *mem_image, uint32_t init_pc)
{
	int i;

	memset(cpu_state, 0, sizeof(*cpu_state));

	cpu_state->pc = init_pc;
	cpu_state->next_pc = cpu_state->pc + 4;

	for (i=0; i<GENERAL_REGS_NUM; i++)
		cpu_state->general_reg[i].valid_flag = 1;

	cpu_state->lo.valid_flag = cpu_state->hi.valid_flag = 1;

	cpu_state->I_E_latch.opcode.valid_flag = 0;
	cpu_state->E_M_latch.opcode.valid_flag = 0;
	cpu_state->M_A_latch.opcode.valid_flag = 0;
	cpu_state->A_W_latch.opcode.valid_flag = 0;

	cpu_state->mem_image = mem_image;

	bypass_init(&cpu_state->M_E_bypass, "M_E_bypass");
	bypass_init(&cpu_state->A_E_bypass, "A_E_bypass");
}

void m4k_execute(CPU_STATE *cpu_state, int clocks_num)
{
	char buffer[128];
	int i;

	for (i=0; i<clocks_num; i++) {

		if (debug_flag) {

			printf("---------- Clock %d pipeline state ----------\n", cpu_state->cycle_num);

			if (cpu_state->A_W_latch.opcode.valid_flag) {
				MIPS_disasm(cpu_state->A_W_latch.opcode.address, cpu_state->A_W_latch.opcode.value, buffer);
				printf("W: %s\n", buffer);
			} else
				printf("W:\n");

			if (cpu_state->MDU_A_latch.opcode.valid_flag) {
				MIPS_disasm(cpu_state->MDU_A_latch.opcode.address, cpu_state->MDU_A_latch.opcode.value, buffer);
				printf("A: %s\n", buffer);
			} else if (cpu_state->M_A_latch.opcode.valid_flag) {
				MIPS_disasm(cpu_state->M_A_latch.opcode.address, cpu_state->M_A_latch.opcode.value, buffer);
				printf("A: %s\n", buffer);
			} else
				printf("A:\n");

			if (cpu_state->E_M_latch.opcode.valid_flag) {
				MIPS_disasm(cpu_state->E_M_latch.opcode.address, cpu_state->E_M_latch.opcode.value, buffer);
				printf("M: %16s  ", buffer);
			} else
				printf("M:  ");

			if (cpu_state->E_MDU_latch.opcode.valid_flag) {
				MIPS_disasm(cpu_state->E_MDU_latch.opcode.address, cpu_state->E_MDU_latch.opcode.value, buffer);
				printf("MDU: %s\n", buffer);
			} else
				printf("MDU:\n");

			if (cpu_state->I_E_latch.opcode.valid_flag) {
				MIPS_disasm(cpu_state->I_E_latch.opcode.address, cpu_state->I_E_latch.opcode.value, buffer);
				printf("E: %s\n", buffer);
			} else
				printf("E:\n");

			printf("I: PC = 0x%08x\n", cpu_state->pc);
			printf("---------- Clock %d pipeline execution ----------\n", cpu_state->cycle_num);
		}

		/* The only pipeline stage which can trigger a stall is the E stage
		   for a RAW hazard, so only check for a stall in the E stage. */

		W_stage(cpu_state);
		A_stage(cpu_state);

		MDU_stage(cpu_state);

		M_stage(cpu_state);

		E_stage(cpu_state);

		I_stage(cpu_state);

		cpu_state->cycle_num++;
	}
}

#endif

void main(void)
{
	uint32_t addr;
	CPU_STATE cpu_state;
	MEM32_IMAGE *mem_image;
	ELF_FILE *elf_file;

	addr = 0;

	mem_image = mem32_init(addr, 0x10000, 1);

	m4k_init(&cpu_state, mem_image, addr);

	elf_file = elf32_fopen("dhry-mips");

	elf32_load(elf_file, mem_image);

	m4k_execute(&cpu_state, 100000);
}

