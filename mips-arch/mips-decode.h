
/*
** mips-decode.h
**
** Info about the MIPS instruction set shared across simulators.
*/

#ifndef _MIPS_DECODE_H
#define _MIPS_DECODE_H

/* Location of register fields in opcode. */

#define RS_SHIFTS 21
#define RT_SHIFTS 16
#define RD_SHIFTS 11
#define SA_SHIFTS 6

/*
** Decoded instruction
**
** Must stay synched with the cpu_op_name array.
*/

typedef enum {

	NOP,

	ALU_OP_ADD,		/* Add, result may cause overflow exception. */
	ALU_OP_ADDU,		/* Add, result will not cause overflow exception. */
	ALU_OP_AND,
	ALU_OP_DIV,		/* Signed divide */
	ALU_OP_DIV_CALC,
	ALU_OP_DIVU,		/* Unsigned divide */
	ALU_OP_DIVU_CALC,
	ALU_OP_MOVE,		/* Move op1 to a register. */
	ALU_OP_MULT0,		/* Signed multiply */
	ALU_OP_MULT1,
	ALU_OP_MULTU0,		/* Unsigned multiply */
	ALU_OP_MULTU1,
	ALU_OP_NOR,
	ALU_OP_OR,
	ALU_OP_SLL,		/* Shift left logical. */
	ALU_OP_SRA,		/* Shift right arithmetic. */
	ALU_OP_SRL,		/* Shift right logical. */
	ALU_OP_SLTS,		/* operand1 < operand2, signed compare */
	ALU_OP_SLTU,		/* operand1 < operand2, unsigned compare */
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

	MDU_OP_DIV,
	MDU_OP_DIVU,
	MDU_OP_MULT,
	MDU_OP_MULTU,
	MDU_OP_LAST,

	ALIGN_OP_LEFT,
	ALIGN_OP_RIGHT,

	WB_OP_WRITE_GENERAL_REG,
	WB_OP_WRITE_HI,
	WB_OP_WRITE_LO,
	WB_OP_WRITE_HI_LO,

} CPU_OP;

#ifdef DEFINE_CPU_OP_NAMES
char cpu_op_name[][32] = {

	{"NOP"},

	{"ALU_OP_ADD"},		/* Add, result may cause overflow exception. */
	{"ALU_OP_ADDU"},	/* Add, result will not cause overflow exception. */
	{"ALU_OP_AND"},
	{"ALU_OP_DIV"},		/* Signed divide */
	{"ALU_OP_DIV_CALC"},
	{"ALU_OP_DIVU"},	/* Unsigned divide */
	{"ALU_OP_DIVU_CALC"},
	{"ALU_OP_MOVE"},	/* Move op1 to a register. */
	{"ALU_OP_MULT0"},	/* Signed multiply */
	{"ALU_OP_MULT1"},
	{"ALU_OP_MULTU0"},	/* Unsigned multiply */
	{"ALU_OP_MULTU1"},
	{"ALU_OP_NOR"},
	{"ALU_OP_OR"},
	{"ALU_OP_SLL"},		/* Shift left logical. */
	{"ALU_OP_SRA"},		/* Shift right arithmetic. */
	{"ALU_OP_SRL"},		/* Shift right logical. */
	{"ALU_OP_SLTS"},	/* operand1 < operand2, signed compare */
	{"ALU_OP_SLTU"},	/* operand1 < operand2, unsigned compare */
	{"ALU_OP_SUB"},
	{"ALU_OP_SUBU"},
	{"ALU_OP_TEQ"},
	{"ALU_OP_TGE"},
	{"ALU_OP_TGEU"},
	{"ALU_OP_TLT"},
	{"ALU_OP_TLTU"},
	{"ALU_OP_TNE"},
	{"ALU_OP_XOR"},
	{"ALU_OP_LAST"},

	{"MEM_OP_NOP"},
	{"MEM_OP_LOAD_BYTE_SIGNED"},
	{"MEM_OP_LOAD_BYTE_UNSIGNED"},
	{"MEM_OP_LOAD_HALFWORD_SIGNED"},
	{"MEM_OP_LOAD_HALFWORD_UNSIGNED"},
	{"MEM_OP_LOAD_WORD_SIGNED"},
	{"MEM_OP_LOAD_WORD_UNSIGNED"},
	{"MEM_OP_LOAD_WORD_LEFT"},
	{"MEM_OP_LOAD_WORD_RIGHT"},

	{"MEM_OP_STORE_BYTE"},
	{"MEM_OP_STORE_HALFWORD"},
	{"MEM_OP_STORE_WORD"},
	{"MEM_OP_STORE_WORD_LEFT"},
	{"MEM_OP_STORE_WORD_RIGHT"},

	{"MDU_OP_DIV"},
	{"MDU_OP_DIVU"},
	{"MDU_OP_MULT"},
	{"MDU_OP_MULTU"},
	{"MDU_OP_LAST"},

	{"ALIGN_OP_LEFT"},
	{"ALIGN_OP_RIGHT"},

	{"WB_OP_WRITE_GENERAL_REG"},
	{"WB_OP_WRITE_HI"},
	{"WB_OP_WRITE_LO"},
	{"WB_OP_WRITE_HI_LO"},
};
#endif

#endif
