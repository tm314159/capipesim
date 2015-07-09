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

#include <stdint.h>
#include <stdio.h>

#include "mips-common.h"

/*************************************
* Start disassembler
*************************************/

void MIPS_disasm(uint32_t addr, uint32_t opcode, char *buf)
{
	int rs, rt, rd, sa;
	uint32_t major_op, immediate_bits, branch_target, jump_target;

	buf[0] = 0;

	major_op = opcode >> 26;
	rs = (opcode >> RS_SHIFTS) & 0x1f;
	rt = (opcode >> RT_SHIFTS) & 0x1f;
	rd = (opcode >> RD_SHIFTS) & 0x1f;
	sa = (opcode >> SA_SHIFTS) & 0x1f;
	immediate_bits = opcode & 0xffff;

	branch_target = addr + 4 + (int16_t)(opcode & 0xffff);
	jump_target   = (addr & 0xf0000000) | ((opcode & ((1 << 26) - 1)) << 2);

	if (!major_op) {		/* SPECIAL */

		switch (opcode & 0x3f) {

			case 0x00:

				sprintf(buf, "%08x: SLL $%d,$%d,%d", addr, rd, rt, sa);
				return;

			case 0x02:

				sprintf(buf, "%08x: SRL $%d,$%d,%d", addr, rd, rt, sa);
				return;

			case 0x03:

				sprintf(buf, "%08x: SRA $%d,$%d,%d", addr, rd, rt, sa);
				return;

			case 0x04:

				sprintf(buf, "%08x: SLLV $%d,$%d,$%d", addr, rd, rt, rs);
				return;

			case 0x06:

				sprintf(buf, "%08x: SRLV $%d,$%d,$%d", addr, rd, rt, rs);
				return;

			case 0x07:

				sprintf(buf, "%08x: SRAV $%d,$%d,$%d", addr, rd, rt, rs);
				return;

			case 0x08:

				sprintf(buf, "%08x: JR $%d", addr, rs);
				return;

			case 0x09:

				sprintf(buf, "%08x: JALR $%d,$%d", addr, rd, rs);
				return;

			case 0x0c:

				sprintf(buf, "%08x: SYSCALL", addr);
				break;

			case 0x0d:

				sprintf(buf, "%08x: BREAK 0x%x", addr, (opcode >> 6) & ((1 << 20) - 1));
				return;

			case 0x0f:

				sprintf(buf, "%08x: SYNC", addr);
				return;

			case 0x10:

				sprintf(buf, "%08x: MFHI $%d", addr, rd);
				return;

			case 0x11:

				sprintf(buf, "%08x: MTHI $%d", addr, rs);
				return;

			case 0x12:

				sprintf(buf, "%08x: MFLO $%d", addr, rd);
				return;

			case 0x13:

				sprintf(buf, "%08x: MTLO $%d", addr, rd);
				return;

			case 0x14:

				sprintf(buf, "%08x: DSLLV $%d,$%d,$%d", addr, rd, rt, rs);
				return;

			case 0x16:

				sprintf(buf, "%08x: DSRLV $%d,$%d,$%d", addr, rd, rt, rs);
				return;

			case 0x17:

				sprintf(buf, "%08x: DSRAV $%d,$%d,$%d", addr, rd, rt, rs);
				return;

			case 0x18:

				sprintf(buf, "%08x: MULT $%d,$%d", addr, rs, rt);
				return;

			case 0x19:

				sprintf(buf, "%08x: MULTU $%d,$%d", addr, rs, rt);
				return;

			case 0x1a:

				sprintf(buf, "%08x: DIV $%d,$%d", addr, rs, rt);
				return;

			case 0x1b:

				sprintf(buf, "%08x: DIVU $%d,$%d", addr, rs, rt);
				return;

			case 0x1c:

				sprintf(buf, "%08x: DMULT $%d,$%d", addr, rs, rt);
				return;

			case 0x1d:

				sprintf(buf, "%08x: DMULTU $%d,$%d", addr, rs, rt);
				return;

			case 0x1e:

				sprintf(buf, "%08x: DDIV $%d,$%d", addr, rs, rt);
				return;

			case 0x1f:

				sprintf(buf, "%08x: DDIVU $%d,$%d", addr, rs, rt);
				return;

			case 0x20:

				sprintf(buf, "%08x: ADD $%d,$%d,$%d", addr, rd, rs, rt);
				return;

			case 0x21:

				sprintf(buf, "%08x: ADDU $%d,$%d,$%d", addr, rd, rs, rt);
				return;

			case 0x22:

				sprintf(buf, "%08x: SUB $%d,$%d,$%d", addr, rd, rs, rt);
				return;

			case 0x23:

				sprintf(buf, "%08x: SUBU $%d,$%d,$%d", addr, rd, rs, rt);
				return;

			case 0x24:

				sprintf(buf, "%08x: AND $%d,$%d,$%d", addr, rd, rs, rt);
				return;

			case 0x25:

				sprintf(buf, "%08x: OR $%d,$%d,$%d", addr, rd, rs, rt);
				return;

			case 0x26:

				sprintf(buf, "%08x: XOR $%d,$%d,$%d", addr, rd, rs, rt);
				return;

			case 0x27:

				sprintf(buf, "%08x: NOR $%d,$%d,$%d", addr, rd, rs, rt);
				return;

			case 0x2a:

				sprintf(buf, "%08x: SLT $%d,$%d,$%d", addr, rd, rs, rt);
				return;

			case 0x2b:

				sprintf(buf, "%08x: SLTU $%d,$%d,$%d", addr, rd, rs, rt);
				return;

			case 0x2c:

				sprintf(buf, "%08x: DADD $%d,$%d,$%d", addr, rd, rs, rt);
				return;

			case 0x2e:

				sprintf(buf, "%08x: DSUB $%d,$%d,$%d", addr, rd, rs, rt);
				return;

			case 0x2d:

				sprintf(buf, "%08x: DADDU $%d,$%d,$%d", addr, rd, rs, rt);
				return;

			case 0x2f:

				sprintf(buf, "%08x: DSUBU $%d,$%d,$%d", addr, rd, rs, rt);
				return;

			case 0x30:

				sprintf(buf, "%08x: TGE $%d,$%d", addr, rs, rt);
				return;

			case 0x31:

				sprintf(buf, "%08x: TGEU $%d,$%d", addr, rs, rt);
				return;

			case 0x32:

				sprintf(buf, "%08x: TLT $%d,$%d", addr, rs, rt);
				return;

			case 0x33:

				sprintf(buf, "%08x: TLTU $%d,$%d", addr, rs, rt);
				return;

			case 0x34:

				sprintf(buf, "%08x: TEQ $%d,$%d", addr, rs, rt);
				return;

			case 0x36:

				sprintf(buf, "%08x: TNE $%d,$%d", addr, rs, rt);
				return;

			case 0x38:

				sprintf(buf, "%08x: DSLL $%d,$%d,$%d", addr, rd, rt, sa);
				return;

			case 0x3a:

				sprintf(buf, "%08x: DSRL $%d,$%d,$%d", addr, rd, rt, sa);
				return;

			case 0x3b:

				sprintf(buf, "%08x: DSRA $%d,$%d,$%d", addr, rd, rt, sa);
				return;

			case 0x3c:

				sprintf(buf, "%08x: DSLL32 $%d,$%d,$%d", addr, rd, rt, sa);
				return;

			case 0x3e:

				sprintf(buf, "%08x: DSRL32 $%d,$%d,$%d", addr, rd, rt, sa);
				return;

			case 0x3f:

				sprintf(buf, "%08x: DSRA32 $%d,$%d,$%d", addr, rd, rt, sa);
				return;
		}

	} else if (major_op == 1) {	/* REGIMM */

		switch ((opcode >> 16) & 0x1f) {

			case 0x00:

				sprintf(buf, "%08x: BLTZ $%d,0x%08x", addr, rs, branch_target);
				return;

			case 0x01:

				sprintf(buf, "%08x: BGEZ $%d,0x%08x", addr, rs, branch_target);
				return;

			case 0x02:

				sprintf(buf, "%08x: BLTZL $%d,0x%08x", addr, rs, branch_target);
				return;

			case 0x03:

				sprintf(buf, "%08x: BGEZL $%d,0x%08x", addr, rs, branch_target);
				return;

			case 0x08:

				sprintf(buf, "%08x: TGEI $%d,0x%x", addr, rs, immediate_bits);
				return;

			case 0x09:

				sprintf(buf, "%08x: TGEIU $%d,0x%x", addr, rs, immediate_bits);
				return;

			case 0x0a:

				sprintf(buf, "%08x: TLTI $%d,0x%x", addr, rs, immediate_bits);
				return;

			case 0x0b:

				sprintf(buf, "%08x: TLTIU $%d,0x%x", addr, rs, immediate_bits);
				return;

			case 0x0c:

				sprintf(buf, "%08x: TEQI $%d,0x%x", addr, rs, immediate_bits);
				return;

			case 0x0e:

				sprintf(buf, "%08x: TNEI $%d,0x%x", addr, rs, immediate_bits);
				return;

			case 0x10:

				sprintf(buf, "%08x: BLTZAL $%d,$%d,0x%08x", addr, rs, rt, branch_target);
				return;

			case 0x11:

				sprintf(buf, "%08x: BGEZAL $%d,0x%08x", addr, rs, branch_target);
				return;

			case 0x12:

				sprintf(buf, "%08x: BLTZALL $%d,0x%08x", addr, rs, branch_target);
				return;

			case 0x13:

				sprintf(buf, "%08x: BGEZALL $%d,0x%08x", addr, rs, branch_target);
				return;

		}

	} else {

		switch (major_op) {

			case 0x02:

				sprintf(buf, "%08x: J 0x%x", addr, jump_target);
				return;

			case 0x03:

				sprintf(buf, "%08x: JAL 0x%08x", addr, jump_target);
				return;
				
			case 0x04:

				sprintf(buf, "%08x: BEQ $%d,$%d,0x%08x", addr, rs, rt, branch_target);
				return;

			case 0x05:

				sprintf(buf, "%08x: BNE $%d,$%d,0x%08x", addr, rs, rt, branch_target);
				return;

			case 0x06:

				sprintf(buf, "%08x: BLEZ $%d,0x%08x", addr, rs, branch_target);
				return;

			case 0x07:

				sprintf(buf, "%08x: BGTZ $%d,0x%08x", addr, rs, branch_target);
				return;

			case 0x08:

				sprintf(buf, "%08x: ADDI $%d,$%d,0x%x", addr, rt, rs, immediate_bits);
				return;

			case 0x09:

				sprintf(buf, "%08x: ADDIU $%d,$%d,0x%x", addr, rt, rs, immediate_bits);
				return;

			case 0x0a:

				sprintf(buf, "%08x: SLTI $%d,$%d,0x%x", addr, rt, rs, immediate_bits);
				return;

			case 0x0b:

				sprintf(buf, "%08x: SLTIU $%d,$%d,0x%x", addr, rt, rs, immediate_bits);
				return;
				
			case 0x0c:

				sprintf(buf, "%08x: ANDI $%d,$%d,0x%x", addr, rt, rs, immediate_bits);
				return;

			case 0x0d:

				sprintf(buf, "%08x: ORI $%d,$%d,0x%x", addr, rt, rs, immediate_bits);
				return;

			case 0x0e:

				sprintf(buf, "%08x: XORI $%d,$%d,0x%x", addr, rt, rs, immediate_bits);
				return;

			case 0x0f:

				sprintf(buf, "%08x: LUI $%d,0x%x", addr, rt, immediate_bits);
				return;

			case 0x11:

				sprintf(buf, "%08x: BGEZAL $%d,0x%08x", addr, rs, branch_target);
				return;

			case 0x14:

				sprintf(buf, "%08x: BEQL $%d,$%d,0x%08x", addr, rt, rs, branch_target);
				return;

			case 0x15:

				sprintf(buf, "%08x: BNEL $%d,$%d,0x%08x", addr, rs, rt, branch_target);
				return;

			case 0x16:

				sprintf(buf, "%08x: BLEZL $%d,0x%08x", addr, rs, branch_target);
				return;

			case 0x17:

				sprintf(buf, "%08x: BGTZL $%d,0x%08x", addr, rs, branch_target);
				return;

			case 0x18:

				sprintf(buf, "%08x: DADDI $%d,$%d,0x%x", addr, rs, rt, immediate_bits);
				return;

			case 0x19:

				sprintf(buf, "%08x: DADDIU $%d,$%d,0x%x", addr, rs, rt, immediate_bits);
				return;

			case 0x1a:

				sprintf(buf, "%08x: LDL $%d,%d($%d)", addr, rt, immediate_bits, rs);
				return;

			case 0x1b:

				sprintf(buf, "%08x: LDR $%d,%d($%d)", addr, rt, immediate_bits, rs);
				return;

			case 0x20:

				sprintf(buf, "%08x: LB $%d,%d($%d)", addr, rt, immediate_bits, rs);
				return;

			case 0x21:

				sprintf(buf, "%08x: LH $%d,%d($%d)", addr, rt, immediate_bits, rs);
				return;

			case 0x22:

				sprintf(buf, "%08x: LWL $%d,%d($%d)", addr, rt, immediate_bits, rs);
				return;

			case 0x23:

				sprintf(buf, "%08x: LW $%d,%d($%d)", addr, rt, immediate_bits, rs);
				return;

			case 0x24:

				sprintf(buf, "%08x: LBU $%d,%d($%d)", addr, rt, immediate_bits, rs);
				return;

			case 0x25:

				sprintf(buf, "%08x: LHU $%d,%d($%d)", addr, rt, immediate_bits, rs);
				return;

			case 0x26:

				sprintf(buf, "%08x: LWR $%d,%d($%d)", addr, rt, immediate_bits, rs);
				return;

			case 0x28:

				sprintf(buf, "%08x: SB $%d,%d($%d)", addr, rt, immediate_bits, rs);
				return;

			case 0x29:

				sprintf(buf, "%08x: SH $%d,%d($%d)", addr, rt, immediate_bits, rs);
				return;

			case 0x2a:

				sprintf(buf, "%08x: SWL $%d,%d($%d)", addr, rt, immediate_bits, rs);
				return;

			case 0x2b:

				sprintf(buf, "%08x: SW $%d,%d($%d)", addr, rt, immediate_bits, rs);
				return;

			case 0x2c:

				sprintf(buf, "%08x: SDL $%d,%d($%d)", addr, rt, immediate_bits, rs);
				return;

			case 0x2d:

				sprintf(buf, "%08x: SDR $%d,%d($%d)", addr, rt, immediate_bits, rs);
				return;

			case 0x2e:

				sprintf(buf, "%08x: SWR $%d,%d($%d)", addr, rt, immediate_bits, rs);
				return;

			case 0x2f:

				sprintf(buf, "%08x: LWU $%d,%d($%d)", addr, rt, immediate_bits, rs);
				return;

			case 0x30:

				sprintf(buf, "%08x: LL $%d,%d($%d)", addr, rt, immediate_bits, rs);
				return;

			case 0x34:

				sprintf(buf, "%08x: LLD $%d,%d($%d)", addr, rt, immediate_bits, rs);
				return;

			case 0x37:

				sprintf(buf, "%08x: LD $%d,%d($%d)", addr, rt, immediate_bits, rs);
				return;

			case 0x38:

				sprintf(buf, "%08x: SC $%d,%d($%d)", addr, rt, immediate_bits, rs);
				return;

			case 0x3c:

				sprintf(buf, "%08x: SCD $%d,%d($%d)", addr, rt, immediate_bits, rs);
				return;

			case 0x3f:

				sprintf(buf, "%08x: SD $%d,%d($%d)", addr, rt, immediate_bits, rs);
				return;
		}
	}
}

/*************************************
* End disassembler
*************************************/

