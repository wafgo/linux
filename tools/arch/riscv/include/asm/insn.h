/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef _ASM_RISCV_INSN_H
#define _ASM_RISCV_INSN_H

#include <linux/types.h>
#include <objtool/arch.h>

enum riscvc_insn_format {          /* 15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 0   */
	RISCVC_FMT_REG,            /* |  funct4  |   rd/rs1  |   rs2   | op | */
	RISCVC_FMT_IMM,            /* |funct3|imm|   rd/rs1  |   imm   | op | */
	RISCVC_FMT_STACK_REL_STORE,/* |funct3|      imm      |   rs2   | op | */
	RISCVC_FMT_WIDE_IMM,       /* |funct3|      imm          | rd´ | op | */
	RISCVC_FMT_LOAD,           /* |funct3|  imm    |rs1` |imm| rd´ | op | */
	RISCVC_FMT_STORE,          /* |funct3|  imm    |rs1` |imm|rs2` | op | */
	RISCVC_FMT_BRANCH,         /* |funct3| offset  |rs1` |  offset | op | */
	RISCVC_FMT_JUMP            /* |funct3|        jump target      | op | */
};

enum riscvg_insn_format {
	RISCVG_FMT_REG,
	RISCVG_FMT_IMM,
	RISCVG_FMT_STACK,
	RISCVG_FMT_BRANCH,
	RISCVG_FMT_U,
	RISCVG_FMT_JUMP
};


struct riscv_insn {
	int len;
	int immediate;
	enum insn_type type;
	enum riscvc_insn_format insn_fmt;
	union {
		u16 raw;
		struct riscv_hwi {
			u8 op0;
			u8 op1;
		} hwi;

		struct riscv_cr_format {
			unsigned op: 2;
			unsigned rs2: 5;
			unsigned rds1: 5;
			unsigned func4: 4;
		} cr;

		struct riscv_ci_format {
			unsigned op: 2;
			unsigned imm_l: 5;
			unsigned rds1: 5;
			unsigned imm_h: 1;
			unsigned func3: 3;
		} ci;

		struct riscv_css_format {
			unsigned op: 2;
			unsigned rs2: 5;
			unsigned imm: 6;
			unsigned func3: 3;
		} css;

		struct riscv_ciw_format {
			unsigned op: 2;
			unsigned rd_: 3;
			unsigned imm: 8;
			unsigned func3: 3;
		} ciw;

		struct riscv_cl_format {
			unsigned op: 2;
			unsigned rd_: 3;
			unsigned imm_l: 2;
			unsigned rs1_: 3;
			unsigned imm_h: 3;
			unsigned func3: 3;
		} cl;

		struct riscv_cs_format {
			unsigned op: 2;
			unsigned rs2_: 3;
			unsigned imm_l: 2;
			unsigned rs1_: 3;
			unsigned imm_h: 3;
			unsigned func3: 3;
		} cs;

		struct riscv_cb_format {
			unsigned op: 2;
			unsigned offset_l: 5;
			unsigned rs1_: 3;
			unsigned offset_h: 3;
			unsigned func3: 3;
		} cb;

		struct riscv_cj_format {
			unsigned op: 2;
			unsigned jump_target: 11;
			unsigned func3: 3;
		} cj;
	} rvc;
	union {
		u32 raw;
		struct riscv_wi {
			u8 op0;
			u8 op1;
			u8 op2;
			u8 op3;
		} wi;
		struct riscv_gj_format {
			unsigned op: 7;
			unsigned rd: 5;
			unsigned imm: 20;
		} gj;
		struct riscv_gb_format {
			unsigned op: 7;
			unsigned imm_l: 5;
			unsigned func: 3;
			unsigned rs1: 5;
			unsigned rs2: 5;
			unsigned imm_h: 7;
		} gb;
		struct riscv_gr_format {
			unsigned op: 7;
			unsigned rd: 5;
			unsigned func_l: 3;
			unsigned rs1: 5;
			unsigned rs2: 5;
			unsigned func_h: 7;
		} gr;
		struct riscv_gi_format {
			unsigned op: 7;
			unsigned rd: 5;
			unsigned func_l: 3;
			unsigned rs1: 5;
			unsigned imm: 12;
		} gi;
		struct riscv_gs_format {
			unsigned op: 7;
			unsigned imm_l: 5;
			unsigned func: 3;
			unsigned rs1: 5;
			unsigned rs2: 5;
			unsigned imm: 7;
		} gs;
		struct riscv_gu_format {
			unsigned op: 7;
			unsigned rd: 5;
			unsigned imm: 20;
		} gu;
	} rvg;
};

#endif /* _ASM_RISCV_INSN_H */
