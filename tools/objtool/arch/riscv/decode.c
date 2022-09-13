// SPDX-License-Identifier: GPL-2.0-or-later
/*
 */

#include <stdio.h>
#include <stdlib.h>

#define unlikely(cond) (cond)
#include <asm/insn.h>
#include <asm/nops.h>
#include <asm/orc_types.h>

#include <objtool/check.h>
#include <objtool/elf.h>
#include <objtool/arch.h>
#include <objtool/warn.h>
#include <objtool/endianness.h>
#include <objtool/builtin.h>
#include <arch/elf.h>
#include <inttypes.h>

/* #define DEBUG */

#if defined(DEBUG)
#define debug_insnc_printf(fmt,...) printf("%-4" PRIx64 ":  %.2x %.2x        "fmt" [OP: %i, FUNC: %i -> %s]\n",next_addr,  insn->rvc.hwi.op0, insn->rvc.hwi.op1, ##__VA_ARGS__,  op, func, rvc_opcode_map[func][op])
#define debug_insng_printf(fmt,...) printf("%-4" PRIx64 ":  %.2x %.2x %.2x %.2x  "fmt" [OP: %i, FUNC: %i -> %s]\n",next_addr,  insn->rvg.wi.op0, insn->rvg.wi.op1, insn->rvg.wi.op2, insn->rvg.wi.op3, ##__VA_ARGS__,  op, func, rvg_opcode_map[func][op])
#define debug_printf(fmt,...) printf(fmt" [%s(%i)]\n", ##__VA_ARGS__, __FUNCTION__, __LINE__)
#else
#define debug_printf(fmt,...) do { } while (0)
#define debug_insnc_printf(fmt,...) do { } while (0)
#define debug_insng_printf(fmt,...) do { } while (0)
#endif

#define RVC_REG(_r_) (_r_ + CFI_BP)
#define RVC_TO_RED_REG(_r_) (_r_ - CFI_BP)

#define ADD_OP(sop) \
	if (!(sop = calloc(1, sizeof(*sop)))) \
		return -1; \
	else for (saved_sop = sop, list_add_tail(&sop->list, ops_list); sop; sop = NULL)

static const char *riscv_reg_abi_names[] = {
	"zero", "ra", "sp", "gp", "tp", "t0","t1","t2","fp","s1","a0","a1","a2","a3","a4","a5","a6","a7","s2","s3","s4","s5","s6","s7","s8","s9","s10","s11","t3","t4","t5","t6"
};

static const bool callee_saved_regs[] = {
/*zero*/false, /*ra -> actually its caller saved but to workaround the x86 inspired implemenation we need to track it*/	true, /*sp*/	true, /*gp*/	false, /*tp*/	false, /*t0*/	false, /*t1*/	false, /*t2*/	false, /*fp*/	true, /*s1*/	true, /*a0*/	false, /*a1*/	false, /*a2*/	false, /*a3*/	false, /*a4*/	false, /*a5*/	false, /*a6*/	false, /*a7*/	false, /*s2*/	true, /*s3*/	true, /*s4*/	true, /*s5*/	true, /*s6*/	true, /*s7*/	true, /*s8*/	true, /*s9*/	true, /*s10*/	true, /*s11*/	true, /*t3*/	false, /*t4*/	false, /*t5*/	false, /*t6*/	false };

static uint64_t next_addr = 0;
static int is_64bit;

bool arch_callee_saved_reg(unsigned char reg)
{
	return callee_saved_regs[reg];
}

unsigned long arch_dest_reloc_offset(int addend)
{
	return addend;
}

unsigned long arch_jump_destination(struct instruction *insn)
{
	debug_printf("------> insn at offset: 0x%lx jumps to 0x%lx", insn->offset, insn->offset + insn->immediate);
	return insn->offset + insn->immediate;
}



static int get_instruction_length(struct riscv_insn *insn, const void *kaddr)
{
	memcpy(&insn->rvc.raw, kaddr, 2);

	if ((insn->rvc.hwi.op0 & 0x3) != 0x3) {
		insn->len = 2;
	} else if ((insn->rvc.hwi.op0 & 0x1c) != 0x1c) {
		insn->len = 4;
	} else if ((insn->rvc.hwi.op0 & 0x1f) != 0x1f) {
		insn->len = 6;
	} else if ((insn->rvc.hwi.op0 & 0x3f) != 0x3f) {
		insn->len = 8;
	} else if ((insn->rvc.hwi.op0 & 0x7f) != 0x7f) {
		u8 n = (insn->rvc.hwi.op0 >> 12) & 0x7;
		if (n != 0x7) {
			insn->len = ((80 + 16 * n) << 3);
		} else {
			// How to handle >= 192 bits?
			insn->len = -1;
		}
	} else {
		insn->len = -1;
	}
	return insn->len;
}


static int riscvc_decode(struct objtool_file *file, const struct section *sec, int offset, struct riscv_insn *insn, struct list_head *ops_list)
{
	int func, op;
	struct stack_op *sop = NULL;
	struct stack_op *saved_sop = NULL;
	
	static const char *rvc_opcode_map[8][4] = {
		{ "ADDI4SPN", "ADDI", "SLLI", ">16b" },
		{ "FLD-FLD-LQ", "JAL,ADDIW,ADDIW", "FLDSP,FLDSP,LQ", ">16b"},
		{ "LW", "LI", "LWSP", ">16b"},
		{ "FLW,LD,L", "LUI/ADDI16SP", "FLWSP,LDSP,LDSP", ">16b"},
		{ "reserved", "MISC-ALU", "J[AL]R/MV/ADD", ">16b"},
		{ "FSD,FSD,SQ", "J", "FSDSP,FSDSP,SQ", ">16b"},
		{ "SW", "BEQZ", "SWSP", ">16b"},
		{ "FSW,SD,SD", "BNEZ", "FSWSP,SDSP,SDSP", ">16b"},
	};

	func = (insn->rvc.hwi.op1 >> 5) & 0x7;
	op = insn->rvc.hwi.op0 & 0x3;

	if (func > 7 || op > 3) {
		debug_printf("Unsupported instruction detected");
		return -1;
	}

	switch (op) {
	case 0x0:
		if (func == 0) {
			struct riscv_ciw_format *ciw = &insn->rvc.ciw;
			if (ciw->rd_ == RVC_TO_RED_REG(CFI_BP)) {
				// addi fp, sp, imm
				int imm = ((ciw->imm << 1) & BIT(2)) | ((ciw->imm << 3) & BIT(3)) | ((ciw->imm >> 2) & 0x30) | ((ciw->imm << 4) & 0x3c0);
				ADD_OP(sop)
				{
					sop->src.type = OP_SRC_ADD;
					sop->src.reg = CFI_SP;
					sop->src.offset = imm;
					sop->dest.type = OP_DEST_REG;
					sop->dest.reg = CFI_BP;
				}
				insn->immediate = imm;
				debug_insnc_printf("addi %s, %s, %i", riscv_reg_abi_names[CFI_BP], riscv_reg_abi_names[CFI_SP], imm);
			}
		}
		break;
	case 0x1:
		switch(func) {
		case 0x0:
			if (insn->rvc.hwi.op0 == 0x1 && insn->rvc.hwi.op1 == 0)
				insn->type = INSN_NOP;
			else {
				/* addi sp, sp, imm*/
				struct riscv_ci_format *ci = &insn->rvc.ci;
				if (ci->rds1 == CFI_SP || ci->rds1 == CFI_BP) {
					struct {signed int s_ext:6;} imm = {.s_ext = ci->imm_l | (ci->imm_h << 5)};
					ADD_OP(sop)
					{
						sop->src.type = OP_SRC_ADD;
						sop->src.reg = ci->rds1;
						sop->src.offset = imm.s_ext;
						sop->dest.type = OP_DEST_REG;
						sop->dest.reg = ci->rds1;
					}
					insn->immediate = imm.s_ext;
					debug_insnc_printf("addi %s, %s, %i", riscv_reg_abi_names[ci->rds1], riscv_reg_abi_names[ci->rds1], imm.s_ext);
				}
			}
			break;
		case 0x1:
			if (is_64bit)  {
				/* addiw s/fp, s/fp, imm*/
				struct riscv_ci_format *ci = &insn->rvc.ci;
				if (ci->rds1 == CFI_SP || ci->rds1 == CFI_BP) {
					struct {signed int s_ext:6;} imm = {.s_ext = ci->imm_l | (ci->imm_h << 5)};
					ADD_OP(sop)
					{
						sop->src.type = OP_SRC_ADD;
						sop->src.reg = ci->rds1;
						sop->src.offset = imm.s_ext;
						sop->dest.type = OP_DEST_REG;
						sop->dest.reg = ci->rds1;
					}
					insn->immediate = imm.s_ext;
					debug_insnc_printf("addiw %s, %s, %i", riscv_reg_abi_names[ci->rds1], riscv_reg_abi_names[ci->rds1], imm.s_ext);
				}
			} else {
				/* */
				insn->type = INSN_CALL;
				insn->insn_fmt = RISCVC_FMT_JUMP;
			}
			break;
		case 0x3: {
			struct riscv_ci_format *ci = &insn->rvc.ci;
			// addi16sp
			if (ci->rds1 == CFI_SP) {
				struct {signed int s_ext:10;} imm = {.s_ext = (ci->imm_l & BIT(4)) | ((ci->imm_l << 5) & BIT(5)) | ((ci->imm_l << 3) & BIT(6)) |  ((ci->imm_l << 3) & BIT(6)) | ((ci->imm_l << 6) & 0x180) | (ci->imm_h << 9)};
				ADD_OP(sop)
				{
					sop->src.type = OP_SRC_ADD;
					sop->src.reg = CFI_SP;
					sop->src.offset = imm.s_ext;
					sop->dest.type = OP_DEST_REG;
					sop->dest.reg = CFI_SP;
				}
				debug_insnc_printf("addi sp, sp, %i", imm.s_ext);
			}
			break;
		}
		case 0x4:
			if ((insn->rvc.hwi.op1 & 0xc) == 0x8) {
				/* andi fp, fp, imm[5:0] */
				struct riscv_cb_format *cb = &insn->rvc.cb;
				if (RVC_REG(cb->rs1_) == CFI_BP) {
					struct {signed int s_ext:6;} imm = {.s_ext = cb->offset_l | (cb->offset_h << 5)};
					ADD_OP(sop)
					{
						sop->src.type = OP_SRC_AND;
						sop->src.reg = CFI_BP;
						sop->src.offset = imm.s_ext;
						sop->dest.type = OP_DEST_REG;
						sop->dest.reg = CFI_BP;
					}
					insn->immediate = imm.s_ext;
					debug_insnc_printf("andi fp, fp, %i ", imm.s_ext);
				}
			}
			break;
		case 0x5:{
			struct riscv_cj_format *cj = &insn->rvc.cj;
			struct {signed int s_ext:12;} imm;
			insn->type = INSN_JUMP_UNCONDITIONAL;
			insn->insn_fmt = RISCVC_FMT_JUMP;
			imm.s_ext = (cj->jump_target & 0xe) | ((cj->jump_target & BIT(9)) >> 5) | ((cj->jump_target << 5) & BIT(5)) | ((cj->jump_target << 1) & BIT(6)) | ((cj->jump_target << 3) & BIT(7)) | ((cj->jump_target << 1) & 0x300) | ((cj->jump_target << 4) & BIT(10)) | ((cj->jump_target) << 1) & BIT(11);
			insn->immediate = imm.s_ext;
			debug_insnc_printf("j 0x%"PRIx64, imm.s_ext + next_addr);
			break;
		}
		case 0x6 ... 0x7: {
			struct riscv_cb_format *cb = &insn->rvc.cb;
			struct {signed int s_ext:9;} imm;
			insn->type = INSN_JUMP_CONDITIONAL;
			insn->insn_fmt = RISCVC_FMT_BRANCH;
			imm.s_ext = ((cb->offset_l) & 0x6) | ((cb->offset_h) << 3) & 0x18 | ((cb->offset_l << 5) & BIT(5)) | ((cb->offset_l << 3) & 0xc0) | ((cb->offset_h << 6) & BIT(8));
			insn->immediate = imm.s_ext;
			debug_insnc_printf("%s %s, 0x%lx", func == 0x6 ? "beqz"  : "bneq", riscv_reg_abi_names[RVC_REG(cb->rs1_)], imm.s_ext + next_addr);
			break;
		}
		default:
			break;
		}
		break;
	case 0x2:
		switch(func) {
		case 0x3:{
			if (is_64bit) { //ld	rs2, imm[8:3](sp)
				struct riscv_ci_format *ci = &insn->rvc.ci;
				int imm = ((ci->imm_l & 0x18) | (ci->imm_h << 5) | ((ci->imm_l << 6) & 0x1c0));
				ADD_OP(sop)
				{
					sop->src.type = OP_SRC_REG_INDIRECT;
					sop->src.reg = CFI_SP;
					sop->src.offset = imm;
					sop->dest.type = OP_DEST_REG;
					sop->dest.reg = ci->rds1;

				}
				insn->immediate = imm;
				debug_insnc_printf("ld %s, %i(sp)", riscv_reg_abi_names[ci->rds1], imm);
			}
		}
			break;
		case 0x4: {
			struct riscv_cr_format *cr = &insn->rvc.cr;
			if (cr->func4 & 0x1) { /* ebreak || jalr || add*/
				if (cr->rds1 == CFI_ZERO && cr->rs2 == CFI_ZERO) { /* ebreak */
					insn->type = INSN_TRAP;
					debug_insnc_printf("ebreak");
				} else if (cr->rs2 == CFI_ZERO && cr->rds1 != CFI_ZERO) { /* jalr */
					insn->type = INSN_CALL_DYNAMIC;
					debug_insnc_printf("jalr %s", riscv_reg_abi_names[cr->rds1]);
				} else if (cr->rds1 != CFI_ZERO && cr->rs2 != CFI_ZERO) { /* add*/
					debug_insnc_printf("add %s, %s", riscv_reg_abi_names[cr->rs2], riscv_reg_abi_names[cr->rds1]);
				}
			} else { /* jr || mv*/
				if (cr->rs2 == CFI_ZERO && cr->rds1 != CFI_ZERO) { /* jr */
					if (cr->rds1 == CFI_RA) {
						insn->type = INSN_RETURN;
						debug_insnc_printf("ret");
					} else {
						insn->type = INSN_JUMP_DYNAMIC;
						debug_insnc_printf("jr %s", riscv_reg_abi_names[cr->rds1]);
					}
				} else if(cr->rs2 != CFI_ZERO && cr->rds1 != CFI_ZERO) { /* mv */
					if (cr->rs2 == CFI_SP || cr->rds1 == CFI_SP) {
						ADD_OP(sop)
						{
							sop->src.type = OP_SRC_REG;
							sop->src.reg = cr->rs2;
							sop->dest.type = OP_DEST_REG;
							sop->dest.reg = cr->rds1;
						}
						debug_insnc_printf("mv %s, %s", riscv_reg_abi_names[cr->rs2], riscv_reg_abi_names[cr->rds1]);
					}
				}
			}
			break;
		}

		case 0x7:{
			if (is_64bit) { //sd	rs2, imm[8:3](sp)
				struct riscv_css_format *css = &insn->rvc.css;

				int imm = (((css->imm >> 3) & 0x7) | ((css->imm << 3) & 0x38)) << 3;
				ADD_OP(sop)
				{
					sop->src.type = OP_SRC_REG;
					sop->src.reg = css->rs2;
					sop->dest.type = OP_DEST_REG_INDIRECT;
					sop->dest.reg = CFI_SP;
					sop->dest.offset = imm;
				}
				insn->immediate = imm;
				debug_insnc_printf("sd %s, %i(sp)",riscv_reg_abi_names[css->rs2], imm);
			}
			break;
		}

		default:
			break;
		}
		break;
	default:
		break;
	}

	if (saved_sop == NULL && insn->type == INSN_OTHER) {
		debug_insnc_printf("----");
	}
	
	return 0;
}

static inline const char *riscvg_get_cond_mnemonic(int func)
{
	switch (func) {
	case 0x0:
		return "beq";
	case 0x1:
		return "bne";
	case 0x4:
		return "blt";
	case 0x5:
		return "bge";
	case 0x6:
		return "bltu";
	case 0x7:
		return "bgeu";
	default:
		return "unknown";

	}
}

#define __elf_table(name)	(file->elf->name##_hash)
#define __elf_bits(name)	(file->elf->name##_bits)

#define elf_hash_add(name, node, key) \
	hlist_add_head(node, &__elf_table(name)[hash_min(key, __elf_bits(name))])

static int riscvg_decode(struct objtool_file *file, const struct section *sec, int offset, struct riscv_insn *insn, struct list_head *ops_list)
{
	int op, func;
	int opcode;
	struct stack_op *sop = NULL;
	struct stack_op *saved_sop = NULL;
	static struct auipc_op_info {
		int imm;
		struct reloc *reloc;
		bool exec;
		int offset;
	} auipc_info = {0};
	
	static const char *rvg_opcode_map[8][4] = {
		{ "LOAD", "STORE", "MADD", "BRANCH" },
		{ "LOAD-FP", "STORE-FP", "MSUB", "JALR"},
		{ "custom-0", "custom-1", "NMSUB", "reserved"},
		{ "MISC-MEM", "AMO", "NMADD", "JAL"},
		{ "OP-IMM", "OP", "OP-FP", "SYSTEM"},
		{ "AUIPC", "LUI", "reserved", "reserved"},
		{ "OP-IMM-32", "OP-32", "custom-2/rv-128", "custom-2/rv128"},
		{ "48b", "64b", "48b", ">=80b"},
	};

	func = (insn->rvc.hwi.op0 >> 2) & 0x7;
	op = (insn->rvc.hwi.op0 >> 5) & 0x3;

	opcode = insn->rvg.raw & 0x7f;

	if (func > 7 || op > 3) {
		debug_printf("Invalid instruction detected");
		return -1;
	}

	switch (opcode) {
	case 0x13: { // addi sp, rs, imm
		struct riscv_gi_format *gi = &insn->rvg.gi;
		if (gi->rd == CFI_SP) {
			struct {signed int s_ext:12;} imm = { .s_ext = gi->imm };
			ADD_OP(sop)
			{
				sop->src.type = OP_SRC_ADD;
				sop->src.reg = gi->rs1;
				sop->src.offset = imm.s_ext;
				sop->dest.type = OP_DEST_REG;
				sop->dest.reg = CFI_SP;
			}
			insn->immediate = imm.s_ext;
			debug_insng_printf("addi %s, %s, %i", riscv_reg_abi_names[gi->rd], riscv_reg_abi_names[gi->rs1], imm.s_ext);
		} else if (gi->rd == CFI_ZERO && gi->rs1 == CFI_ZERO && gi->imm == 0) {
			insn->type = INSN_NOP;
			debug_insng_printf("nop");
		}
		break;
	}
	case 0x17: { // auipc
		struct riscv_gu_format *gu = &insn->rvg.gu;
		int64_t s_ext = (gu->imm & BIT(19)) ? ((gu->imm << 12) | (0xffffffff00000000LL)) : gu->imm << 12;

		auipc_info.reloc = find_reloc_by_dest(file->elf, (struct section *)sec, offset);
		auipc_info.exec = true;
		auipc_info.imm = s_ext;
		auipc_info.offset = offset;

		debug_insng_printf("auipc %s, 0x%lx", riscv_reg_abi_names[gu->rd], (int64_t)s_ext);
		return 0;
	}
	case 0x6f:  { // jal imm[20:1]
		struct riscv_gj_format *gj = &insn->rvg.gj;
		struct {signed int s_ext:20;} imm;
		if (gj->rd == CFI_RA) {
			insn->type = INSN_CALL;
		} else if (gj->rd == CFI_ZERO) {
			insn->type = INSN_JUMP_UNCONDITIONAL;
		}
		imm.s_ext = (((gj->imm) << 12) & 0xff000) | (((gj->imm) << 3) & BIT(11)) | (((gj->imm) >> 8) & 0x7fe) | (gj->imm & BIT(20));
		insn->immediate = imm.s_ext;
		debug_insng_printf("%s 0x%lx", insn->type == INSN_CALL ? "jal" : "j", imm.s_ext + next_addr);
		break;
	}
	case 0x67: { // jalr
		struct riscv_gi_format *gi = &insn->rvg.gi;
		struct {signed int s_ext:12;} imm;
		imm.s_ext = gi->imm;
		insn->type = (auipc_info.exec == true) ? INSN_CALL : INSN_CALL_DYNAMIC;
		if (auipc_info.reloc) {
			elf_add_reloc(file->elf, (struct section *)sec, offset, auipc_info.reloc->type, auipc_info.reloc->sym, 0);
		}
		insn->immediate = imm.s_ext + ((auipc_info.exec == true) ? auipc_info.imm - 4 : 0) ;
		debug_insng_printf("jalr %s, 0x%x %s (0x%x -- 0x%x)", riscv_reg_abi_names[gi->rs1], imm.s_ext, (auipc_info.reloc) ? "(EXTERNAL_CALL)" : "", insn->immediate, imm.s_ext);
		break;
	}
	case 0x63: { // beq
		struct riscv_gb_format *gb = &insn->rvg.gb;
		struct {signed int s_ext:12;} imm;
		imm.s_ext = (gb->imm_l & 0x1e) | (gb->imm_h << 5) & 0x7e0 | ((gb->imm_l) << 11) & BIT(11) | ((gb->imm_h) << 6) & BIT(12);
		insn->type = INSN_JUMP_CONDITIONAL;
		insn->immediate = imm.s_ext;
		debug_insng_printf("%s %s, %s, 0x%lx", riscvg_get_cond_mnemonic(gb->func), riscv_reg_abi_names[gb->rs1], riscv_reg_abi_names[gb->rs2], imm.s_ext + next_addr);
		break;
	}

	default:
		break;
	}

	if (saved_sop == NULL && insn->type == INSN_OTHER) {
		debug_insng_printf("----");
	}
	memset(&auipc_info, 0, sizeof(auipc_info));
	return 0;
}

static int insn_decode(struct objtool_file *file, const struct section *sec, int offset, struct riscv_insn *insn, const void *kaddr, int buf_len, struct list_head *ops_list)
{
	get_instruction_length(insn, kaddr);
	insn->type = INSN_OTHER;

	switch (insn->len) {
	case 2:
		riscvc_decode(file, sec, offset, insn, ops_list);
		break;
	case 4:
		memcpy(&insn->rvg.raw, kaddr, 4);
		riscvg_decode(file, sec, offset, insn, ops_list);
		break;
	default:
		debug_printf("Unsupported Instruction length detected %i", insn->len);
		break;

	}
	return 0;
}

static int is_riscv_64(const struct elf *elf)
{
	return elf->ehdr.e_ident[EI_CLASS] == 2;
}

int arch_decode_instruction(struct objtool_file *file, const struct section *sec,
			    unsigned long offset, unsigned int maxlen,
			    unsigned int *len, enum insn_type *type,
			    unsigned long *immediate,
			    struct list_head *ops_list)
{
	struct riscv_insn insn = {0};
	is_64bit = is_riscv_64(file->elf);
	insn_decode(file, sec, offset, &insn, sec->data->d_buf + offset, maxlen, ops_list);
	*len = insn.len;
	*type = insn.type;
	*immediate = insn.immediate;
	next_addr += insn.len;
	return 0;
}

void arch_initial_func_cfi_state(struct cfi_init_state *state)
{
	int i;

	for (i = 0; i < CFI_NUM_REGS; i++) {
		state->regs[i].base = CFI_UNDEFINED;
		state->regs[i].offset = 0;
	}

	/* initial CFA (canonical frame address) */
	state->cfa.base = CFI_SP;
	state->cfa.offset = 0;

	debug_printf("");
}

int arch_cfa_bp_offset(struct cfi_state *cfi)
{
	return (cfi->regs[CFI_BP].base != CFI_UNDEFINED) ? cfi->regs[CFI_BP].offset : INT_MAX;
}

int arch_cfa_rax_offset(struct cfi_state *cfi)
{
	return (cfi->regs[CFI_RA].base != CFI_UNDEFINED) ? cfi->regs[CFI_RA].offset : INT_MAX;
}

const char *arch_nop_insn(int len)
{
	static u16 nop16 = 0x001;
	static u32 nop32 = 0x0000000b;
	printf("-----> BULLSHIT %i", len);
	return len == 2 ? (char *)&nop16 : (char *)&nop32;
}


const char *arch_ret_insn(int len)
{
	printf("----------->BULLSHIT");
	return "ret";
}

int arch_decode_hint_reg(u8 sp_reg, int *base)
{
	printf("------------> BULLSHIT");
	return 0;
}
