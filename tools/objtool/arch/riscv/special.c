// SPDX-License-Identifier: GPL-2.0-or-later
#include <string.h>

#include <objtool/special.h>
#include <objtool/builtin.h>


bool arch_support_alt_relocation(struct special_alt *special_alt,
				 struct instruction *insn,
				 struct reloc *reloc)
{
    return false;
}

/*
 * There are 3 basic jump table patterns:
 *
 * 1. jmpq *[rodata addr](,%reg,8)
 *
 *    This is the most common case by far.  It jumps to an address in a simple
 *    jump table which is stored in .rodata.
 *
 * 2. jmpq *[rodata addr](%rip)
 *
 *    This is caused by a rare GCC quirk, currently only seen in three driver
 *    functions in the kernel, only with certain obscure non-distro configs.
 *
 *    As part of an optimization, GCC makes a copy of an existing switch jump
 *    table, modifies it, and then hard-codes the jump (albeit with an indirect
 *    jump) to use a single entry in the table.  The rest of the jump table and
 *    some of its jump targets remain as dead code.
 *
 *    In such a case we can just crudely ignore all unreachable instruction
 *    warnings for the entire object file.  Ideally we would just ignore them
 *    for the function, but that would require redesigning the code quite a
 *    bit.  And honestly that's just not worth doing: unreachable instruction
 *    warnings are of questionable value anyway, and this is such a rare issue.
 *
 * 3. mov [rodata addr],%reg1
 *    ... some instructions ...
 *    jmpq *(%reg1,%reg2,8)
 *
 *    This is a fairly uncommon pattern which is new for GCC 6.  As of this
 *    writing, there are 11 occurrences of it in the allmodconfig kernel.
 *
 *    As of GCC 7 there are quite a few more of these and the 'in between' code
 *    is significant. Esp. with KASAN enabled some of the code between the mov
 *    and jmpq uses .rodata itself, which can confuse things.
 *
 *    TODO: Once we have DWARF CFI and smarter instruction decoding logic,
 *    ensure the same register is used in the mov and jump instructions.
 *
 *    NOTE: RETPOLINE made it harder still to decode dynamic jumps.
 */
struct reloc *arch_find_switch_table(struct objtool_file *file,
				struct instruction *insn)
{
	struct reloc  *text_reloc, *rodata_reloc;
	struct instruction *dest_insn;
	/* look for a relocation which references .rodata */
	text_reloc = find_reloc_by_dest_range(file->elf, insn->sec,
					      insn->offset, insn->len);
	if (text_reloc && text_reloc->sym->sec->rodata){
		rodata_reloc = find_reloc_by_dest_range(file->elf, text_reloc->sym->sec, text_reloc->sym->offset, 1);
		if (rodata_reloc && rodata_reloc->sym->sec->text) {
			/* Why is x86 using the addend as the offset? */
			rodata_reloc->addend = rodata_reloc->sym->offset;
			return rodata_reloc;
		}
	}
	return NULL;
}
