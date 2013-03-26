#ifndef _EXEC_ALL_H_
#define _EXEC_ALL_H_

//#include "trans_all.h"

#define TCG_MAX_OP_SIZE 192
#define TARGET_PAGE_BITS 12


/*----------cpu-def----------*/
#define TB_JMP_CACHE_BITS 12
#define TB_JMP_CACHE_SIZE (1 << TB_JMP_CACHE_BITS) //4096


/* Only the bottom TB_JMP_PAGE_BITS of the jump cache hash bits vary for
   addresses on the same page.  The top bits are the same.  This allows
   TLB invalidation to quickly clear a subset of the hash table.  */
#define TB_JMP_PAGE_BITS (TB_JMP_CACHE_BITS / 2)
#define TB_JMP_PAGE_SIZE (1 << TB_JMP_PAGE_BITS)
#define TB_JMP_ADDR_MASK (TB_JMP_PAGE_SIZE - 1)
#define TB_JMP_PAGE_MASK (TB_JMP_CACHE_SIZE - TB_JMP_PAGE_SIZE)

#define TARGET_PHYS_ADDR_SPACE_BITS 52
/* ??? This is really 48 bits, sign-extended, but the only thing
   accessible to userland with bit 48 set is the VSYSCALL, and that
   is handled via other mechanisms.  */
#define TARGET_VIRT_ADDR_SPACE_BITS 47

#define CODE_GEN_PHYS_HASH_BITS     15
#define CODE_GEN_PHYS_HASH_SIZE     (1 << CODE_GEN_PHYS_HASH_BITS)

typedef unsigned long target_ulong;

typedef uint64_t ram_addr_t;

typedef ram_addr_t tb_page_addr_t;

static inline unsigned int tb_jmp_cache_hash_func(target_ulong pc)  
{
    target_ulong tmp;
    tmp = pc ^ (pc >> (TARGET_PAGE_BITS - TB_JMP_PAGE_BITS));
    return (((tmp >> (TARGET_PAGE_BITS - TB_JMP_PAGE_BITS)) & TB_JMP_PAGE_MASK)
	    | (tmp & TB_JMP_ADDR_MASK));
}

static inline unsigned int tb_phys_hash_func(tb_page_addr_t pc)
{
    return (pc >> 2) & (CODE_GEN_PHYS_HASH_SIZE - 1);
}


static inline void tb_set_jmp_target1(uintptr_t jmp_addr, uintptr_t addr)
{
    /* patch the branch destination */
    *(uint32_t *)jmp_addr = addr - (jmp_addr + 4);
    /* no need to flush icache explicitly */
}


#endif /* _EXEC_ALL_H_ */
