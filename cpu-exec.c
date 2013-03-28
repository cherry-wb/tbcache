

#include <stdio.h>
#include <stdint.h>
#include "main.h"
#include "trans_all.h"
#include "cpu.h"




/*----------------------------------------------------------------*/
/*					Translation Block Lookup Functions						*/	
/*----------------------------------------------------------------*/

static TranslationBlock *tb_find_slow(CPUArchState *env,
                                      target_ulong pc,
                                      target_ulong cs_base,
                                      uint64_t flags)
{
    TranslationBlock *tb, **ptb1;
    unsigned int h;
    tb_page_addr_t phys_pc, phys_page1;
    target_ulong virt_page2;

    tcg_ctx.tb_ctx.tb_invalidated_flag = 0;

    /* find translated block using physical mappings */
    phys_pc = pc; //get_page_addr_code(env, pc);
    phys_page1 = phys_pc & TARGET_PAGE_MASK;
    h = tb_phys_hash_func(phys_pc);
    ptb1 = &tcg_ctx.tb_ctx.tb_phys_hash[h];
//    for(;;) {
        tb = *ptb1;
        if (!tb)
            goto not_found;
        if (tb->pc == pc &&
            tb->page_addr[0] == phys_page1 &&
            tb->cs_base == cs_base &&
            tb->flags == flags) {
            /* check next page if needed */
            if (tb->page_addr[1] != -1) {
                tb_page_addr_t phys_page2;

                virt_page2 = (pc & TARGET_PAGE_MASK) +
                    TARGET_PAGE_SIZE;
                phys_page2 = pc; //get_page_addr_code(env, virt_page2);
                if (tb->page_addr[1] == phys_page2)
                    goto found;
            } else {
                goto found;
            }
        }
        ptb1 = &tb->phys_hash_next;
//    }
 not_found:
   /* if no translated code available, then translate it now */

    pc = tb_gen_code(env, pc, cs_base, flags, 0);			//really just tb


 found:
    /* Move the last found TB to the head of the list */
/*    if (likely(*ptb1)) {												//first worked then crashed!!
        *ptb1 = tb->phys_hash_next;
        tb->phys_hash_next = tcg_ctx.tb_ctx.tb_phys_hash[h];
        tcg_ctx.tb_ctx.tb_phys_hash[h] = tb;
    }*/
    /* we add the TB in the virtual pc hash table */
    env->tb_jmp_cache[tb_jmp_cache_hash_func(pc)] = tb;
    return pc;
}

/*----------------------------------------------------------------*/

/*static*/ inline TranslationBlock *tb_find_fast(CPUArchState *env)
{
    TranslationBlock *tb;
    target_ulong cs_base;		 pc;
    int flags;

    /* we record a subset of the CPU state. It will
       always be the same before a given translated block
       is executed. */
//    cpu_get_tb_cpu_state(env, &pc, &cs_base, &flags);
    tb = env->tb_jmp_cache[tb_jmp_cache_hash_func(pc)];
//    if (unlikely(!tb || tb->pc != pc || tb->cs_base != cs_base ||
//                 tb->flags != flags)) {
        pc = tb_find_slow(env, pc, cs_base, flags);
//    }
    return pc;
}

/*----------------------------------------------------------------*/


extern tcg_target_ulong next_tb;

/* Execute a TB, and fix up the CPU state afterwards if necessary */
/*static*/  inline tcg_target_ulong cpu_tb_exec(CPUState *cpu, uint8_t *tb_ptr)
{
    CPUArchState *env = cpu->env_ptr;
//    tcg_target_ulong next_tb = tcg_qemu_tb_exec(env, tb_ptr);
    if ((next_tb & TB_EXIT_MASK) > TB_EXIT_IDX1) {
        /* We didn't start executing this TB (eg because the instruction
         * counter hit zero); we must restore the guest PC to the address
         * of the start of the TB.
         */
        TranslationBlock *tb = (TranslationBlock *)(next_tb & ~TB_EXIT_MASK);
//        cpu_pc_from_tb(env, tb);
    }
    if ((next_tb & TB_EXIT_MASK) == TB_EXIT_REQUESTED) {
        /* We were asked to stop executing TBs (probably a pending
         * interrupt. We've now stopped, so clear the flag.
         */
//        cpu->tcg_exit_req = 0;
    }
    return next_tb;
}