

#include <stdio.h>
#include <stdint.h>
#include "main.h"
#include "trans_all.h"
#include "cpu.h"




/*----------------------------------------------------------------*/
/*					Translation Block Lookup Functions						*/	
/*----------------------------------------------------------------*/

// To Be Defined
/*static TranslationBlock *tb_find_slow(CPUArchState *env,
                                      target_ulong pc,
                                      target_ulong cs_base,
                                      uint64_t flags);*/
																				

/*----------------------------------------------------------------*/



/*static*/ inline TranslationBlock *tb_find_fast(CPUArchState *env) 	
{
    TranslationBlock *tb;     

//    int flags;
extern    target_ulong cs_base, pc;           //originally not 'extern' !
    /* we record a subset of the CPU state. It will
       always be the same before a given translated block
       is executed. */
//   cpu_get_tb_cpu_state(env, &pc, &cs_base, &flags);    
	    tb = env->tb_jmp_cache[tb_jmp_cache_hash_func(pc)];
//	    if (unlikely(!tb || tb->pc != pc 			//|| tb->cs_base != cs_base ||  tb->flags != flags
//	    			) ) 
//    {
//			fprintf(stderr,"Warning: calling tb_find_slow!\n");                 	
//    	   tb = tb_find_slow(env, pc, cs_base, flags);
//    }
//    else {fprintf(stderr,"calling tb_find_fast!\n");}
    
    return tb;
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