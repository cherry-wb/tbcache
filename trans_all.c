   /***************************************************************
	*	Optimization of Qemu translation cache policy					*
	*																					*
	*	FERJANI SABER - Tima Lab - SLS Team									*
	*																					*
	*	March 2013																	*
	*																					*
	****************************************************************/


/*----------------------------------------------------------------*/



#include <stdio.h>
#include <stdlib.h>								// for using free() ( instead of g_free() )
#include <stdint.h>
//#include <glib.h>                    	// missing Package !
#define g_free free
#define g_malloc malloc
//#define g_malloc0 malloc						// but need to add init

//#include <sys/mman.h>

#include "trans_all.h"
#include "main.h"


/* code generation context */
//extern TCGContext tcg_ctx;

/* Translation Buffer context */
//extern TBContext tb_ctx;




/*----------------------------------------------------------------*/
/*					Translation Buffer Flush Functions 						*/
/*----------------------------------------------------------------*/

static inline void invalidate_page_bitmap(PageDesc *p)
{
    if (p->code_bitmap) {
        g_free(p->code_bitmap);          	
        p->code_bitmap = NULL;
    }
    p->code_write_count = 0;
}

/*----------------------------------------------------------------*/

/* Set to NULL all the 'first_tb' fields in all PageDescs. */
static void page_flush_tb_1(int level, void **lp)
{
    int i;

    if (*lp == NULL) {
//    	fprintf(stderr,"tb already free! \n");
        return;
    }
    if (level == 0) {
        PageDesc *pd = *lp;

        for (i = 0; i < L2_SIZE; ++i) {
            pd[i].first_tb = NULL;
            invalidate_page_bitmap(pd + i);
        }
        fprintf(stderr, "L0 invalidate_page_bitmap at %u\n",pd);
    } else {
        void **pp = *lp;

        for (i = 0; i < L2_SIZE; ++i) {
            page_flush_tb_1(level - 1, pp + i);  // recursive call to inferior level
        }
    }
}

/*----------------------------------------------------------------*/

static void page_flush_tb(void)
{
    int i;

    for (i = 0; i < V_L1_SIZE; i++) {										//V_L1_SIZE = 32
        page_flush_tb_1(V_L1_SHIFT / L2_BITS - 1, l1_map + i);		//level=30/10 -1 = 2 (always); walk l1_map[32]
    }
    fprintf(stderr,"%d page_flush_tb! \n",V_L1_SIZE);
}

/*----------------------------------------------------------------*/

/* flush all the translation blocks */
/* XXX: tb_flush is currently not thread safe */
void tb_flush(CPUArchState *env1)
{
    CPUArchState *env;

//#if defined(DEBUG_FLUSH)
    printf("qemu: flush code_size=%ld nb_tbs=%d avg_tb_size=%ld\n",
           (unsigned long)(tcg_ctx.code_gen_ptr - tcg_ctx.code_gen_buffer),
           tcg_ctx.tb_ctx.nb_tbs, tcg_ctx.tb_ctx.nb_tbs > 0 ?
           ((unsigned long)(tcg_ctx.code_gen_ptr - tcg_ctx.code_gen_buffer)) /
           tcg_ctx.tb_ctx.nb_tbs : 0);
//#endif
    if ((unsigned long)(tcg_ctx.code_gen_ptr - tcg_ctx.code_gen_buffer)
        > tcg_ctx.code_gen_buffer_size) {
//        cpu_abort(env1, "Internal error: code buffer overflow\n");
        fprintf(stderr,"Warning: (Internal error: code buffer overflow) calling cpu abort in tb_flush...\n");
    }
    tcg_ctx.tb_ctx.nb_tbs = 0;

//    for (env = first_cpu; env != NULL; env = env->next_cpu) {
        memset(env->tb_jmp_cache, 0, TB_JMP_CACHE_SIZE * sizeof(void *));
//    }

    memset(tcg_ctx.tb_ctx.tb_phys_hash, 0,
            CODE_GEN_PHYS_HASH_SIZE * sizeof(void *));
    page_flush_tb();

    tcg_ctx.code_gen_ptr = tcg_ctx.code_gen_buffer;
    /* XXX: flush processor icache at this point if cache flush is
       expensive */
    tcg_ctx.tb_ctx.tb_flush_count++;
    fprintf(stderr,"tb flush:%d\n",tcg_ctx.tb_ctx.tb_flush_count);
    getchar();
}


 
/*----------------------------------------------------------------*/     
/*			Translation Buffer Allocation & Block Generation			*/   
/*----------------------------------------------------------------*/    
    
/* Allocate a new translation block. Flush the translation buffer if
   too many translation blocks or too much generated code. */
static TranslationBlock *tb_alloc(target_ulong pc)
{
    TranslationBlock *tb;

    if (tcg_ctx.tb_ctx.nb_tbs >= tcg_ctx.code_gen_max_blocks ||
        (tcg_ctx.code_gen_ptr - tcg_ctx.code_gen_buffer) >=
         tcg_ctx.code_gen_buffer_max_size) {
        return NULL;
    }
    tb = &tcg_ctx.tb_ctx.tbs[tcg_ctx.tb_ctx.nb_tbs++];
    tb->pc = pc;
    tb->cflags = 0;
    return tb;
}


/*----------------------------------------------------------------*/

/* reset the jump entry 'n' of a TB so that it is not chained to
   another TB */
static inline void tb_reset_jump(TranslationBlock *tb, int n)
{
    tb_set_jmp_target(tb, n, (uintptr_t)(tb->tc_ptr + tb->tb_next_offset[n]));
}

/*----------------------------------------------------------------*/

static PageDesc *page_find_alloc(tb_page_addr_t index, int alloc)
{
//	fprintf(stderr, "call to page_find_alloc\n");
    PageDesc *pd;
    void **lp;
    int i;

//#if defined(CONFIG_USER_ONLY)
    /* We can't use g_malloc because it may recurse into a locked mutex. */
# define ALLOC(P, SIZE)                                 \
    do {                                                \
        P = mmap(NULL, SIZE, PROT_READ | PROT_WRITE,    \
                 MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);   \
    } while (0)
//#else
//# define ALLOC(P, SIZE) \
//    do { P = g_malloc0(SIZE); } while (0)
//#endif

    /* Level 1.  Always allocated.  */
    lp = l1_map + ((index >> V_L1_SHIFT) & (V_L1_SIZE - 1));

    /* Level 2..N-1.  */
    for (i = V_L1_SHIFT / L2_BITS - 1; i > 0; i--) {
        void **p = *lp;

        if (p == NULL) {
            if (!alloc) {
                return NULL;
            }
            ALLOC(p, sizeof(void *) * L2_SIZE);
            *lp = p;
        }

        lp = p + ((index >> (i * L2_BITS)) & (L2_SIZE - 1));
    }

    pd = *lp;
    if (pd == NULL) {
        if (!alloc) {
            return NULL;
        }
        ALLOC(pd, sizeof(PageDesc) * L2_SIZE);
        *lp = pd;
    }

#undef ALLOC

    return pd + (index & (L2_SIZE - 1));
}

/*----------------------------------------------------------------*/

static inline PageDesc *page_find(tb_page_addr_t index)
{
    return page_find_alloc(index, 0);
}

static inline void tb_page_remove(TranslationBlock **ptb, TranslationBlock *tb)
{
    TranslationBlock *tb1;
    unsigned int n1;

    for (;;) {
        tb1 = *ptb;
        n1 = (uintptr_t)tb1 & 3;
        tb1 = (TranslationBlock *)((uintptr_t)tb1 & ~3);
        if (tb1 == tb) {
            *ptb = tb1->page_next[n1];
            break;
        }
        ptb = &tb1->page_next[n1];
    }
}

static inline void tb_jmp_remove(TranslationBlock *tb, int n)
{
    TranslationBlock *tb1, **ptb;
    unsigned int n1;

    ptb = &tb->jmp_next[n];
    tb1 = *ptb;
    if (tb1) {
        /* find tb(n) in circular list */
        for (;;) {
            tb1 = *ptb;
            n1 = (uintptr_t)tb1 & 3;
            tb1 = (TranslationBlock *)((uintptr_t)tb1 & ~3);
            if (n1 == n && tb1 == tb) {
                break;
            }
            if (n1 == 2) {
                ptb = &tb1->jmp_first;
            } else {
                ptb = &tb1->jmp_next[n1];
            }
        }
        /* now we can suppress tb(n) from the list */
        *ptb = tb->jmp_next[n];

        tb->jmp_next[n] = NULL;
    }
}

/* add the tb in the target page and protect it if necessary */
static inline void tb_alloc_page(TranslationBlock *tb,
                                 unsigned int n, tb_page_addr_t page_addr)
{
    PageDesc *p;
//#ifndef CONFIG_USER_ONLY
//    bool page_already_protected;
//#endif

    tb->page_addr[n] = page_addr;
    p = page_find_alloc(page_addr >> TARGET_PAGE_BITS, 1);
    tb->page_next[n] = p->first_tb;
//#ifndef CONFIG_USER_ONLY
//    page_already_protected = p->first_tb != NULL;
//#endif
    p->first_tb = (TranslationBlock *)((uintptr_t)tb | n);
    invalidate_page_bitmap(p);

//#if defined(TARGET_HAS_SMC) || 1

#if defined(CONFIG_USER_ONLY)
    if (p->flags & PAGE_WRITE) {
        target_ulong addr;
        PageDesc *p2;
        int prot;

        /* force the host page as non writable (writes will have a
           page fault + mprotect overhead) */
/*        page_addr &= qemu_host_page_mask;
        prot = 0;
        for (addr = page_addr; addr < page_addr + qemu_host_page_size;
            addr += TARGET_PAGE_SIZE) {

            p2 = page_find(addr >> TARGET_PAGE_BITS);
            if (!p2) {
                continue;
            }
            prot |= p2->flags;
            p2->flags &= ~PAGE_WRITE;
          }
        mprotect(g2h(page_addr), qemu_host_page_size,
                 (prot & PAGE_BITS) & ~PAGE_WRITE);*/
#ifdef DEBUG_TB_INVALIDATE
        printf("protecting code page: 0x" TARGET_FMT_lx "\n",
               page_addr);
#endif
    }
//#else
    /* if some code is already present, then the pages are already
       protected. So we handle the case where only the first TB is
       allocated in a physical page */
    if (!page_already_protected) {
        tlb_protect_code(page_addr);
    }
#endif

//#endif /* TARGET_HAS_SMC */
}

/*----------------------------------------------------------------*/

/* add a new TB and link it to the physical page tables. phys_page2 is
   (-1) to indicate that only one page contains the TB. */
static void tb_link_page(TranslationBlock *tb, tb_page_addr_t phys_pc,
                         tb_page_addr_t phys_page2)
{
    unsigned int h;
    TranslationBlock **ptb;

    /* Grab the mmap lock to stop another thread invalidating this TB
       before we are done.  */
//    mmap_lock();
    /* add in the physical hash table */
    h = tb_phys_hash_func(phys_pc);
    ptb = &tcg_ctx.tb_ctx.tb_phys_hash[h];
    tb->phys_hash_next = *ptb;
    *ptb = tb;

    /* add in the page list */
    tb_alloc_page(tb, 0, phys_pc & TARGET_PAGE_MASK);
    if (phys_page2 != -1) {
        tb_alloc_page(tb, 1, phys_page2);
    } else {
        tb->page_addr[1] = -1;
    }

    tb->jmp_first = (TranslationBlock *)((uintptr_t)tb | 2);
    tb->jmp_next[0] = NULL;
    tb->jmp_next[1] = NULL;

    /* init original jump addresses */
    if (tb->tb_next_offset[0] != 0xffff) {
        tb_reset_jump(tb, 0);
    }
    if (tb->tb_next_offset[1] != 0xffff) {
        tb_reset_jump(tb, 1);
    }

//#ifdef DEBUG_TB_CHECK
//    tb_page_check();
//#endif
//    mmap_unlock();
}
/*----------------------------------------------------------------*/

/* return non zero if the very first instruction is invalid so that
   the virtual CPU can trigger an exception.

   '*gen_code_size_ptr' contains the size of the generated code (host
   code).
*/
int cpu_gen_code(CPUArchState *env, TranslationBlock *tb, int *gen_code_size_ptr)
{
    TCGContext *s = &tcg_ctx;
    uint8_t *gen_code_buf;
    int gen_code_size;
	 unsigned long pc = tb->pc;
    extern uint16_t src[MAX_PC][2];
    
    unsigned long old_pc;
#ifdef CONFIG_PROFILER
    int64_t ti;
#endif

#ifdef CONFIG_PROFILER
    s->tb_count1++; /* includes aborted translations because of exceptions */
    ti = profile_getclock();
#endif
//   tcg_func_start(s);

//   gen_intermediate_code(env, tb);
//	  fprintf(stderr,"pc = %d\n", pc);

/*------------------ added part ------------------*/
	 
	 old_pc = pc;	
    for(;;)
    {
    	//fprintf(stderr,"pc=%u\n",pc);
    	pc++;
      if ((src[pc][0]=='j') || (src[pc][0]==0)) break;    	
    }
    	
/*    if (src[pc][0]==0) 															// EOF should never be reached!
    	{
    		fprintf(stderr,"EOF at pc = %u \n",pc);
    		fprintf(stderr," code_size=%ld nb_tbs=%d avg_tb_size=%ld\n",
    		           	(unsigned long)(tcg_ctx.code_gen_ptr - tcg_ctx.code_gen_buffer),
           				tcg_ctx.tb_ctx.nb_tbs, tcg_ctx.tb_ctx.nb_tbs > 0 ?
           				((unsigned long)(tcg_ctx.code_gen_ptr - tcg_ctx.code_gen_buffer)) /
           				tcg_ctx.tb_ctx.nb_tbs : 0);
    		pc = 0;
    		getchar();
    		}
		else	 */

	 gen_code_size = pc - old_pc;		
	 
	 pc=src[pc][1];
	 
	 tb->pc=pc;
	 
    //getchar();
    
    fprintf(stderr,"nb tbs=%4d -- tb size=%4u -- pc = %4u -- tb = %u\n",tcg_ctx.tb_ctx.nb_tbs,gen_code_size,pc,&tb);	 

//	 fprintf(stderr,"pc = %u tb size = %u\n",pc, gen_code_size);
/* ------------------ end of added part ------------------ */


    /* generate machine code */
    gen_code_buf = tb->tc_ptr;
    tb->tb_next_offset[0] = 0xffff;
    tb->tb_next_offset[1] = 0xffff;
    s->tb_next_offset = tb->tb_next_offset;
#ifdef USE_DIRECT_JUMP
    s->tb_jmp_offset = tb->tb_jmp_offset;
    s->tb_next = NULL;
#else
    s->tb_jmp_offset = NULL;
    s->tb_next = tb->tb_next;
#endif

#ifdef CONFIG_PROFILER
    s->tb_count++;
    s->interm_time += profile_getclock() - ti;
    s->code_time -= profile_getclock();
#endif
//    gen_code_size = tcg_gen_code(s, gen_code_buf);
    *gen_code_size_ptr = gen_code_size;
#ifdef CONFIG_PROFILER
    s->code_time += profile_getclock();
    s->code_in_len += tb->size;
    s->code_out_len += gen_code_size;
#endif

#ifdef DEBUG_DISAS
    if (qemu_loglevel_mask(CPU_LOG_TB_OUT_ASM)) {
        qemu_log("OUT: [size=%d]\n", *gen_code_size_ptr);
        log_disas(tb->tc_ptr, *gen_code_size_ptr);
        qemu_log("\n");
        qemu_log_flush();
    }
#endif
//	fprintf(stderr,"OUT: [size=%d]\n", *gen_code_size_ptr);	
    return 0;
}

/*----------------------------------------------------------------*/ 

static inline void tb_hash_remove(TranslationBlock **ptb, TranslationBlock *tb)
{
    TranslationBlock *tb1;

    for (;;) {
        tb1 = *ptb;
        if (tb1 == tb) {
            *ptb = tb1->phys_hash_next;
            break;
        }
        ptb = &tb1->phys_hash_next;
    }
}

/* invalidate one TB */
void tb_phys_invalidate(TranslationBlock *tb, tb_page_addr_t page_addr)
{
    CPUArchState *env;
    PageDesc *p;
    unsigned int h, n1;
    tb_page_addr_t phys_pc;
    TranslationBlock *tb1, *tb2;

    /* remove the TB from the hash list */
    phys_pc = tb->page_addr[0] + (tb->pc & ~TARGET_PAGE_MASK);
    h = tb_phys_hash_func(phys_pc);
//    tb_hash_remove(&tcg_ctx.tb_ctx.tb_phys_hash[h], tb);

    /* remove the TB from the page list */
    if (tb->page_addr[0] != page_addr) {
        p = page_find(tb->page_addr[0] >> TARGET_PAGE_BITS);
        tb_page_remove(&p->first_tb, tb);
        invalidate_page_bitmap(p);
    }
    if (tb->page_addr[1] != -1 && tb->page_addr[1] != page_addr) {
        p = page_find(tb->page_addr[1] >> TARGET_PAGE_BITS);
        tb_page_remove(&p->first_tb, tb);
        invalidate_page_bitmap(p);
    }

    tcg_ctx.tb_ctx.tb_invalidated_flag = 1;

    /* remove the TB from the hash list */
    h = tb_jmp_cache_hash_func(tb->pc);
//    for (env = first_cpu; env != NULL; env = env->next_cpu) {
        if (env->tb_jmp_cache[h] == tb) {
            env->tb_jmp_cache[h] = NULL;
        }
//    }

    /* suppress this TB from the two jump lists */
    tb_jmp_remove(tb, 0);
    tb_jmp_remove(tb, 1);

    /* suppress any remaining jumps to this TB */
    tb1 = tb->jmp_first;
    for (;;) {
        n1 = (uintptr_t)tb1 & 3;
        if (n1 == 2) {
            break;
        }
        tb1 = (TranslationBlock *)((uintptr_t)tb1 & ~3);
        tb2 = tb1->jmp_next[n1];
        tb_reset_jump(tb1, n1);
        tb1->jmp_next[n1] = NULL;
        tb1 = tb2;
    }
    tb->jmp_first = (TranslationBlock *)((uintptr_t)tb | 2); /* fail safe */

    tcg_ctx.tb_ctx.tb_phys_invalidate_count++;
}



// this function try to 
static void tb_switch(TranslationBlock *tb)
{
	//	env->tb_seg = (!tb_seg);
	fprintf(stderr,"switch occured\n");

	//tb_page_addr_t page_addr;
	//invalidate fews pages..
	
	tb_phys_invalidate(&tb, -1);
}
/*----------------------------------------------------------------*/ 
    
    TranslationBlock *tb_gen_code(CPUArchState *env,
                              target_ulong pc, target_ulong cs_base,
                              int flags, int cflags)
{
    TranslationBlock *tb;
    uint8_t *tc_ptr;
    tb_page_addr_t phys_pc, phys_page2;
    target_ulong virt_page2;
    int code_gen_size;

//fprintf(stderr, "before alloc, tb->pc= %u..\n", tb->pc);
//    phys_pc = get_page_addr_code(env, pc);

	 if ((tcg_ctx.tb_ctx.nb_tbs == (tcg_ctx.code_gen_max_blocks >> 1)) || (tcg_ctx.tb_ctx.nb_tbs >= tcg_ctx.code_gen_max_blocks)) tb_switch(&tb);

    tb = tb_alloc(pc);
    if (!tb) {
		  fprintf(stderr,"Error: no more available space! tb_flush call\n");
		  getchar();
		  tb_flush(env);
        /* cannot fail at this point */
        tb = tb_alloc(pc);
        /* Don't forget to invalidate previous TB info.  */
        tcg_ctx.tb_ctx.tb_invalidated_flag = 1;
    }
//    fprintf(stderr, "TB= %u tb->pc = %u   # nb tb = %u \n", tb,tb->pc, tcg_ctx.tb_ctx.nb_tbs);
    tc_ptr = tcg_ctx.code_gen_ptr;
    tb->tc_ptr = tc_ptr;
    tb->cs_base = cs_base;
    tb->flags = flags;
    tb->cflags = cflags;
    cpu_gen_code(env, tb, &code_gen_size);

    tcg_ctx.code_gen_ptr = (void *)(((uintptr_t)tcg_ctx.code_gen_ptr +
            code_gen_size + CODE_GEN_ALIGN - 1) & ~(CODE_GEN_ALIGN - 1));

    /* check next page if needed */
    virt_page2 = (pc + tb->size - 1) & TARGET_PAGE_MASK;
    phys_page2 = -1;
    if ((pc & TARGET_PAGE_MASK) != virt_page2) {
//        phys_page2 = get_page_addr_code(env, virt_page2);
    }
    tb_link_page(tb, phys_pc, phys_page2);
//    fprintf(stderr,"(gencode) tb -> pc = %u \n",tb->pc);    
    return tb->pc;															// really just tb
}



/*----------------------------------------------------------------*/

/* Minimum size of the code gen buffer.  This number is randomly chosen,
   but not so small that we can't have a fair number of TB's live.  */
#define MIN_CODE_GEN_BUFFER_SIZE     (1024u * 1024)

/* Maximum size of the code gen buffer we'd like to use.  Unless otherwise
   indicated, this is constrained by the range of direct branches on the
   host cpu, as used by the TCG implementation of goto_tb.  */

# define MAX_CODE_GEN_BUFFER_SIZE  (2ul * 1024 * 1024 * 1024)

#define DEFAULT_CODE_GEN_BUFFER_SIZE_1 (32u * 1024 * 1024)

#define DEFAULT_CODE_GEN_BUFFER_SIZE \
  (DEFAULT_CODE_GEN_BUFFER_SIZE_1 < MAX_CODE_GEN_BUFFER_SIZE \
   ? DEFAULT_CODE_GEN_BUFFER_SIZE_1 : MAX_CODE_GEN_BUFFER_SIZE)

#define USE_STATIC_CODE_GEN_BUFFER
#define CODE_GEN_ALIGN           16 /* must be >= of the size of a icache line */

#ifdef USE_STATIC_CODE_GEN_BUFFER
static uint8_t static_code_gen_buffer[DEFAULT_CODE_GEN_BUFFER_SIZE]  __attribute__((aligned(CODE_GEN_ALIGN)));


static inline void *alloc_code_gen_buffer(void)
{
    map_exec(static_code_gen_buffer, tcg_ctx.code_gen_buffer_size);
    return static_code_gen_buffer;
}
#endif   
   

/*----------------------------------------------------------------*/

static inline size_t size_code_gen_buffer(size_t tb_size)
{
    /* Size the buffer. */
    if (tb_size == 0) {
//#ifdef USE_STATIC_CODE_GEN_BUFFER
        tb_size = DEFAULT_CODE_GEN_BUFFER_SIZE;
//#else
        /* ??? Needs adjustments.  */
        /* ??? If we relax the requirement that CONFIG_USER_ONLY use the
           static buffer, we could size this on RESERVED_VA, on the text
           segment size of the executable, or continue to use the default.  */
//        tb_size = (unsigned long)(ram_size / 4);
//#endif
    }
    if (tb_size < MIN_CODE_GEN_BUFFER_SIZE) {
        tb_size = MIN_CODE_GEN_BUFFER_SIZE;
    }
    if (tb_size > MAX_CODE_GEN_BUFFER_SIZE) {
        tb_size = MAX_CODE_GEN_BUFFER_SIZE;
    }
    tcg_ctx.code_gen_buffer_size = tb_size;
    return tb_size;
}


/*----------------------------------------------------------------*/
/*static*/ inline void code_gen_alloc(size_t tb_size)
{
    tcg_ctx.code_gen_buffer_size = size_code_gen_buffer(tb_size);
    tcg_ctx.code_gen_buffer = alloc_code_gen_buffer();								//what is 'code_gen_buffer' ** statically allocated! 
    if (tcg_ctx.code_gen_buffer == NULL) {
        fprintf(stderr, "Could not allocate dynamic translator buffer\n");
        exit(1);
    }

//#define QEMU_MADV_HUGEPAGE -1			// #define copied from: include/qemu/osdep.h
//    qemu_madvise(tcg_ctx.code_gen_buffer, tcg_ctx.code_gen_buffer_size,
//            QEMU_MADV_HUGEPAGE);

    /* Steal room for the prologue at the end of the buffer.  This ensures
       (via the MAX_CODE_GEN_BUFFER_SIZE limits above) that direct branches
       from TB's to the prologue are going to be in range.  It also means
       that we don't need to mark (additional) portions of the data segment
       as executable.  */
    tcg_ctx.code_gen_prologue = tcg_ctx.code_gen_buffer +
            tcg_ctx.code_gen_buffer_size - 1024;
    tcg_ctx.code_gen_buffer_size -= 1024;                                  	

    tcg_ctx.code_gen_buffer_max_size = tcg_ctx.code_gen_buffer_size -
        (TCG_MAX_OP_SIZE * OPC_BUF_SIZE);													// what's this ?
    tcg_ctx.code_gen_max_blocks = tcg_ctx.code_gen_buffer_size /
            CODE_GEN_AVG_BLOCK_SIZE;
    tcg_ctx.tb_ctx.tbs =
            g_malloc(tcg_ctx.code_gen_max_blocks * sizeof(TranslationBlock));	// allocated Table for translated blocks		
}



/*----------------------------------------------------------------*/





