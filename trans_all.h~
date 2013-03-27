#ifndef _TRANS_ALL_H_
#define _TRANS_ALL_H_


#include "exec_all.h"

#define likely(x)       __builtin_expect((x),1)
#define unlikely(x)     __builtin_expect((x),0)

#define CPUArchState struct CPUX86State

#define TF_MASK 		0x00000100
#define IF_MASK 		0x00000200
#define DF_MASK 		0x00000400
#define IOPL_MASK		0x00003000
#define NT_MASK	   0x00004000
#define RF_MASK		0x00010000
#define VM_MASK		0x00020000
#define AC_MASK		0x00040000
#define VIF_MASK     0x00080000
#define VIP_MASK     0x00100000
#define ID_MASK      0x00200000

/* page related stuff */

#define TARGET_PAGE_SIZE (1 << TARGET_PAGE_BITS)
#define TARGET_PAGE_MASK ~(TARGET_PAGE_SIZE - 1)
#define TARGET_PAGE_ALIGN(addr) (((addr) + TARGET_PAGE_SIZE - 1) & TARGET_PAGE_MASK)

#include <pthread.h>
#define spin_lock pthread_mutex_lock
#define spin_unlock pthread_mutex_unlock
#define spinlock_t pthread_mutex_t
#define SPIN_LOCK_UNLOCKED PTHREAD_MUTEX_INITIALIZER

#define CPU_NB_REGS 16  //modified as target i386_64


#define CODE_GEN_ALIGN           16 /* must be >= of the size of a icache line */

#define CODE_GEN_PHYS_HASH_BITS     15
#define CODE_GEN_PHYS_HASH_SIZE     (1 << CODE_GEN_PHYS_HASH_BITS)

/* estimated block size for TB allocation */
/* XXX: use a per code average code fragment size and modulate it
   according to the host CPU */
#if defined(CONFIG_SOFTMMU)
#define CODE_GEN_AVG_BLOCK_SIZE 128
#else
#define CODE_GEN_AVG_BLOCK_SIZE 64
#endif

/* A Call op needs up to 4 + 2N parameters on 32-bit archs,
 * and up to 4 + N parameters on 64-bit archs
 * (N = number of input arguments + output arguments).  */
#define MAX_OPC_PARAM (4 + (MAX_OPC_PARAM_PER_ARG * MAX_OPC_PARAM_ARGS))
#define OPC_BUF_SIZE 640
#define OPC_MAX_SIZE (OPC_BUF_SIZE - MAX_OP_PER_INSTR)

/* Maximum size a TCG op can expand to.  This is complicated because a
   single op may require several host instructions and register reloads.
   For now take a wild guess at 192 bytes, which should allow at least
   a couple of fixup instructions per argument.  */
#define TCG_MAX_OP_SIZE 192


struct TranslationBlock;
typedef struct TranslationBlock TranslationBlock;


/* Size of the L2 (and L3, etc) page tables.  */
#define L2_BITS 10
#define L2_SIZE (1 << L2_BITS)

/* In system mode we want L1_MAP to be based on ram offsets,
   while in user mode we want it to be based on virtual addresses. */
//    *** implemantation modified *** 

#define L1_MAP_ADDR_SPACE_BITS  TARGET_VIRT_ADDR_SPACE_BITS  // 47

/* The bits remaining after N lower levels of page tables.  */
#define V_L1_BITS_REM \
    ((L1_MAP_ADDR_SPACE_BITS - TARGET_PAGE_BITS) % L2_BITS)  // ( 47 -12 ) % 10 = 5

//if V_L1_BITS_REM < 4
//#define V_L1_BITS  (V_L1_BITS_REM + L2_BITS)
//#else
#define V_L1_BITS  V_L1_BITS_REM  // 5
//#endif                                      /* --- a verifier --- */

#define V_L1_SIZE  ((target_ulong)1 << V_L1_BITS)				// 32

#define V_L1_SHIFT (L1_MAP_ADDR_SPACE_BITS - TARGET_PAGE_BITS - V_L1_BITS) 	//30

#define P_L2_LEVELS \
    (((TARGET_PHYS_ADDR_SPACE_BITS - TARGET_PAGE_BITS - 1) / L2_BITS) + 1)		//(52-12-1) /10 +1 = 39/10 +1 =4

/* This is a multi-level map on the virtual address space.
   The bottom level has pointers to PageDesc.  */
static void *l1_map[V_L1_SIZE];													//32 ptr



struct TranslationBlock {
    target_ulong pc;   /* simulated PC corresponding to this block (EIP + CS base) */
    target_ulong cs_base; /* CS base for this block */
    uint64_t flags; /* flags defining in which context the code was generated */
    uint16_t size;      /* size of target code for this block (1 <=
                           size <= TARGET_PAGE_SIZE) */
    uint16_t cflags;    /* compile flags */
#define CF_COUNT_MASK  0x7fff
#define CF_LAST_IO     0x8000 /* Last insn may be an IO access.  */

    uint8_t *tc_ptr;    /* pointer to the translated code */
    /* next matching tb for physical address. */
    struct TranslationBlock *phys_hash_next;
    /* first and second physical page containing code. The lower bit
       of the pointer tells the index in page_next[] */
    struct TranslationBlock *page_next[2];
    tb_page_addr_t page_addr[2];

    /* the following data are used to directly call another TB from
       the code of this one. */
    uint16_t tb_next_offset[2]; /* offset of original jump target */
#ifdef USE_DIRECT_JUMP
    uint16_t tb_jmp_offset[2]; /* offset of jump instruction */
#else
    uintptr_t tb_next[2]; /* address of jump generated code */
#endif
    /* list of TBs jumping to this one. This is a circular list using
       the two least significant bits of the pointers to tell what is
       the next pointer: 0 = jmp_next[0], 1 = jmp_next[1], 2 =
       jmp_first */
    struct TranslationBlock *jmp_next[2];
    struct TranslationBlock *jmp_first;
    uint32_t icount;
};

typedef struct PageDesc {
    /* list of TBs intersecting this ram page */
    TranslationBlock *first_tb;
    /* in order to optimize self modifying code, we count the number
       of lookups we do to a given page to use a bitmap */
    unsigned int code_write_count;
    uint8_t *code_bitmap;
#if defined(CONFIG_USER_ONLY)
    unsigned long flags;
#endif
} PageDesc;

typedef struct SegmentCache {
    uint32_t selector;
    target_ulong base;
    uint32_t limit;
    uint32_t flags;
} SegmentCache;

typedef struct CPUX86State {
    /* standard registers */
    target_ulong regs[CPU_NB_REGS];
    target_ulong eip;
    target_ulong eflags; /* eflags register. During CPU emulation, CC
                        flags and DF are set to zero because they are
                        stored elsewhere */

    /* emulator internal eflags handling */
    target_ulong cc_dst;
    target_ulong cc_src;
    target_ulong cc_src2;
    uint32_t cc_op;
    int32_t df; /* D flag : 1 if D = 0, -1 if D = 1 */
    uint32_t hflags; /* TB flags, see HF_xxx constants. These flags
                        are known at translation time. */
    uint32_t hflags2; /* various other flags, see HF2_xxx constants. */

    /* segments */
    SegmentCache segs[6]; /* selector values */
    SegmentCache ldt;
    SegmentCache tr;
    SegmentCache gdt; /* only base and limit are used */
    SegmentCache idt; /* only base and limit are used */

    target_ulong cr[5]; /* NOTE: cr1 is unused */
    int32_t a20_mask;

    /* FPU state */
    unsigned int fpstt; /* top of stack index */
    uint16_t fpus;
    uint16_t fpuc;
    uint8_t fptags[8];   /* 0 = valid, 1 = empty */
//    FPReg fpregs[8];
    /* KVM-only so far */
    uint16_t fpop;
    uint64_t fpip;
    uint64_t fpdp;

    /* emulator internal variables */
//    float_status fp_status;
//    floatx80 ft0;

//    float_status mmx_status; /* for 3DNow! float ops */
//    float_status sse_status;
//    uint32_t mxcsr;
//    XMMReg xmm_regs[CPU_NB_REGS];
//    XMMReg xmm_t0;
//    MMXReg mmx_t0;

    /* sysenter registers */
    uint32_t sysenter_cs;
    target_ulong sysenter_esp;
    target_ulong sysenter_eip;
    uint64_t efer;
    uint64_t star;

    uint64_t vm_hsave;
    uint64_t vm_vmcb;
    uint64_t tsc_offset;
    uint64_t intercept;
    uint16_t intercept_cr_read;
    uint16_t intercept_cr_write;
    uint16_t intercept_dr_read;
    uint16_t intercept_dr_write;
    uint32_t intercept_exceptions;
    uint8_t v_tpr;

#ifdef TARGET_X86_64
    target_ulong lstar;
    target_ulong cstar;
    target_ulong fmask;
    target_ulong kernelgsbase;
#endif
    uint64_t system_time_msr;
    uint64_t wall_clock_msr;
    uint64_t async_pf_en_msr;
    uint64_t pv_eoi_en_msr;

    uint64_t tsc;
    uint64_t tsc_adjust;
    uint64_t tsc_deadline;

    uint64_t mcg_status;
    uint64_t msr_ia32_misc_enable;

    /* exception/interrupt handling */
    int error_code;
    int exception_is_int;
    target_ulong exception_next_eip;
    target_ulong dr[8]; /* debug registers */
//    union {
//        CPUBreakpoint *cpu_breakpoint[4];
//        CPUWatchpoint *cpu_watchpoint[4];
//    }; /* break/watchpoints for dr[0..3] */
    uint32_t smbase;
    int old_exception;  /* exception in flight */

    /* KVM states, automatically cleared on reset */
    uint8_t nmi_injected;
    uint8_t nmi_pending;

//    CPU_COMMON
struct TranslationBlock *tb_jmp_cache[TB_JMP_CACHE_SIZE];  // added from CPU_COMMON -- Size=4096
    uint64_t pat;

    /* processor features (e.g. for CPUID insn) */
    uint32_t cpuid_level;
    uint32_t cpuid_vendor1;
    uint32_t cpuid_vendor2;
    uint32_t cpuid_vendor3;
    uint32_t cpuid_version;
    uint32_t cpuid_features;
    uint32_t cpuid_ext_features;
    uint32_t cpuid_xlevel;
    uint32_t cpuid_model[12];
    uint32_t cpuid_ext2_features;
    uint32_t cpuid_ext3_features;
    uint32_t cpuid_apic_id;
    /* Store the results of Centaur's CPUID instructions */
    uint32_t cpuid_xlevel2;
    uint32_t cpuid_ext4_features;
    /* Flags from CPUID[EAX=7,ECX=0].EBX */
    uint32_t cpuid_7_0_ebx_features;

    /* MTRRs */
    uint64_t mtrr_fixed[11];
    uint64_t mtrr_deftype;
//    MTRRVar mtrr_var[8];

    /* For KVM */
    uint32_t mp_state;
    int32_t exception_injected;
    int32_t interrupt_injected;
    uint8_t soft_interrupt;
    uint8_t has_error_code;
    uint32_t sipi_vector;
    uint32_t cpuid_kvm_features;
    uint32_t cpuid_svm_features;
//    bool tsc_valid;
    int tsc_khz;
    void *kvm_xsave_buf;

    /* in order to simplify APIC support, we leave this pointer to the
       user */
    struct DeviceState *apic_state;

    uint64_t mcg_cap;
    uint64_t mcg_ctl;
//    uint64_t mce_banks[MCE_BANKS_DEF*4];

    uint64_t tsc_aux;

    /* vmstate */
    uint16_t fpus_vmstate;
    uint16_t fptag_vmstate;
    uint16_t fpregs_format_vmstate;

    uint64_t xstate_bv;
//    XMMReg ymmh_regs[CPU_NB_REGS];

    uint64_t xcr0;

//    TPRAccess tpr_access_type;
} CPUX86State;


/*static inline void cpu_get_tb_cpu_state(CPUX86State *env, target_ulong *pc,
                                        target_ulong *cs_base, int *flags)
{
    *cs_base = env->segs[R_CS].base;
    *pc = *cs_base + env->eip;
    *flags = env->hflags |
        (env->eflags & (IOPL_MASK | TF_MASK | RF_MASK | VM_MASK | AC_MASK));
}*/


typedef struct TBContext TBContext;

struct TBContext {

    TranslationBlock *tbs;
    TranslationBlock *tb_phys_hash[CODE_GEN_PHYS_HASH_SIZE];
    int nb_tbs;
    /* any access to the tbs or the page table must use this lock */
    spinlock_t tb_lock;

    /* statistics */
    int tb_flush_count;
    int tb_phys_invalidate_count;

    int tb_invalidated_flag;
};


typedef struct TCGContext TCGContext;

struct TCGContext {
    int nb_labels;
    int nb_globals;
    int nb_temps;
    /* index of free temps, -1 if none */
//  int first_free_temp[TCG_TYPE_COUNT * 2]; //not needed

    /* goto_tb support */
    uint8_t *code_buf;
    uintptr_t *tb_next;
    uint16_t *tb_next_offset;
    uint16_t *tb_jmp_offset; /* != NULL if USE_DIRECT_JUMP */

    /* liveness analysis */
    uint16_t *op_dead_args; /* for each operation, each bit tells if the
                               corresponding argument is dead */
    uint8_t *op_sync_args;  /* for each operation, each bit tells if the
                               corresponding output argument needs to be
                               sync to memory. */

    uint8_t *code_ptr;

//    uint16_t gen_opc_buf[OPC_BUF_SIZE];
//    TCGArg gen_opparam_buf[OPPARAM_BUF_SIZE];

    uint16_t *gen_opc_ptr;

/*    target_ulong gen_opc_pc[OPC_BUF_SIZE];
    uint16_t gen_opc_icount[OPC_BUF_SIZE];
    uint8_t gen_opc_instr_start[OPC_BUF_SIZE];*/

    /* Code generation */
    int code_gen_max_blocks;
    uint8_t *code_gen_prologue;
    uint8_t *code_gen_buffer;
    size_t code_gen_buffer_size;
    /* threshold to flush the translated code buffer */
    size_t code_gen_buffer_max_size;
    uint8_t *code_gen_ptr;

    TBContext tb_ctx;
};

/* set the jump target */
static inline void tb_set_jmp_target(TranslationBlock *tb,
                                     int n, uintptr_t addr)
{
    tb->tb_next[n] = addr;
}

/*-- added from include/exec/exec-all.h --*/	
static inline void tb_add_jump(TranslationBlock *tb, int n,
                               TranslationBlock *tb_next)
{
    /* NOTE: this test is only needed for thread safety */
    if (!tb->jmp_next[n]) {
        /* patch the native jump address */
        tb_set_jmp_target(tb, n, (uintptr_t)tb_next->tc_ptr);

        /* add in TB jmp circular list */
        tb->jmp_next[n] = tb_next->jmp_first;
        tb_next->jmp_first = (TranslationBlock *)((uintptr_t)(tb) | (n));
    }
}


typedef uint64_t tcg_target_ulong;

/*---------------------------------------------------------------------*/










#endif /* _TRANS_ALL_H_ */