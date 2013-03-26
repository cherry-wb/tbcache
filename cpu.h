
typedef unsigned long tcg_target_ulong;


/* Restriction from include/qom/cpu.h */


typedef struct CPUState CPUState;

/**
 * CPUState:
 * @cpu_index: CPU index (informative).
 * @nr_cores: Number of cores within this CPU package.
 * @nr_threads: Number of threads within this CPU.
 * @numa_node: NUMA node this CPU is belonging to.
 * @host_tid: Host thread ID.
 * @running: #true if CPU is currently running (usermode).
 * @created: Indicates whether the CPU thread has been successfully created.
 * @stop: Indicates a pending stop request.
 * @stopped: Indicates the CPU has been artificially stopped.
 * @tcg_exit_req: Set to force TCG to stop executing linked TBs for this
 *           CPU and return to its top level loop.
 * @env_ptr: Pointer to subclass-specific CPUArchState field.
 * @current_tb: Currently executing TB.
 * @kvm_fd: vCPU file descriptor for KVM.
 *
 * State of one CPU core or thread.
 */
struct CPUState {
    /*< private >*/
//    DeviceState parent_obj;
    /*< public >*/

//    int nr_cores;
//    int nr_threads;
//    int numa_node;

//    struct QemuThread *thread;
//#ifdef _WIN32
//    HANDLE hThread;
//#endif
//    int thread_id;
//    uint32_t host_tid;
//    bool running;
//   struct QemuCond *halt_cond;
//    struct qemu_work_item *queued_work_first, *queued_work_last;
//    bool thread_kicked;
//    bool created;
//    bool stop;
//    bool stopped;
//    volatile sig_atomic_t exit_request;
//    volatile sig_atomic_t tcg_exit_req;

    void *env_ptr; /* CPUArchState */
    struct TranslationBlock *current_tb;

//    int kvm_fd;
//    bool kvm_vcpu_dirty;
//    struct KVMState *kvm_state;
//    struct kvm_run *kvm_run;

    /* TODO Move common fields from CPUArchState here. */
//    int cpu_index; /* used by alpha TCG */
};
