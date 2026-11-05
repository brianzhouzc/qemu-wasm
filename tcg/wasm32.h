#ifndef TCG_WASM32_H
#define TCG_WASM32_H

struct wasmContext {
    // 0
    CPUArchState *env;
    // 4
    uint64_t *stack;
    // 8
    uint32_t *tb_ptr;
    // 12
    uint32_t *tci_tb_ptr;
    // 16
    uint32_t do_init;
    // 20
    uint32_t done_flag;
    // 24
    uint64_t *stack128;
    // 28
    uint32_t unwinding;
};

#define ENV_OFF 0
#define STACK_OFF 4
#define TB_PTR_OFF 8
#define HELPER_RET_TB_PTR_OFF 12
#define DO_INIT_OFF 16
#define DONE_FLAG_OFF 20
#define STACK128_OFF 24
#define UNWINDING_OFF 28

void set_done_flag();

void set_unwinding_flag();

int get_core_nums();

void init_wasm32();

/*
 * A block is interpreted until it has run this many times, then translated
 * into a Wasm module. ktock's default is 1500; the machine's wasm-instantiate
 * property overrides it at run time so a boot can be measured at another
 * threshold.
 */
#define INSTANTIATE_NUM 1500

/* How many translation blocks have been compiled into Wasm modules so far. */
extern unsigned long wasm_tb_compiled_count;

/*
 * Override the run count at which a block is compiled (machine properties
 * wasm-instantiate and wasm-jit).  INT32_MAX keeps every block in TCI.
 */
void wasm_set_instantiate_num(int n);

#endif
