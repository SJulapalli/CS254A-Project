#ifndef _COMMON_H_
#define _COMMON_H_

#include <stdint.h>

typedef struct {
    uint64_t vals_addr;   // vals[i] = i
    uint64_t keys_addr;   // keys[t] = value thread t breaks on
    uint64_t out_addr;    // out[t]  = how many loop iterations thread t ran
    uint32_t num_threads;
    uint32_t n;           // loop bound
} kernel_arg_t;

#endif
