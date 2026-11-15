#include <vx_spawn2.h>
#include "common.h"

// Each thread counts up to n, but breaks early if it sees its key.
// Even threads have key 3 (they break). Odd threads have key 100 (never reached).
__kernel void kernel_main(kernel_arg_t* __UNIFORM__ arg) {
    auto vals = reinterpret_cast<const uint32_t*>(arg->vals_addr);
    auto keys = reinterpret_cast<const uint32_t*>(arg->keys_addr);
    auto out  = reinterpret_cast<uint32_t*>(arg->out_addr);

    uint32_t t = blockIdx.x * blockDim.x + threadIdx.x;
    if (t >= arg->num_threads) return;

    uint32_t key = keys[t];
    uint32_t count = 0;
    for (uint32_t i = 0; i < arg->n; ++i) {
        count++;
        if (vals[i] == key) break;
    }
    out[t] = count;
}
