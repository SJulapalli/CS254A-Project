#include <vortex2.h>
#include "common.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <vector>

#define CHECK(expr) do { \
    vx_result_t _r = (expr); \
    if (_r != VX_SUCCESS) { \
        std::fprintf(stderr, "FAIL '%s' returned %s\n", #expr, vx_result_string(_r)); \
        std::exit(1); \
    } \
} while (0)

int main(int argc, char** argv) {
    const uint32_t num_threads = 8;
    const uint32_t n = 8;
    // Optional block size from the command line (e.g. 1 = one thread per block).
    // 0 = let Vortex pick.
    uint32_t block_size = (argc > 1) ? std::atoi(argv[1]) : 0;

    // vals[i] = i
    std::vector<uint32_t> vals(256);
    for (uint32_t i = 0; i < vals.size(); i++) vals[i] = i;

    // Even threads break at i = 3 (4 iterations). Odd threads never break (8 iterations).
    std::vector<uint32_t> keys(num_threads);
    for (uint32_t t = 0; t < num_threads; t++) {
        if (t % 2 == 0) keys[t] = 3;
        else keys[t] = 100;
    }

    vx_device_h dev = nullptr;
    CHECK(vx_device_open(0, &dev));
    vx_queue_info_t qi = { sizeof(qi), nullptr, VX_QUEUE_PRIORITY_NORMAL, 0 };
    vx_queue_h q = nullptr;
    CHECK(vx_queue_create(dev, &qi, &q));

    const uint32_t global_dim[1] = {num_threads};
    uint32_t grid[1], block[1];
    CHECK(vx_device_max_occupancy_grid(dev, 1, global_dim, grid, block));
    if (block_size) block[0] = block_size;
    grid[0] = (num_threads + block[0] - 1) / block[0];
    std::printf("threads=%u grid=%u block=%u\n", num_threads, grid[0], block[0]);

    vx_buffer_h vals_buf = nullptr, keys_buf = nullptr, out_buf = nullptr;
    CHECK(vx_buffer_create(dev, vals.size() * 4, VX_MEM_READ,  &vals_buf));
    CHECK(vx_buffer_create(dev, keys.size() * 4, VX_MEM_READ,  &keys_buf));
    CHECK(vx_buffer_create(dev, num_threads * 4, VX_MEM_WRITE, &out_buf));

    vx_module_h mod = nullptr;
    vx_kernel_h kern = nullptr;
    CHECK(vx_module_load_file(dev, "kernel.vxbin", &mod));
    CHECK(vx_module_get_kernel(mod, "main", &kern));

    kernel_arg_t arg{};
    arg.num_threads = num_threads;
    arg.n = n;
    CHECK(vx_buffer_address(vals_buf, &arg.vals_addr));
    CHECK(vx_buffer_address(keys_buf, &arg.keys_addr));
    CHECK(vx_buffer_address(out_buf,  &arg.out_addr));

    CHECK(vx_enqueue_write(q, vals_buf, 0, vals.data(), vals.size() * 4, 0, nullptr, nullptr));
    CHECK(vx_enqueue_write(q, keys_buf, 0, keys.data(), keys.size() * 4, 0, nullptr, nullptr));

    vx_launch_info_t li{};
    li.struct_size  = sizeof(li);
    li.kernel       = kern;
    li.args_host    = &arg;
    li.args_size    = sizeof(arg);
    li.ndim         = 1;
    li.grid_dim[0]  = grid[0];
    li.block_dim[0] = block[0];

    std::vector<uint32_t> out(num_threads);
    vx_event_h launch_ev = nullptr, read_ev = nullptr;
    CHECK(vx_enqueue_launch(q, &li, 0, nullptr, &launch_ev));
    CHECK(vx_enqueue_read(q, out.data(), out_buf, 0, num_threads * 4, 1, &launch_ev, &read_ev));
    CHECK(vx_event_wait_value(read_ev, 1, VX_TIMEOUT_INFINITE));

    int errors = 0;
    for (uint32_t t = 0; t < num_threads; ++t) {
        uint32_t expected;
        if (t % 2 == 0) expected = 4;
        else expected = 8;
        std::printf("thread %u: expected %u, got %u%s\n",
                    t, expected, out[t], out[t] == expected ? "" : "   <-- WRONG");
        if (out[t] != expected) ++errors;
    }

    vx_event_release(read_ev);
    vx_event_release(launch_ev);
    vx_buffer_release(out_buf);
    vx_buffer_release(keys_buf);
    vx_buffer_release(vals_buf);
    vx_kernel_release(kern);
    vx_module_release(mod);
    vx_queue_release(q);
    vx_device_release(dev);

    std::cout << (errors ? "FAILED!" : "PASSED!") << std::endl;
    return errors;
}
