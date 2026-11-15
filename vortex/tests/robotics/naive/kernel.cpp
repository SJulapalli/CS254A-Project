#include <vx_spawn2.h>
#include "common.h"
#include <math.h>

// OBB (link) vs AABB (obstacle)
// Returns true if they overlap
bool obb_vs_aabb(const obb_t& b, const aabb_t& a) {
    // TODO: Fill out!
    (void)b;
    (void)a;
    return true;
}

__kernel void kernel_main(kernel_arg_t* __UNIFORM__ arg) {
    // Parse inputs
    auto obbs      = reinterpret_cast<const obb_t*>(arg->obb_addr);
    auto obstacles = reinterpret_cast<const aabb_t*>(arg->obstacle_addr);
    auto motions   = reinterpret_cast<const motion_t*>(arg->motion_addr);
    auto results   = reinterpret_cast<result_t*>(arg->result_addr);

    uint32_t L = arg->links_per_pose;
    uint32_t num_obstacles = arg->num_obstacles;

    // This is the thread's question number
    uint32_t m = blockIdx.x * blockDim.x + threadIdx.x;
    // These are extra threads on the last block that is not tasked with a question.
    if (m >= arg->num_motions) return;

    motion_t motion = motions[m];
    result_t r = {0, 0, 0, 0xFFFFFFFF, 0xFFFFFFFF};

    // Same loops as check_cpu in main.cpp
    // Looping over each pose in the motion
    for (uint32_t p = 0; p < motion.num_poses && !r.collided; ++p) {
        uint32_t pose = motion.first_pose + p;
        // Looping over eeach link in the pose
        for (uint32_t k = 0; k < L && !r.collided; ++k) {
            const obb_t& link = obbs[pose * L + k];
            r.cdq_count++;
            // Looping over each obstacle
            for (uint32_t o = 0; o < num_obstacles; ++o) {
                r.box_tests++;
                if (obb_vs_aabb(link, obstacles[o])) {
                    r.collided = 1;
                    r.hit_pose = p;
                    r.hit_link = k;
                    break;
                }
            }
        }
    }

    results[m] = r;
}