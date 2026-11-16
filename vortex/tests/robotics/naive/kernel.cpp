#include <vx_spawn2.h>
#include "common.h"
#include <math.h>


float dot(const float* u, const float* v) {
    return u[0]*v[0] + u[1]*v[1] + u[2]*v[2];
}

void cross(const float* u, const float* v, float* out) {
    out[0] = u[1]*v[2] - u[2]*v[1];
    out[1] = u[2]*v[0] - u[0]*v[2];
    out[2] = u[0]*v[1] - u[1]*v[0];
}

// True if there is a gap between the two boxes along direction L
bool gap_along(const float* L, const obb_t& b, const aabb_t& a) {
    // Vector between centers
    float d[3] = {b.center[0] - a.center[0], b.center[1] - a.center[1], b.center[2] - a.center[2]};

    // Distance between centers projected onto L
    float dist   = std::fabs(dot(d, L));
    
    // How far the obstacle sticks out along L
    float reachA = a.half_extent[0] * std::fabs(L[0])
                 + a.half_extent[1] * std::fabs(L[1])
                 + a.half_extent[2] * std::fabs(L[2]);
    // How far the link sticks out along L
    float reachB = b.half_extent[0] * std::fabs(dot(b.axis[0], L))
                 + b.half_extent[1] * std::fabs(dot(b.axis[1], L))
                 + b.half_extent[2] * std::fabs(dot(b.axis[2], L));
    // If the distance between centers is greater than the sum of the reaches, there is a gap
    return dist > reachA + reachB;
}


// OBB (link) vs AABB (obstacle)
// Returns true if they overlap
/*
    To understand the algorithm easier, start with 2D.

    If two boxes do not overlap, there must exist some line that separates them.
    Key insight: There must exist a line that is parallel to one of the edges of either box.
    In other words, if we project the boxes on each box's 2 edge directions (4 total),
    and there is a gap on any, that means that the boxes do not overlap.
    Mathematically, this is just the dot product. Remember, dot product to some direction L
    means that you are projecting onto the direction L. So, we just need to see if the vector
    between the centers projected onto some axis is greater than the sum of the reaches of each box
    in that direction.


    Algorithm in 3D:
    If 2 boxes do not overlap, there must exist some plane that separates them.
    Key insight: There must exist a plane that is parallel to one of the faces of either box OR
    parallel to one out of the 3 edges in each box. The first part is obvious. The reason why the second
    portion is needed is because if the two boxes almost intersect at an edge, then no planes parallel to faces
    will spli the boxes, even if they do not overlap.

    Thus, we need to test on 15 planes:
        3 face directions from A
        3 face directions from B
        3x3=9 face directions formed by being parallel to an edge in A and edge in B.
*/
bool obb_vs_aabb(const obb_t& b, const aabb_t& a) {
    // face directions of a
    const float a_axis[3][3] = { {1, 0, 0}, {0, 1, 0}, {0, 0, 1} };

    // 3 face directions of the obstacle
    for (int i = 0; i < 3; ++i)
        if (gap_along(a_axis[i], b, a)) return false;

    // 3 face directions of the link
    for (int j = 0; j < 3; ++j)
        if (gap_along(b.axis[j], b, a)) return false;

    // 9 cross products: each obstacle edge with each link edge
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            float L[3];
            cross(a_axis[i], b.axis[j], L);
            if (gap_along(L, b, a)) return false;
        }

    // No gap on any of the 15 axes
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
                    //break;
                }
            }
        }
    }

    results[m] = r;
}