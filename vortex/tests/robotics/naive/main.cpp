#include <vortex2.h>
#include "common.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <unistd.h>
#include <vector>

#define CHECK(expr) do { \
    vx_result_t _r = (expr); \
    if (_r != VX_SUCCESS) { \
        std::fprintf(stderr, "FAIL %s:%d: '%s' returned %s\n", \
                     __FILE__, __LINE__, #expr, vx_result_string(_r)); \
        std::exit(1); \
    } \
} while (0)

namespace {
const char* kernel_file = "kernel.vxbin";
const char* data_file   = "data.bin";

void parse_args(int argc, char** argv) {
    int c;
    while ((c = getopt(argc, argv, "f:k:h")) != -1) {
        switch (c) {
            case 'f': data_file   = optarg; break;
            case 'k': kernel_file = optarg; break;
            default:
                std::cout << "Usage: [-k kernel] [-f file] [-h]" << std::endl;
                std::exit(c == 'h' ? 0 : -1);
        }
    }
}

file_header_t header;
std::vector<obb_t>    obbs;
std::vector<aabb_t>   obstacles;
std::vector<motion_t> motions;
std::vector<uint8_t> labels;

// Read file into memory & quit if file runs out early.
void read_exact(void* dst, size_t size, size_t count, FILE* f) {
    if (std::fread(dst, size, count, f) != count) {
        std::fprintf(stderr, "file too short\n");
        std::exit(1);
    }
}

void load_file(const char* path) {
    FILE* f = std::fopen(path, "rb");
    // Check if file exists
    if (!f) {
        std::fprintf(stderr, "cannot open %s\n", path);
        std::exit(1);
    }

    read_exact(&header, sizeof(header), 1, f);
    // Sanity check: struct sizes in the file must match common.h
    if (header.obb_size != sizeof(obb_t) || header.aabb_size != sizeof(aabb_t)) {
        std::fprintf(stderr, "struct size mismatch\n");
        std::exit(1);
    }

    // Read rest of data then save
    obbs.resize(header.num_poses * header.links_per_pose);
    obstacles.resize(header.num_obstacles);
    motions.resize(header.num_motions);

    read_exact(obbs.data(),      sizeof(obb_t),    obbs.size(),      f);
    read_exact(obstacles.data(), sizeof(aabb_t),   obstacles.size(), f);
    read_exact(motions.data(),   sizeof(motion_t), motions.size(),   f);
    labels.resize(obbs.size());
    read_exact(labels.data(), 1, labels.size(), f);
    std::fclose(f);
}

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

// CPU version of the kernel: one result per question.
// Checks poses in order, stops at the first link that hits.
std::vector<result_t> check_cpu() {
    std::vector<result_t> out(header.num_motions);
    uint32_t L = header.links_per_pose;

    // Looping over motions/questions
    for (uint32_t m = 0; m < header.num_motions; ++m) {
        result_t r = {0, 0, 0, 0xFFFFFFFF, 0xFFFFFFFF};
        const motion_t& motion = motions[m];

        // Looping over each pose in the motion
        for (uint32_t p = 0; p < motion.num_poses && !r.collided; ++p) {
            uint32_t pose = motion.first_pose + p;
            // Looping over eeach link in the pose
            for (uint32_t k = 0; k < L && !r.collided; ++k) {
                const obb_t& link = obbs[pose * L + k];
                r.cdq_count++;
                // Looping over each obstacle
                for (uint32_t o = 0; o < header.num_obstacles; ++o) {
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
        out[m] = r;
    }
    return out;
}
} // namespace

int main(int argc, char** argv) {
    // Parse args (data file)
    parse_args(argc, argv);
    std::cout << "data file: " << data_file << std::endl;

    load_file(data_file);
    std::printf("motions=%u poses=%u obstacles=%u\n",
                header.num_motions, header.num_poses, header.num_obstacles);

    // Run CPU version
    std::vector<result_t> expected = check_cpu();

    /*
    for (uint32_t m = 0; m < header.num_motions; ++m) {
        const result_t& r = expected[m];
        std::printf("motion %u: collided=%u hit_pose=%u hit_link=%u cdq=%u box_tests=%u\n",
                    m, r.collided, r.hit_pose, r.hit_link, r.cdq_count, r.box_tests);
    }

    // PyBullet's answer per motion: first pose/link its label says collides
    uint32_t L = header.links_per_pose;
    for (uint32_t m = 0; m < header.num_motions; ++m) {
        const motion_t& motion = motions[m];
        uint32_t hit_pose = 0xFFFFFFFF, hit_link = 0xFFFFFFFF;
        for (uint32_t p = 0; p < motion.num_poses && hit_pose == 0xFFFFFFFF; ++p) {
            for (uint32_t k = 0; k < L; ++k) {
                if (labels[(motion.first_pose + p) * L + k]) {
                    hit_pose = p;
                    hit_link = k;
                    break;
                }
            }
        }
        std::printf("pybullet %u: collided=%u hit_pose=%u hit_link=%u\n",
                    m, hit_pose != 0xFFFFFFFF, hit_pose, hit_link);
    }
    */

    // Connect to GPU
    vx_device_h dev = nullptr;
    CHECK(vx_device_open(0, &dev));

    vx_queue_info_t qi = { sizeof(qi), nullptr, VX_QUEUE_PRIORITY_NORMAL, 0 };
    vx_queue_h q = nullptr;
    CHECK(vx_queue_create(dev, &qi, &q));

    // Calculate the number of threads needed.
    // One thread per motion in the data.
    const uint32_t num_motions = header.num_motions;
    const uint32_t global_dim[1] = {num_motions};
    uint32_t grid[1], block[1];
    CHECK(vx_device_max_occupancy_grid(dev, 1, global_dim, grid, block));
    grid[0] = (num_motions + block[0] - 1) / block[0];

    // Reserve memory on the GPU for the data
    const uint64_t obb_bytes = obbs.size() * sizeof(obb_t);
    const uint64_t obstacle_bytes = obstacles.size() * sizeof(aabb_t);
    const uint64_t motion_bytes = motions.size() * sizeof(motion_t);
    const uint64_t result_bytes = num_motions * sizeof(result_t);

    vx_buffer_h obb_buf = nullptr, obstacle_buf = nullptr, motion_buf = nullptr, result_buf = nullptr;
    CHECK(vx_buffer_create(dev, obb_bytes, VX_MEM_READ, &obb_buf));
    CHECK(vx_buffer_create(dev, obstacle_bytes, VX_MEM_READ, &obstacle_buf));
    CHECK(vx_buffer_create(dev, motion_bytes, VX_MEM_READ, &motion_buf));
    CHECK(vx_buffer_create(dev, result_bytes, VX_MEM_WRITE, &result_buf));

    // Load the compiled kernel and get its entry point
    vx_module_h mod = nullptr;
    vx_kernel_h kern = nullptr;
    CHECK(vx_module_load_file(dev, kernel_file, &mod));
    CHECK(vx_module_get_kernel(mod, "main", &kern));

    // Kernel arguments: GPU buffer addresses + sizes
    kernel_arg_t kernel_arg{};
    kernel_arg.num_motions = num_motions;
    kernel_arg.links_per_pose = header.links_per_pose;
    kernel_arg.num_obstacles = header.num_obstacles;
    CHECK(vx_buffer_address(obb_buf, &kernel_arg.obb_addr));
    CHECK(vx_buffer_address(obstacle_buf, &kernel_arg.obstacle_addr));
    CHECK(vx_buffer_address(motion_buf, &kernel_arg.motion_addr));
    CHECK(vx_buffer_address(result_buf, &kernel_arg.result_addr));

    // CPU side to store GPU results
    std::vector<result_t> actual(num_motions);

    // Copy the data from CPU to GPU
    CHECK(vx_enqueue_write(q, obb_buf, 0, obbs.data(), obb_bytes, 0, nullptr, nullptr));
    CHECK(vx_enqueue_write(q, obstacle_buf, 0, obstacles.data(), obstacle_bytes, 0, nullptr, nullptr));
    CHECK(vx_enqueue_write(q, motion_buf, 0, motions.data(), motion_bytes, 0, nullptr, nullptr));

    // Define kernel settings
    vx_launch_info_t li{};
    li.struct_size = sizeof(li);
    li.kernel = kern;
    li.args_host = &kernel_arg;
    li.args_size = sizeof(kernel_arg);
    li.ndim = 1;
    li.grid_dim[0] = grid[0];
    li.block_dim[0] = block[0];    

    // Run kernel, copy results back, and wait
    vx_event_h launch_ev = nullptr, read_ev = nullptr;
    CHECK(vx_enqueue_launch(q, &li, 0, nullptr, &launch_ev));
    CHECK(vx_enqueue_read(q, actual.data(), result_buf, 0, result_bytes, 1, &launch_ev, &read_ev));
    CHECK(vx_event_wait_value(read_ev, 1, VX_TIMEOUT_INFINITE));

    // Check GPU results to CPU
    int errors = 0;
    for (uint32_t m = 0; m < num_motions; ++m) {
        const result_t& e = expected[m];
        const result_t& a = actual[m];
        if (a.collided != e.collided || a.hit_pose != e.hit_pose ||
            a.hit_link != e.hit_link || a.cdq_count != e.cdq_count ||
            a.box_tests != e.box_tests) {
            std::printf("*** motion %u: got collided=%u hit_pose=%u hit_link=%u cdq=%u box_tests=%u\n",
                        m, a.collided, a.hit_pose, a.hit_link, a.cdq_count, a.box_tests);
            ++errors;
        }
    }

    // Totals for the results CSV
    uint64_t total_cdq = 0, total_box_tests = 0;
    for (uint32_t m = 0; m < num_motions; ++m) {
        total_cdq       += actual[m].cdq_count;
        total_box_tests += actual[m].box_tests;
    }
    std::printf("TOTALS: cdq=%lu box_tests=%lu\n",
                (unsigned long)total_cdq, (unsigned long)total_box_tests);

    // Check GPU results against PyBullet labels
    // Only reports error if PyBullet says collided but GPU says free.
    uint32_t L = header.links_per_pose;
    uint32_t extras = 0;
    for (uint32_t i = 0; i < num_motions; ++i) {
        const motion_t& motion = motions[i];
        bool pybullet_collided = false;
        for (uint32_t p = 0; p < motion.num_poses && !pybullet_collided; ++p)
            for (uint32_t k = 0; k < L && !pybullet_collided; ++k)
                pybullet_collided = labels[(motion.first_pose + p) * L + k];
        // error
        if (pybullet_collided && !actual[i].collided) {
            std::printf("*** motion %u: pybullet collided, box free\n", i);
            ++errors;
        }
        // not error, warning
        else if (!pybullet_collided && actual[i].collided) {
            ++extras;
        }
    }

    // Free GPU resources
    vx_event_release(read_ev);
    vx_event_release(launch_ev);
    vx_buffer_release(result_buf);
    vx_buffer_release(motion_buf);
    vx_buffer_release(obstacle_buf);
    vx_buffer_release(obb_buf);
    vx_kernel_release(kern);
    vx_module_release(mod);
    vx_queue_release(q);
    vx_device_dump_perf(dev, stdout);
    vx_device_release(dev);

    if (extras) {
        std::printf("%u of %u motions: PyBullet said free but box said collided\n",
                    extras, num_motions);
    }

    if (errors) {
        std::cout << "Found " << errors << " errors!\nFAILED!" << std::endl;
        return errors;
    }

    std::cout << "PASSED!" << std::endl;

    return 0;
}