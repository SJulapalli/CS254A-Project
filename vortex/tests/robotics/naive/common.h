#ifndef _COMMON_H_
#define _COMMON_H_

#include <stdint.h>

typedef struct {
    float center[3];
    float half_extent[3];
    float axis[3][3];
} obb_t;

typedef struct {
    float center[3];
    float half_extent[3];
} aabb_t;

typedef struct {
    uint32_t first_pose;
    uint32_t num_poses;
} motion_t;

typedef struct {
    uint32_t collided; //0-1 value
    uint32_t cdq_count;
    uint32_t box_tests;
    uint32_t hit_pose; //0xFFFFFFFF will mean no hits.
    uint32_t hit_link; //0xFFFFFFFF will mean no hits.
} result_t;

typedef struct {
    uint64_t obb_addr;
    uint64_t obstacle_addr;
    uint64_t motion_addr;
    uint64_t result_addr;
    uint32_t num_motions;
    uint32_t links_per_pose;
    uint32_t num_obstacles;
} kernel_arg_t;

static_assert(sizeof(obb_t) == 60, "obb_t layout");
static_assert(sizeof(aabb_t) == 24, "aabb_t layout");
static_assert(sizeof(motion_t) == 8, "motion_t layout");
static_assert(sizeof(result_t) == 20, "result_t layout");
static_assert(sizeof(kernel_arg_t) == 48, "kernel_arg_t layout + 4 bytes padding");

#ifndef __VORTEX__
typedef struct {
    char magic[4];
    uint32_t seed; //for reproducibility on rando-gen
    uint32_t num_motions;
    uint32_t num_poses; // total poses across all motions
    uint32_t links_per_pose;
    uint32_t num_obstacles;
    uint32_t obb_size;
    uint32_t aabb_size;
} file_header_t;

static_assert(sizeof(file_header_t) == 32, "file_header_t layout"); 
#endif

#endif
