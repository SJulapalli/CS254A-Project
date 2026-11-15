import numpy as np
import pickle
from pathlib import Path
from robots import Robot, ROBOTS
import os

OBB_DT = np.dtype([("center", "<f4", 3), ("half_extent", "<f4", 3), ("axis", "<f4", (3, 3))])
AABB_DT = np.dtype([("center", "<f4", 3), ("half_extent", "<f4", 3)])
MOTION_DT = np.dtype([("first_pose", "<u4"), ("num_poses", "<u4")])
HEADER_DT = np.dtype([("magic", "S4"), ("seed", "<u4"), ("num_motions", "<u4"), ("num_poses", "<u4"), ("links_per_pose", "<u4"), ("num_obstacles", "<u4"), ("obb_size", "<u4"), ("aabb_size", "<u4")])

def load_data_from_pid(trace_directory, pid, bot):
    trace_dir = Path(trace_directory)
    
    return load_data(trace_dir / f"link_info_{pid}.pkl", trace_dir / f"obstacles_{pid}.pkl", bot)
        
# loads the data from the pybullet runs. Technically the kernel spec doesn't require the motion labels, but we might be able to utilize them for comparison later.
def load_data(link_info_path, obstacles_path, bot: Robot):
    with open(link_info_path, "rb") as f:
        poses, labels = pickle.load(f)
    with open(obstacles_path, "rb") as f:
        obstacles = pickle.load(f)
    motion_poses = []
    motion_lengths = []
    # motion_labels = []
    
    for pose_set, label_set in zip(poses, labels):
        if len(pose_set) < 2:
            continue
        poses_np = np.asarray(pose_set, dtype=np.float64)
        order = [0] + list(range(3, len(poses_np))) + [1] if len(poses_np) > 2 else [0, 1]
        
        motion_poses.append(poses_np[order])
        motion_lengths.append(len(order))
        # motion_labels.append(np.asarray(label_set, dtype=np.uint8)[order])
    
    if len(motion_poses) == 0:
        return None
    
    concat_poses = np.concatenate(motion_poses)
    centers, axes, half_extents = bot.link_obbs(concat_poses)
    
    obstacle_centers = []
    obstacle_half_extents = []
    for obstacle_half_extent, obstacle_center in obstacles:
        obstacle_centers += [obstacle_center]
        obstacle_half_extents += [obstacle_half_extent]
        
    
    return motion_lengths, centers, axes, half_extents, obstacle_centers, obstacle_half_extents

def write_data_to_bin(motion_lengths, centers, axes, half_extents, obstacle_centers, obstacle_half_extents, output_directory, output_file, pid):
    output_dir = Path(output_directory)
    num_poses, num_links = centers.shape[:2] # For the iiwa, num links should always be 7
    
    obb_data = np.zeros(num_poses * num_links, dtype=OBB_DT)
    obb_data["center"] = centers.reshape(-1, 3)       # stored in pose major order, so all poses for a motion are contiguous, index is pose * num_links_per_pose + link
    obb_data["half_extent"] = half_extents.reshape(-1, 3)
    obb_data["axis"] = axes.reshape(-1, 3, 3)       # row i = obb axis i

    # print(obb_data["center"].shape)
    # print(obb_data["half_extent"].shape)
    # print(obb_data["axis"].shape)

    aabb_data = np.zeros(len(obstacle_centers), dtype=AABB_DT)
    aabb_data["center"] = obstacle_centers
    aabb_data["half_extent"] = obstacle_half_extents

    lengths = np.asarray(motion_lengths)
    motions = np.zeros(len(lengths), dtype=MOTION_DT)
    motions["first_pose"] = np.concatenate([[0], np.cumsum(lengths)[:-1]])
    motions["num_poses"] = lengths

    header = np.zeros(1, dtype=HEADER_DT)
    header["magic"] = b"URDF"           # doesn't really matter what the val is so long as main.cpp checks the same val.        
    header["seed"] = pid                # don't really have a seed rn so just storing the pid for reference.
    header["num_motions"] = len(motion_lengths)
    header["num_poses"] = num_poses
    header["links_per_pose"] = num_links
    header["num_obstacles"] = len(obstacle_centers)
    header["obb_size"], header["aabb_size"] = OBB_DT.itemsize, AABB_DT.itemsize

    # The data write order is important, it needs to be read back by main.cpp in the same order.
    output_path = output_dir / output_file
    with open(output_path, "wb") as f:
        for arr in (header, obb_data, aabb_data, motions):
            arr.tofile(f)
    
    
    
    # Validation check for the data to make sure it's outputting properly, shouldn't be ran during actual use.
    # h = np.fromfile(output_path, HEADER_DT, count=1)
    # off = HEADER_DT.itemsize
    
    # obb2 = np.fromfile(output_path, OBB_DT, count=num_poses*num_links, offset=off);
    # off += obb2.nbytes
    
    # aabb2 = np.fromfile(output_path, AABB_DT, count=len(obstacle_centers), offset=off);
    # off += aabb2.nbytes
    
    # mot2 = np.fromfile(output_path, MOTION_DT, count=len(motion_lengths), offset=off)
    
    # for a, b in zip(obb_data, obb2):
    #     assert(a == b)
        
    # for a, b in zip(aabb2, aabb_data):
    #     assert(a == b)
        
    # print(mot2)
    
    # print(h)
    # assert(os.path.getsize(output_path) == 32 + 60*num_poses*num_links + 24*len(obstacle_centers) + 8*len(motion_lengths))


if __name__ == "__main__":
    # motion_lengths, centers, axes, half_extents, obstacle_centers, obstacle_half_extents = load_data_from_pid("/Users/Suhas/CS254A-Project/coll_prediction_artifact/trace_generation/bit_planning/logfiles_BIT_link", 2000, ROBOTS["iiwa"])
    # print(centers.shape)
    # print(half_extents.shape)
    # print(axes.shape)
    # print(motion_lengths)
    # write_data_to_bin(motion_lengths, centers, axes, half_extents, obstacle_centers, obstacle_half_extents, "/Users/Suhas/CS254A-Project/vortex/tests/robotics/test_data", "test_output.bin", 2000)
    pass