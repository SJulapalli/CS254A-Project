# KUKA collision data for Vortex


## 1. Install the planner's dependencies

From the top of the repo:

```
cd coll_prediction_artifact/trace_generation/bit_planning
pip install torch --extra-index-url https://download.pytorch.org/whl/cpu
pip install -r requirements.txt --no-build-isolation --extra-index-url https://download.pytorch.org/whl/cpu
```

## 2. Run planner

```
mkdir -p logfiles_BIT_link
python3 eval_all.py 2000 BIT*
```

The planner runs problems 2000 to 2199, one after another. For each problem,
it writes two files to logfiles_BIT_link/:

link_info_<id>.pkl: every collision check the planner made, in order.
For each pose it checked, and for each of the KUKA's 7 links:

- center of mass (x, y, z)
- rotation as a quaternion (qx, qy, qz, qw)
- collision result: 1 = free, 0 = collides

The file is a pair (poses, labels):

```
poses[question][pose][link]  = [x, y, z, qx, qy, qz, qw]
labels[question][pose][link] = 1 or 0
```

A "question" is either one sampled pose, or a motion between two poses
(several poses along the way).

obstacles_<id>.pkl: the problem's obstacles, as a list of
axis-aligned boxes:

```
[(half_size [hx, hy, hz], center [cx, cy, cz]), ...]
```