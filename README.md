# Robot Collision Detection on Vortex

CS254A (Fall 2026) project. We reproduce the collision prediction idea from
Shah et al., ISCA 2024, on the Vortex RISC-V GPU, using real collision checks
from a KUKA iiwa arm planned with BIT*.

## KUKA collision data for Vortex


### 1. Install the planner's dependencies

From the top of the repo:

```
cd coll_prediction_artifact/trace_generation/bit_planning
pip install torch --extra-index-url https://download.pytorch.org/whl/cpu
pip install -r requirements.txt --no-build-isolation --extra-index-url https://download.pytorch.org/whl/cpu
```

### 2. Run planner

```
mkdir -p logfiles_BIT_link
python3 eval_all.py 2000 BIT*
```

The planner runs problems 2000 to 2199, one after another. For each problem,
it writes two files to logfiles_BIT_link/:

link_info_<id>.pkl: every collision check the planner made, in order.
For each pose it checked:

- the arm's 7 joint angles (radians)
- the collision result for each of the KUKA's 7 links: 1 = free, 0 = collides

The file is a pair (poses, labels):

```
poses[question][pose]        = [a0, a1, a2, a3, a4, a5, a6]
labels[question][pose][link] = 1 or 0
```

A "question" is either one sampled pose, or a motion between two poses
(several poses along the way).

The robot base is fixed at (0, 0, 0) with no rotation, so the joint angles
and the robot model (kuka_iiwa/model_0.urdf) fully determine every link's
position and orientation.

obstacles_<id>.pkl: the problem's obstacles, as a list of
axis-aligned boxes:

```
[(half_size [hx, hy, hz], center [cx, cy, cz]), ...]
```

To read them in Python, for example 2000 one:

```python
import pickle

poses, labels = pickle.load(open("logfiles_BIT_link/link_info_2000.pkl", "rb"))
obstacles = pickle.load(open("logfiles_BIT_link/obstacles_2000.pkl", "rb"))

print(len(poses))       # number of questions
print(poses[0][0])      # 7 joint angles of the first pose
print(labels[0][0])     # 7 collision results (1 = free, 0 = collides)
print(obstacles)        # [(half_size, center), ...]
```

### 3. Convert the joint angles to boxes.

From the top of the repo:

```
mkdir -p vortex/tests/robotics/test_data
python3 vortex/tests/robotics/gen_data.py
```

For each problem 2000 to 2199, gen_data.py reads the two log files from step 2
and writes vortex/tests/robotics/test_data/data_<id>.bin, the input file for
the Vortex collision checker.

It keeps every question, both single sampled poses and motions. Each one becomes
a motion in the file (a single pose is a motion with 1 pose). The planner saves
a motion's poses as start, end, then the poses in between (the first of which
repeats the start). gen_data.py reorders them to start, in between, end, and
drops the repeat.

For each pose it turns the 7 joint angles into 7 oriented boxes, one per link,
using the KUKA model in vortex/tests/robotics/robots.py.

It also saves PyBullet's collision result for each link, flipped to
1 = collides, 0 = free (the opposite of the log files).

Each .bin file is laid out in this order (structs in
vortex/tests/robotics/naive/common.h):

```
header      28 bytes             problem id, counts, struct sizes
link boxes  60 bytes each        center, half size, 3 axes; index = pose * 7 + link
obstacles   24 bytes each        center, half size
motions     8 bytes each         first pose, number of poses
labels      1 byte each          PyBullet result per link box; 1 = collides
```

### 4. Run collision checker on Vortex

From vortex/build:


```
make -C tests/robotics/naive run-simx
```

This runs problem 2000 by default. To run another problem:

```
make -C tests/robotics/naive run-simx OPTS="-f $PWD/../tests/robotics/test_data/data_2001.bin"
```

The test runs one GPU thread per motion. Each thread checks its poses in order,
link by link against every obstacle, and stops at the first hit.

It then checks the results two ways:

- GPU vs CPU: main.cpp runs the same check on the CPU. Every motion must
  match exactly, or the test fails.
- GPU vs PyBullet: if PyBullet's label says a motion collides but the boxes
  say it is free, the test fails. The opposite case (boxes collide, PyBullet
  free) is expected, since the boxes are larger than the real link meshes.
  These are only counted and printed as a warning.

The output ends with the cycle count and PASSED or FAILED:

```
PERF: instrs=..., cycles=..., IPC=...
PASSED!
```