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