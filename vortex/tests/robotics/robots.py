import numpy as np
from dataclasses import dataclass

@dataclass
class SelfTestCase:
    name: str
    joint_angles: list[float]
    flange_xyz: tuple[float, float, float]
    tol: float = 1e-4

def trans(x, y, z):
    M = np.eye(4)
    M[:3,3] = (x, y, z)
    return M

def rot_z(a):
    ca, sa = np.cos(a), np.sin(a)
    M = np.eye(4)
    
    M[0, 0] = ca
    M[0, 1] = -sa

    M[1, 0] = sa
    M[1, 1] = ca
    return M

def rot_x(a):
    ca, sa = np.cos(a), np.sin(a)
    M = np.eye(4)
    
    M[1, 1] = ca
    M[1, 2] = -sa

    M[2, 1] = sa
    M[2, 2] = ca
    return M

def rot_y(a):
    ca, sa = np.cos(a), np.sin(a)
    M = np.eye(4)
    
    M[0, 0] = ca
    M[0, 2] = sa

    M[2, 0] = -sa
    M[2, 2] = ca
    return M
    
def rot_z_batch(theta):
    ct, st = np.cos(theta), np.sin(theta)
    M = np.zeros((len(theta), 4, 4))
    
    M[:, 0, 0] = ct
    M[:, 0, 1] = -st
    
    M[:, 1, 0] = st
    M[:, 1, 1] = ct
    
    M[:, 2, 2] = 1.0
    M[:, 3, 3] = 1.0
    return M

class Robot():
    def __init__(self, name: str, low: list[float], high: list[float], self_test_cases: list[SelfTestCase], joint_pre, joint_post, tool, link_radius, min_segment = 1e-6, skip_base = False, reach:float = 0, base_clearance: tuple[float, float] = (0, 0)):
        self.name = name
        self.low = np.asarray(low, dtype=np.float64)
        self.high = np.asarray(high, dtype=np.float64)
        self.reach = reach
        self.base_clearance = base_clearance            # defines how much clear space we put around the robot base.
        self.self_test_cases = self_test_cases          # have to be calculated ourselves
        # Matrices used to calculate the transformation matrix. Depends on whether the convention utilized is DH or URDF. For URDF, joint_post = I so it doesn't do anything.
        self.joint_pre  = np.asarray(joint_pre, dtype=np.float64).reshape(-1, 4, 4)
        self.joint_post = np.asarray(joint_post, dtype=np.float64).reshape(-1, 4, 4)
        self.tool = np.asarray(tool, dtype=np.float64).reshape(-1, 4, 4)
        self.dof = len(self.joint_pre)
        
        # checking if there are any segments too short to be considered. Using a min_segment length for floating point error tolerance. Any segment less than the tolerance is treated as zero len, which breaks DH if left in.
        origins0 = self._link_frames(np.zeros((1, self.dof)))[0, :, :3, 3]
        seg_len = np.linalg.norm(np.diff(origins0, axis=0), axis=-1)
        seg_start = np.flatnonzero(seg_len > min_segment)
        
        if skip_base:
            seg_start = seg_start[seg_start > 0]
        self.seg_start = seg_start
        self.num_links = len(seg_start)
        self.link_radius = np.broadcast_to(np.asarray(link_radius, dtype=np.float64), (self.num_links,)).copy()
    
    def _as_batch(self, joint_angles):
        joint_angles = np.atleast_2d(np.asarray(joint_angles, dtype=np.float64))
        
        assert(joint_angles.shape[1] == self.dof)
        
        return joint_angles
    
    # takes in the angles of the pose, uses shape (B, dof)
    def link_obbs(self, joint_angles):
        joint_angles = self._as_batch(joint_angles)
        
        centers, axes, half_extents = self._link_obbs(joint_angles)
        
        return centers, axes, half_extents
    
    # computes the transformation frame for each link, including any tools (non-segments) at the end of the sequential arm
    def _link_frames(self, joint_angles):
        B = joint_angles.shape[0]
        T = np.broadcast_to(np.eye(4), (B, 4, 4))
        frames = [T]

        # Both DH and URDF (at least for the iiwa) exclusively rotate around their z axis, so both can utilize the z rotate matrix.
        for i in range(self.dof):
            T = T @ self.joint_pre[i] @ rot_z_batch(joint_angles[:, i]) @ self.joint_post[i]
            frames.append(T)
        
        for F in self.tool:
            T = T @ F
            frames.append(T)
        
        return np.stack(frames, axis = 1)
    
    
    def flange_position(self, joint_angles):
        return self._link_frames(self._as_batch(joint_angles))[:, -1, :3, 3]
    
    # computes the OBBs for the links for a pose
    def _link_obbs(self, joint_angles):
        origins = self._link_frames(joint_angles)[..., :3, 3]
        start = origins[:, self.seg_start]
        end = origins[:, self.seg_start + 1]
        
        centers = 0.5 * (start + end)   # segment center
        vec = end - start           # vec that goes in direction of the segment
        length = np.linalg.norm(vec, axis = -1, keepdims = True)
        x_axis = vec / length       # unit vec along the direction of the vec that goes in the direction of the segment.

        # Picks whichever of the e_i basis vectors is least parallel to the above x_axis as crossing with the e_z would give 0 when y is straight vertical.
        helper = np.eye(3)[np.argmin(np.abs(x_axis), axis = -1)]
        y_axis = np.cross(x_axis, helper)
        y_axis /= np.linalg.norm(y_axis, axis=-1, keepdims=True)
        
        z_axis = np.cross(x_axis, y_axis)
        z_axis /= np.linalg.norm(z_axis, axis=-1, keepdims=True)
        
        # axes aligned with the box, centered at the box center
        axes = np.stack([x_axis, y_axis, z_axis], axis=-2)
        r = np.broadcast_to(self.link_radius, length.shape[: -1])
        half_extents = np.stack([length[..., 0] / 2 + r, r, r], axis=-1)

        return centers, axes, half_extents
    
    def selftest(self, num_random=1000, seed=0):
        test_passed = True
        for case in self.self_test_cases:
            got = self.flange_position(case.joint_angles)[0]
            err = np.max(np.abs(got - np.asarray(case.flange_xyz)))
            
            passed = (err <= case.tol)
            test_passed &= passed
            print(f"[{'PASS' if passed else 'FAIL'}] {self.name}/{case.name}: "
                  f"got {np.round(got, 4)}, expected {case.flange_xyz}, err {err:.1e}")

        rng = np.random.default_rng(seed)
        samples = rng.uniform(self.low, self.high, size=(num_random, self.dof))
        batched = self.flange_position(samples)
        
        sequential_stack = np.stack([self.flange_position(s)[0] for s in samples[:20]])

        finite = np.all(np.isfinite(batched))
        consistent = np.allclose(batched[:20], sequential_stack, atol=1e-12)
        
        test_passed &= finite and consistent
        print(f"[{'PASS' if finite else 'FAIL'}] {self.name}/random: {num_random} poses within limits, all finite")
        print(f"[{'PASS' if consistent else 'FAIL'}] {self.name}/batching: batched == one pose at a time")
        
        centers, axes, half_extents = self.link_obbs(samples)
        
        
        orthonormal = np.allclose(axes @ axes.swapaxes(-1, -2), np.eye(3), atol=1e-9)
        test_passed &= orthonormal
        
        det_near_1   = np.allclose(np.linalg.det(axes), 1.0, atol=1e-9)
        test_passed &= det_near_1
        
        half_extents_finite = np.all(np.isfinite(half_extents)) 
        half_extents_positive = np.all(half_extents > 0)
        test_passed &= half_extents_finite and half_extents_positive
        
        half_extents_rigid = np.allclose(half_extents, half_extents[:1])
        test_passed &= half_extents_rigid
        
        flange   = self.flange_position(samples)
        u        = np.einsum('bij,bj->bi', axes[:, -1], flange - centers[:, -1])
        flange_inside_half_extents   = np.all(np.abs(u) <= half_extents[:, -1] + 1e-9)       # flange is inside the last box
        test_passed &= flange_inside_half_extents
        
        return test_passed
        
class DHRobot(Robot):
    def __init__(self, config: dict):
        # each row is (a, d, alpha, theta_offset) per the Denavit-Hartenberg convention,  one row per joint
        rows = np.asarray(config["rows"], dtype=np.float64)
        
        tool_rows = np.asarray(config.get("tool_rows", []), dtype=np.float64).reshape(-1, 4)
        
        def dh_setup_pre_post(convention, a, d, alpha, offset):
            if convention == "modified":
                return rot_x(alpha) @ trans(a, 0, 0) @ rot_z(offset), trans(0, 0, d)
            if convention == "standard":
                return rot_z(offset), trans(0, 0, d) @ trans(a, 0, 0) @ rot_x(alpha)
            else: raise ValueError
        
        self.convention = config.get("convention", "standard")
        self.rows = rows
        self.tool_rows = tool_rows
        
        pre_post_pairs = [dh_setup_pre_post(self.convention, *row) for row in rows]
        joint_pre = [pre for pre, _ in pre_post_pairs]
        joint_post = [post for _, post in pre_post_pairs]
        
        # Tool rows don't have joints, so they'll always have rotation 0, which means the rotation matrix is just I.
        tool = [Pres @ Posts for Pres, Posts in (dh_setup_pre_post(self.convention, *row) for row in tool_rows)]


        super().__init__(
                name=config["name"],
                low=config["low"],
                high=config["high"],
                reach=config["reach"],
                base_clearance=config["base_clearance"],
                self_test_cases=config.get("selftest_cases", []),
                joint_pre=joint_pre,
                joint_post=joint_post,
                tool=tool,
                link_radius=config["link_radius"],
                min_segment=config.get("min_segment", 1e-6),
                skip_base=config.get("skip_base", False)
            )
        
class URDFRobot(Robot):
    def __init__(self, config: dict):
        # each joint is ((x, y, z), (roll, pitch, yaw)) per the URDF convention,  one row per joint.
        # Only utilized for computing the pre matrices.
        joint_pre = [trans(*xyz) @ rot_z(rpy[2]) @ rot_y(rpy[1]) @ rot_x(rpy[0]) for xyz, rpy in config["joints"]]
        joint_post = [np.eye(4)] * len(joint_pre) # All I's since URDF doesn't use the post matrix.
        
        super().__init__(
                name=config["name"],
                low=config["low"],
                high=config["high"],
                reach=config.get("reach"),
                base_clearance=config.get("base_clearance"),
                self_test_cases=config.get("selftest_cases", []),
                joint_pre=joint_pre,
                joint_post=joint_post,
                tool=config["tool"],
                link_radius=config["link_radius"],
                min_segment=config.get("min_segment", 1e-6),
                skip_base=config.get("skip_base", False)
            )
    

PANDA_CFG = {
    "name": "panda",
    "convention": "modified",
    "rows": [  # a, d, alpha, theta_offset
        (0.0,     0.333,  0.0,        0.0),
        (0.0,     0.0,   -np.pi / 2,  0.0),
        (0.0,     0.316,  np.pi / 2,  0.0),
        (0.0825,  0.0,    np.pi / 2,  0.0),
        (-0.0825, 0.384, -np.pi / 2,  0.0),
        (0.0,     0.0,    np.pi / 2,  0.0),
        (0.088,   0.0,    np.pi / 2,  0.0),
    ],
    "tool_rows": [(0.0, 0.107, 0.0, 0.0)],
    "low":  [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973],
    "high": [ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973],
    "reach": 0.85,
    "base_clearance": (0.15, 0.35),
    "link_radius": 0.07,
    "selftest_cases": [
        SelfTestCase("zero",  [0, 0, 0, 0, 0, 0, 0],                            (0.088, 0.0, 0.926)),
        SelfTestCase("ready", [0, -np.pi/4, 0, -3*np.pi/4, 0, np.pi/2, np.pi/4], (0.3069, 0.0, 0.5903)),
        SelfTestCase("ready_q1", [np.pi/2, -np.pi/4, 0, -3*np.pi/4, 0, np.pi/2, np.pi/4], (0.0, 0.3069, 0.5903)),
    ],
}

KUKA_CFG = {
    "name": "kuka_iiwa",
    "joints": [  # (xyz, rpy) taken from model_0.urdf, every axis is "0 0 1" per the URDF, so the joints always spin around their z axis.
        ((0, 0, 0.1575), (0, 0, 0)),
        ((0, 0, 0.2025), (np.pi/2, 0, np.pi)),
        ((0, 0.2045, 0), (np.pi/2, 0, np.pi)),
        ((0, 0, 0.2155), (np.pi/2, 0, 0)),
        ((0, 0.1845, 0), (-np.pi/2, np.pi, 0)),
        ((0, 0, 0.2155), (np.pi/2, 0, 0)),
        ((0, 0.081, 0),  (-np.pi/2, np.pi, 0)),
    ],
    "low":  [-2.967, -2.094, -2.967, -2.094, -2.967, -2.094, -3.054],
    "high": [ 2.967,  2.094,  2.967,  2.094,  2.967,  2.094,  3.054],
    "box_pad": 0.002,
    "tool": [trans(0, 0, 0.045)],
    "skip_base": True,
    "link_radius": 0.135,
    "selftest_cases": [SelfTestCase("zero", [0]*7, (0.0, 0.0, 1.306))],
}


ROBOTS = {"panda": DHRobot(PANDA_CFG), "iiwa": URDFRobot(KUKA_CFG)}

if __name__ == "__main__":
    import sys
    sys.exit(0 if all(robot.selftest() for robot in ROBOTS.values()) else 1)
