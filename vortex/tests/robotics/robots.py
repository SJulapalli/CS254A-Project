import numpy as np
from abc import ABC, abstractmethod
from dataclasses import dataclass

@dataclass
class SelfTestCase:
    name: str
    joint_angles: list[float]
    flange_xyz: tuple[float, float, float]
    tol: float = 1e-4


class Robot(ABC):
    def __init__(self, name: str, dof: int, low: list[float], high: list[float], reach:float, base_clearance: tuple[float, float], self_test_cases: list[SelfTestCase]):
        self.name = name
        self.dof = dof
        self.low = np.asarray(low, dtype=np.float64)
        self.high = np.asarray(high, dtype=np.float64)
        self.reach = reach
        self.base_clearance = base_clearance            # defines how much clear space we put around the robot base.
        self.self_test_cases = self_test_cases          # have to be calculated ourselves
    
    def _as_batch(self, joint_angles):
        joint_angles = np.atleast_2d(np.asarray(joint_angles, dtype=np.float64))
        
        assert(joint_angles.shape[1] == self.dof)
        
        return joint_angles
    
    # takes in the angles of the pose, uses shape (B, dof)
    def link_obbs(self, joint_angles):
        joint_angles = self._as_batch(joint_angles)
        
        centers, axes, half_extents = self._link_obbs(joint_angles)
        
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
        
        return test_passed
    
    @abstractmethod
    def _link_obbs(self, joint_angles):
        pass
    
    @abstractmethod
    def flange_position(self, joint_angles):
        pass
        
class DHRobot(Robot):
    def __init__(self, config: dict):
        # each row is (a, d, alpha, theta_offset) per the Denavit-Hartenberg convention,  one row per joint
        rows = np.asarray(config["rows"], dtype=np.float64)
        
        tool_rows = np.asarray(config.get("tool_rows", []), dtype=np.float64).reshape(-1, 4)

        super().__init__(
                name=config["name"],
                dof=len(rows),
                low=config["low"],
                high=config["high"],
                reach=config["reach"],
                base_clearance=config["base_clearance"],
                self_test_cases=config.get("selftest_cases", []),
            )

        self.convention = config.get("convention", "standard")
        self.rows = rows
        self.tool_rows = tool_rows
        self.link_radius = config["link_radius"]
        self.min_segment = config.get("min_segment", 1e-6)
        
        # removing frames that represent no change in length cuz they're rotations, len = 0 so they break the obb calcs.
        all_rows = np.vstack([self.rows, self.tool_rows])
        seg_len = np.hypot(all_rows[:, 0], all_rows[:, 1])
        self.seg_start = np.flatnonzero(seg_len > self.min_segment)
        self.num_links = len(self.seg_start)
        self.link_radius = np.broadcast_to(np.asarray(config["link_radius"], dtype=np.float64), (self.num_links,)).copy()
        # print(origins_zero)
        # print(seg_len)
        # print(self.seg_start)
        # print(self.num_links)
        # print(self.link_radius)
    
    def transform_helper(self, a, d, alpha, theta):
        theta = np.asarray(theta, dtype=np.float64)
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        T = np.zeros((len(theta), 4, 4))
        
        if self.convention == "modified":
            T[:, 0, 0] = ct
            T[:, 0, 1] = -st
            T[:, 0, 3] = a
            
            T[:, 1, 0] = st * ca
            T[:, 1, 1] = ct * ca
            T[:, 1, 2] = -sa
            T[:, 1, 3] = -sa * d
            
            T[:, 2, 0] = st * sa
            T[:, 2, 1] = ct * sa
            T[:, 2, 2] = ca
            T[:, 2, 3] = ca * d
        elif self.convention == "standard":
            T[:, 0, 0] = ct
            T[:, 0, 1] = -st * ca
            T[:, 0, 2] = st * sa
            T[:, 0, 3] = a * ct
            
            T[:, 1, 0] = st
            T[:, 1, 1] = ct * ca
            T[:, 1, 2] = -ct * sa
            T[:, 1, 3] = a * st
            
            T[:, 2, 1] = sa
            T[:, 2, 2] = ca
            T[:, 2, 3] = d
        else:
            raise ValueError()
        
        T[:, 3, 3] = 1.0
        return T
    
    def _frame_origins(self, joint_angles):
        B = joint_angles.shape[0]
        T = np.tile(np.eye(4), (B, 1, 1))
        origins = [T[:, :3, 3].copy()]

        for i, (a, d, alpha, offset) in enumerate(self.rows):
            T = T @ self.transform_helper(a, d, alpha, joint_angles[:, i] + offset)
            origins.append(T[:, :3, 3].copy())

        for a, d, alpha, offset in self.tool_rows:
            T = T @ self.transform_helper(a, d, alpha, np.full(B, offset))
            origins.append(T[:, :3, 3].copy())

        return np.stack(origins, axis=1)
    
    def flange_position(self, joint_angles):
        return self._frame_origins(self._as_batch(joint_angles))[:, -1, :]
    
    def _link_obbs(self, joint_angles):
        origins = self._frame_origins(joint_angles)
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

ROBOTS = {"panda": DHRobot(PANDA_CFG)}

if __name__ == "__main__":
    import sys
    sys.exit(0 if all(robot.selftest() for robot in ROBOTS.values()) else 1)
