import math

import numpy as np
from gym_robotics.envs import rotations

SQRT_3_HALF = 0.5 * math.sqrt(3)


def direct_set_action(sim, action: np.ndarray) -> None:
    assert action.shape == (9,)
    max_forces = sim.model.actuator_forcerange[:7, 1]

    if sim.data.ctrl is not None:
        # Revolute joints
        sim.data.ctrl[:7] = action[:7] * max_forces

        # Gripper
        for i in range(7, 9):
            idx = sim.model.jnt_qposadr[sim.model.actuator_trnid[i, 0]]
            sim.data.ctrl[i] = sim.data.qpos[idx] + action[i]


def cuboid_to_capsule(pos: np.ndarray, rot: np.ndarray, size: np.ndarray) -> np.ndarray:
    capsule = np.zeros(7, dtype=np.float64)  # [p1x, p1y, p1z, p2x, p2y, p2z, radius]
    if size[0] == size[1] == size[2]:  # Special case for cube to minimize empty space
        capsule[:3] = pos
        capsule[3:6] = pos
        capsule[6] = SQRT_3_HALF * size[0]  # r_sphere is half of longest diagonal in cube
    else:
        indices = np.argsort(size)

        offset = np.zeros(3, dtype=np.float64)
        offset[indices[2]] = size[indices[2]] * 0.5  # Half the size in the largest dimension
        offset = rotations.quat_rot_vec(rot, offset)

        capsule[:3] = pos - offset  # line from pos
        capsule[3:6] = pos + offset  # line to pos
        capsule[6] = 0.5 * math.sqrt(size[indices[0]] ** 2 + size[indices[1]] ** 2)  # capsule radius

    return capsule
