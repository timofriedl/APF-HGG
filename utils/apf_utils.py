import math

import numpy as np
from gym_robotics.envs import rotations

from apf.wrapper import control_step

SQRT_2_HALF = 0.5 * math.sqrt(2.0)
SQRT_3_HALF = 0.5 * math.sqrt(3.0)
SKIP_RL = False


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

        capsule[:3] = pos - offset  # line-from pos
        capsule[3:6] = pos + offset  # line-to pos
        capsule[6] = 0.5 * math.sqrt(size[indices[0]] ** 2 + size[indices[1]] ** 2)  # capsule radius

    return capsule


def get_theta(sim) -> np.ndarray:
    theta = np.zeros(7, dtype=np.float64)
    names = [n for n in sim.model.joint_names if n.startswith("robot")]
    for i in range(7):
        theta[i] = sim.data.get_joint_qpos(names[i])

    return theta


def setup_obstacle_capsules(env, obstacle_names):
    for i, obstacle in enumerate(env.obstacles):
        body_id = env.sim.model.body_name2id(
            obstacle_names[i]
        )
        pos = env.sim.data.body_xpos[body_id]
        rot = env.sim.data.body_xquat[body_id]
        size = env.obstacles[i][7:10]
        env.obstacle_capsules[i] = cuboid_to_capsule(pos, rot, size)


def get_capsules(env) -> np.ndarray:
    dyn_obstacles = env.dyn_door_obstacles if env.dyn_door_obstacles is not None else env.dyn_obstacles
    for i in range(len(dyn_obstacles)):
        body_id = env.sim.model.body_name2id(
            env.dyn_obstacle_names[i]
        )
        pos = env.sim.data.body_xpos[body_id]
        rot = env.sim.data.body_xquat[body_id]
        size = dyn_obstacles[i][7:10]
        env.obstacle_capsules[i] = cuboid_to_capsule(pos, rot, size)

    return env.obstacle_capsules


def apf_set_action(env, rl_action):
    rl_action = rl_action.copy()

    # Extract goal position
    current_pos = env.sim.data.get_body_xpos('eef')
    env.rl_goal_pos[:] = env.goal[:3] + np.array([0, 0, 0.05], dtype=np.float64) if SKIP_RL else current_pos[:3] + env.limit_action * rl_action[:3]
    if env.block_z:
        if env.rl_goal_pos[2] > env.block_max_z:
            env.rl_goal_pos[2] = env.block_max_z
        elif env.rl_goal_pos[2] < 0.48:
            env.rl_goal_pos[2] = 0.48

    # Get target orientation
    if env.block_orientation:
        env.rl_goal_rot[:] = np.array([1, 0, 0, 0], dtype=np.float64)  # qw, qx, qy, qz
    else:
        [qx, qy, qz, qw] = rl_action[3:7]
        env.rl_goal_rot[:] += env.limit_action * np.array([qw, qx, qy, qz], dtype=np.float64)  # qw, qx, qy, qz

    # Current joint configuration
    theta = get_theta(env.sim)

    # Obstacle capsules
    obstacle_attributes = get_capsules(env)
    obstacle_attributes[:, 0:3] -= env.robot_offset  # Transform to robot base frame
    obstacle_attributes[:, 3:6] -= env.robot_offset  # Transform to robot base frame

    # Compute joint torques
    dt = env.sim.model.opt.timestep
    torques = control_step(theta, env.rl_goal_pos - env.robot_offset, env.rl_goal_rot, env.pid_rot_weight,
                           obstacle_attributes, dt, env.pid_integral, env.pid_prev_error)

    # Normalize torques to [-1, 1]
    env.direct_action[:7] = torques / env.sim.model.actuator_forcerange[:7, 1]

    # Directly use RL gripper action
    gripper_ctrl = -0.8 if SKIP_RL or env.block_gripper else rl_action[7]
    env.direct_action[7] = gripper_ctrl
    env.direct_action[8] = gripper_ctrl

    # Skip the usual mocap control and use direct torque control
    direct_set_action(env.sim, env.direct_action)
