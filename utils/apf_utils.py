import math

import numpy as np
from gym_robotics.envs import rotations

from apf.wrapper import control_step

SQRT_3_HALF = 0.5 * math.sqrt(3.0)


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
    for i in range(len(env.dyn_obstacles)):
        body_id = env.sim.model.body_name2id(
            env.dyn_obstacle_names[i]
        )
        pos = env.sim.data.body_xpos[body_id]
        rot = env.sim.data.body_xquat[body_id]
        size = env.dyn_obstacles[i][7:10]
        env.obstacle_capsules[i] = cuboid_to_capsule(pos, rot, size)

    return env.obstacle_capsules


def apf_set_action(env, rl_action):
    # Extract goal position
    current_pos = env.sim.data.get_body_xpos('eef')
    env.rl_goal_pos[:] = current_pos[:3] + rl_action[:3]
    if env.block_z and env.rl_goal_pos[2] > env.block_max_z:
        env.rl_goal_pos[2] = env.block_max_z

    # Get target orientation
    if env.block_orientation:
        env.rl_goal_rot[:] = np.array([1, 0, 0, 0], dtype=np.float64)  # qw, qx, qy, qz
    else:
        env.rl_goal_rot[1:4] += 0.05 * rl_action[3:6]  # qx, qy, qz
        env.rl_goal_rot[0] = math.sqrt(1.0 - np.sum(np.square(env.rl_goal_rot[1:4])))

    # Current joint configuration
    theta = get_theta(env.sim)

    # Obstacle capsules
    obstacle_attributes = get_capsules(env)
    obstacle_attributes[:, 0:3] -= env.robot_offset
    obstacle_attributes[:, 3:6] -= env.robot_offset

    # Compute joint torques
    env.rl_goal_pos -= env.robot_offset
    dt = env.sim.model.opt.timestep
    torques = control_step(theta, env.rl_goal_pos, env.rl_goal_rot, obstacle_attributes, dt, env.pid_integral,
                           env.pid_prev_error)

    # Normalize torques to [-1, 1]
    env.direct_action[:7] = torques / env.sim.model.actuator_forcerange[:7, 1]

    # Directly use RL gripper action
    gripper_ctrl = -0.8 if env.block_gripper else rl_action[7]
    env.direct_action[7] = gripper_ctrl
    env.direct_action[8] = gripper_ctrl

    # Skip the usual mocap control and use direct torque control
    direct_set_action(env.sim, env.direct_action)
