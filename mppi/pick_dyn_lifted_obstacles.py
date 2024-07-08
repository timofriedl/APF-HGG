import os

import numpy as np
import pytorch_kinematics
import torch

import mppi_constants
from fast_fk.wrapper import forward_kinematics

collision = None


def update_collision(mppi_k, device):
    global collision
    collision = torch.zeros([mppi_k, ], dtype=torch.bool, device=device)


mppi_constants.register_k_consumer(update_collision)


def getCollisions():
    return collision


def get_parameters(args):
    if args.tune_mppi <= 0:
        args.α = 0  # 5.94e-1
        args.λ = 60  # 40  # 1.62e1
        args.σ = 0.20  # 0.01  # 08  # 0.25  # 4.0505  # 10.52e1
        args.χ = 0.0  # 2.00e-2
        args.ω1 = 1.0003
        args.ω2 = 9.16e3
        args.ω3 = 9.16e3
        args.ω4 = 9.16e3
        args.ω_Φ = 5.41
        args.d_goal = 0.15

    K = args.mppi_k
    T = args.mppi_t
    Δt = 0.01
    T_system = 0.011

    dtype = torch.double
    device = args.device

    α = args.α
    λ = args.λ
    Σ = args.σ * torch.tensor([
        [2.5, args.χ, args.χ, args.χ, args.χ, args.χ, args.χ],
        [args.χ, 0.75, args.χ, args.χ, args.χ, args.χ, args.χ],
        [args.χ, args.χ, 1.0, args.χ, args.χ, args.χ, args.χ],
        [args.χ, args.χ, args.χ, 1.25, args.χ, args.χ, args.χ],
        [args.χ, args.χ, args.χ, args.χ, 1.50, args.χ, args.χ],
        [args.χ, args.χ, args.χ, args.χ, args.χ, 2.00, args.χ],
        [args.χ, args.χ, args.χ, args.χ, args.χ, args.χ, 2.00]
    ], dtype=dtype, device=device)

    # Ensure we get the path separator correct on windows
    MODEL_URDF_PATH = os.path.join(os.getcwd(), 'envs', 'assets', 'fetch', 'franka_panda_arm.urdf')

    xml = bytes(bytearray(open(MODEL_URDF_PATH).read(), encoding='utf-8'))
    dtype_kinematics = torch.double
    chain = pytorch_kinematics.build_serial_chain_from_urdf(xml, end_link_name="panda_link8",
                                                            root_link_name="panda_link0")
    chain = chain.to(dtype=dtype_kinematics, device=device)

    # Translational offset of Robot into World Coordinates
    robot_base_pos = torch.tensor([0.8, 0.75, 0.44],
                                  device=device, dtype=dtype_kinematics)

    def dynamics(x, u):
        new_vel = x[:, 7:14] + u
        new_pos = x[:, 0:7] + new_vel * Δt

        return torch.cat((new_pos, new_vel), dim=1)

    def state_cost(x: torch.Tensor, goal: torch.Tensor, obstacles: torch.Tensor, fast_fk=True):
        global collision

        joint_values = x[:, 0:7]

        if not fast_fk:
            # Old slow method
            ret = chain.forward_kinematics(joint_values, end_only=False)
            link8_matrix = ret['panda_link8'].get_matrix()  # Equals to panda0_gripper Bode in Mujoco File
        else:
            # Use C++ forward kinematics
            q = np.array(joint_values.cpu(), dtype=np.float32)
            link8_matrix = forward_kinematics(q)

            link8_matrix = torch.tensor(link8_matrix, dtype=torch.float32, device=device)

        link8_pos = link8_matrix[:, :3, 3] + robot_base_pos  # Equals to panda0_gripper Bode in Mujoco File

        dist_robot_base = torch.norm(link8_pos - robot_base_pos, dim=1)

        goal_dist = torch.norm((link8_pos - goal), dim=1)

        cost = 1000 * goal_dist ** 2

        # Collision Detection with EEF
        obstacle_0_3 = obstacles[0:3].clone()
        obstacle_7_10 = obstacles[7:10].clone()
        obstacle_10_13 = obstacles[10:13].clone()
        obstacle_17_20 = obstacles[17:20].clone()
        obstacle_20_23 = obstacles[20:23].clone()
        obstacle_27_30 = obstacles[27:30].clone()

        dist1 = torch.abs(link8_pos - obstacle_0_3)
        dist2 = torch.abs(link8_pos - obstacle_10_13)
        dist3 = torch.abs(link8_pos - obstacle_20_23)

        padding = torch.tensor([0.055, 0.055, 0.03], device=device)

        collision = torch.logical_or(collision, torch.all(torch.le(dist1, obstacle_7_10 + padding), dim=1))
        collision = torch.logical_or(collision, torch.all(torch.le(dist2, obstacle_17_20 + padding), dim=1))
        collision = torch.logical_or(collision, torch.all(torch.le(dist3, obstacle_27_30 + padding), dim=1))

        # Collision Detection with Hand
        hand = link8_pos.clone()
        hand[:, 2] += 0.11

        dist4 = torch.abs(hand[:, 0:3] - obstacle_0_3)
        dist5 = torch.abs(hand[:, 0:3] - obstacle_10_13)
        dist6 = torch.abs(hand[:, 0:3] - obstacle_20_23)

        padding1 = torch.tensor([0.030, 0.08, 0.05], device=device)
        padding2 = torch.tensor([0.030, 0.11, 0.05], device=device)
        padding3 = torch.tensor([0.030, 0.08, 0.05], device=device)

        collision = torch.logical_or(collision, torch.all(torch.le(dist4, obstacle_7_10 + padding1), dim=1))
        collision = torch.logical_or(collision, torch.all(torch.le(dist5, obstacle_17_20 + padding2), dim=1))
        collision = torch.logical_or(collision, torch.all(torch.le(dist6, obstacle_27_30 + padding3), dim=1))

        workspace_costs = torch.ge(dist_robot_base, 0.8)

        cost += args.ω2 * collision
        cost += args.ω4 * workspace_costs
        return cost

    def terminal_cost(x, goal):
        global collision
        joint_values = x[:, 0:7]
        ret = chain.forward_kinematics(joint_values, end_only=True)

        eef_pos = ret.get_matrix()[:, :3, 3] + robot_base_pos
        cost = 10 * torch.norm((eef_pos - goal), dim=1) ** 2
        # cost += args.ω_Φ * torch.norm(x[:, 3:6], dim=1) ** 2
        collision = torch.zeros([len(collision), ], dtype=torch.bool, device=device)
        return cost

    def convert_to_target(x, u):
        joint_pos = x[0:7]
        joint_vel = x[7:14]
        new_vel = joint_vel + u / (
                1 - torch.exp(torch.tensor(-Δt / T_system)) * (
                1 + (Δt / T_system)))  # / (1 - torch.exp(torch.tensor(-0.01 / 0.150)))  # 0.175

        new_joint_pos = joint_pos + new_vel * Δt  # Calculate new Target Joint Positions

        # Calculate World Coordinate Target Position
        ret = chain.forward_kinematics(new_joint_pos, end_only=True)
        eef_matrix = ret.get_matrix()
        eef_pos = eef_matrix[:, :3, 3] + robot_base_pos  # Calculate World Coordinate Target
        eef_rot = pytorch_kinematics.matrix_to_quaternion(eef_matrix[:, :3, :3])

        return torch.cat((eef_pos, eef_rot), dim=1)

    return K, T, Δt, α, dynamics, state_cost, terminal_cost, Σ, λ, convert_to_target, dtype, device
