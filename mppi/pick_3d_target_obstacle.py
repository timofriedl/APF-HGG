import math
import os

import pytorch_kinematics
import torch
from pytorch3d.transforms import quaternion_to_matrix, Transform3d

import mppi_constants

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
        args.σ = 0.201  # 0.01  # 08  # 0.25  # 4.0505  # 10.52e1
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
    T_system = 0.01

    dtype = torch.double
    device = args.device

    α = args.α
    λ = args.λ
    Σ = args.σ * torch.tensor([
        [1.5, args.χ, args.χ, args.χ, args.χ, args.χ, args.χ],
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

    link_dimensions = {
        'panda_link0': torch.tensor([0.0, 0.0, 0.333], dtype=dtype_kinematics),
        # 'panda_link1': torch.tensor([0.0, 0.0, 0.000], dtype=dtype_kinematics),
        # Delete from Calculation for Computational Speed
        'panda_link2': torch.tensor([0.0, -0.316, 0.0], dtype=dtype_kinematics),
        'panda_link3': torch.tensor([0.0825, 0.0, 0.0], dtype=dtype_kinematics),
        'panda_link4': torch.tensor([-0.0825, 0.384, 0.0], dtype=dtype_kinematics),
        # 'panda_link5': torch.tensor([0.0, 0.0, 0.0], dtype=dtype_kinematics),
        # Delete from Calculation for Computational Speed
        'panda_link6': torch.tensor([0.088, 0.0, 0.0], dtype=dtype_kinematics),
        'panda_link7': torch.tensor([0.0, 0.0, 0.245], dtype=dtype_kinematics)
        # 'panda_link8': torch.tensor([0.0, 0.0, 0.0], dtype=dtype_kinematics)
        # Delete from Calculation for Computational Speed
    }

    def calculate_link_verticies(links):
        link_verticies = {}
        for link_key in links:
            link = links[link_key]
            length = torch.norm(link)  # Calculate Length of Link in all Dimensions
            points_distance = 0.03
            points_count = math.ceil(length / points_distance)
            points = torch.zeros((points_count, 3))
            for i in range(points_count):
                points[i, :] = torch.tensor([(link[0] / points_count) * i,
                                             (link[1] / points_count) * i,
                                             (link[2] / points_count) * i], dtype=torch.float64)
            link_verticies[link_key] = points
        return link_verticies

    link_verticies = calculate_link_verticies(link_dimensions)

    def dynamics(x, u):

        new_vel = x[:, 7:14] + u
        new_pos = x[:, 0:7] + new_vel * Δt

        return torch.cat((new_pos, new_vel), dim=1)

    def joint_collision_calculation(link_transform, link_verticies, obstacle_pos, obstacle_rot, obstacle_dim):

        transformed_link = link_transform.transform_points(link_verticies.to(torch.float64))
        transformed_link += robot_base_pos  # Translate Link into World Coordinates

        # Translate into Obstacle Coordinate System
        transformed_link_translated = transformed_link - obstacle_pos

        obstacle_rot_quaternion = torch.roll(obstacle_rot, -1)
        transform_obstacle = Transform3d().rotate(quaternion_to_matrix(obstacle_rot_quaternion)).to(
            device=args.device,
            dtype=torch.float64)
        # Calculate Transformed State to get the correct result
        transformed_points = transform_obstacle.transform_points(transformed_link_translated)

        closest_point = torch.clamp(transformed_points, min=-obstacle_dim, max=obstacle_dim)
        dist = torch.norm(transformed_points - closest_point, dim=2)

        collision_link_segments = torch.le(dist, torch.tensor(
            [0.12]))  # 0.1 Freespace From Obstacle to Center of Link --> Link Dimensions included here!

        collision = torch.any(collision_link_segments, dim=1)

        return collision

    def state_cost(x, goal, obstacles):
        global collision

        joint_values = x[:, 0:7]
        ret = chain.forward_kinematics(joint_values, end_only=False)

        link8_matrix = ret['panda_link8'].get_matrix()  # Equals to panda0_gripper Bode in Mujoco File
        link8_pos = link8_matrix[:, :3, 3] + robot_base_pos  # Equals to panda0_gripper Bode in Mujoco File

        dist_robot_base = torch.norm(link8_pos - robot_base_pos, dim=1)

        goal_dist = torch.norm((link8_pos - goal), dim=1)
        cost = 1000 * goal_dist ** 2
        # cost -= args.ω1 * torch.norm(x[:, 3:6], dim=1) ** 2

        # Joint Collision Avoidance
        for link_key in link_verticies:
            link_vertics = link_verticies[link_key]
            link_transform = ret[link_key]

            link_collisions = joint_collision_calculation(link_transform, link_vertics,
                                                          torch.tensor(obstacles[0:3], device=device),
                                                          torch.tensor(obstacles[3:7], device=device),
                                                          torch.tensor(obstacles[7:10], device=device))

            collision = torch.logical_or(collision,
                                         link_collisions)

        if torch.any(collision):
            # print("Trajectorie with collision detected!")
            pass

        if torch.all(collision):
            # print("All Trajectorie with collision detected!")
            pass

        table_collision = torch.le(link8_pos[:, 2], 0.45)
        workspace_costs = torch.ge(dist_robot_base, 0.8)

        cost += args.ω2 * collision
        cost += args.ω3 * table_collision
        cost += args.ω4 * workspace_costs
        collision = torch.zeros([len(collision), ], dtype=torch.bool)
        return cost

    def terminal_cost(x, goal):
        global collision
        joint_values = x[:, 0:7]
        ret = chain.forward_kinematics(joint_values, end_only=True)

        eef_pos = ret.get_matrix()[:, :3, 3] + robot_base_pos
        cost = 10 * torch.norm((eef_pos - goal), dim=1) ** 2
        # cost += args.ω_Φ * torch.norm(x[:, 3:6], dim=1) ** 2
        collision = torch.zeros([len(collision), ], dtype=torch.bool)
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
        # return u

    return K, T, Δt, α, dynamics, state_cost, terminal_cost, Σ, λ, convert_to_target, dtype, device
