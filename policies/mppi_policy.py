import time
from typing import List

import numpy as np
import scipy.signal
import torch
from scipy.spatial.transform import Rotation

import mppi
from env_ext.fetch import MPCControlGoalEnv
from mppi.pick_dyn_door_obstacles import getCollisions
from policies.policy import Policy


# TODO get this working for better performance than scipy
def savgol_filter_2d(tensor, window_length, polyorder, axis=0, deriv=0, delta=1.0):
    pass


class MPPIPolicy(Policy):
    Vector = Policy.Vector
    InfoVector = Policy.InfoVector
    LastPosition = np.zeros((3,))
    grip_action = -0.8

    def __init__(self, args):
        (
            self.K,
            self.T,
            self.Δt,
            self.α,
            self.F,
            self.q,
            self.ϕ,
            self.Σ,
            self.λ,
            self.convert_to_target,
            self.dtype,
            self.device
        ) = mppi.get_mppi_parameters(args)
        # noise_mean_distribution = torch.zeros(3, dtype=self.dtype, device=self.device)
        noise_mean_distribution = torch.zeros(7, dtype=self.dtype, device=self.device)
        self.noise_distribution = torch.distributions.multivariate_normal.MultivariateNormal(
            loc=noise_mean_distribution, covariance_matrix=self.Σ)
        self.γ = self.λ * (1 - self.α)
        self.Σ_inv = torch.inverse(self.Σ)

        self.u = torch.zeros((self.T, 7), dtype=self.dtype, device=self.device)
        self.u_init = torch.zeros(7, dtype=self.dtype, device=self.device)

        self.prev_u = []

        self.collisions = []
        self.trajectory = None

    def initial_info(self, obs: Vector) -> InfoVector:
        pass

    def reset(self):
        self.u = torch.zeros((self.T, 7), dtype=self.dtype, device=self.device)
        self.LastPosition = np.zeros((3,))

    def predict(self, obs: Vector) -> (Vector, InfoVector):
        return self.predict_with_goal(obs, obs[0]['desired_goal'])

    def set_envs(self, envs: List[MPCControlGoalEnv]):
        super().set_envs(envs)
        for env in envs:
            env.disable_action_limit()

    def predict_with_goal(self, obs: Vector, goal) -> (Vector, InfoVector):
        # x_init, obstacle_positions = self.parse_observation(obs[0], goal)
        x_init, joint_init, obstacle_positions = self.parse_observation(obs[0], goal)
        goal = torch.tensor(goal, device=self.device)

        # shift the control inputs
        self.u = torch.roll(self.u, -1, dims=0)
        self.u[-1] = self.u_init

        self.update_control(joint_init, goal, obstacle_positions)
        target = self.convert_to_target(joint_init, self.u[0])

        action = (
                    target - x_init).cpu().numpy()  # Action is the difference between current Position and Target Position

        # Calculate Real Target is Used for Vizualizing the desired Position in the next Timestep for Visualization
        self.target = self.calculate_real_target(joint_init, self.u[0])

        return [np.append(action, self.grip_action)], [{'direction': 'forward'}]

    def calculate_real_target(self, x, u):
        import os
        import pytorch_kinematics

        MODEL_URDF_PATH = os.path.join(os.getcwd(), 'envs', 'assets', 'fetch', 'franka_panda_arm.urdf')

        xml = bytes(bytearray(open(MODEL_URDF_PATH).read(), encoding='utf-8'))
        dtype = torch.double
        chain = pytorch_kinematics.build_serial_chain_from_urdf(xml, end_link_name="panda_link8",
                                                                root_link_name="panda_link0")
        chain = chain.to(dtype=dtype, device=self.device)

        # Translational offset of Robot into World Coordinates
        robot_base_pos = torch.tensor([0.8, 0.75, 0.44], device=self.device)

        joint_pos = x[0:7]
        joint_vel = x[7:14]
        new_vel = joint_vel + u  # / (1 - torch.exp(torch.tensor(-0.01 / 0.100)))  # 0.175

        joint_pos = joint_pos + new_vel * self.Δt  # Calculate new Target Joint Positions
        # Calculate World Coordinate Target Position
        ret = chain.forward_kinematics(joint_pos, end_only=True)
        eef_matrix = ret.get_matrix()
        eef_pos = eef_matrix[:, :3, 3] + robot_base_pos  # Calculate World Coordinate Target
        eef_rot = pytorch_kinematics.matrix_to_quaternion(eef_matrix[:, :3, :3])
        return torch.cat((eef_pos, eef_rot), dim=1)

    def parse_observation(self, obs, goal):
        ob = obs['observation']
        eef_pos = ob[0:3]
        eef_rot = ob[3:7]
        q_pos = ob[7:14]  # Exclusive Finger Joints as they dont matter for MPPI
        q_vel = ob[16:23]  # Exclusive Finger Joints as they dont matter for MPPI
        if self.LastPosition[0] == 0:
            vel = np.zeros(3, )  # ob[20:23]
        else:
            vel = (eef_pos - self.LastPosition) / self.Δt
        self.LastPosition = eef_pos
        x_init = torch.tensor([eef_pos[0], eef_pos[1], eef_pos[2], eef_rot[0], eef_rot[1], eef_rot[2], eef_rot[3]],
                              dtype=self.dtype, device=self.device)
        joint_init = torch.tensor(np.concatenate([q_pos, q_vel]),
                                  dtype=self.dtype, device=self.device)

        parameters = self.envs[0].extract_parameters_3d(self.T, self.Δt, goal)
        parameters = torch.tensor(parameters, device=self.device)

        obstacle_positions = parameters[:, 6:]
        return x_init, joint_init, obstacle_positions

    def update_control(self, state: torch.tensor, goal: torch.tensor, obstacle_positions: torch.tensor):
        x = state.view(1, -1).repeat(self.K, 1)

        trajectory_rollouts = torch.zeros((self.K, state.size(0), self.T + 1), device=self.device, dtype=self.dtype)
        trajectory_rollouts[:, :, 0] = x

        interim_goals = torch.zeros((3, self.T), device=self.device, dtype=self.dtype)

        ε = self.noise_distribution.rsample((self.K, self.T))
        v = ε.clone()
        mask = torch.arange(self.K, device=self.device) < int((1 - self.α) * self.K)
        v[mask] += self.u
        S = torch.zeros(self.K, dtype=self.dtype, device=self.device)
        self.collisions = []
        self.dist = []

        for i in range(self.T):
            x = self.F(x, v[:, i])
            if self.trajectory is not None:
                interim_goal = torch.tensor(self.trajectory[i], device=self.device)
            else:
                interim_goal = goal

            interim_goals[:, i] = interim_goal
            trajectory_rollouts[:, :, i + 1] = x
            S += self.q(x, interim_goal, obstacle_positions[i])
            S += v[:, i] @ self.Σ_inv @ self.u[i]
            self.collisions.append(getCollisions())

        S += self.ϕ(x, goal)
        β = torch.min(S)
        ω = torch.exp((β - S) / self.λ)
        η = torch.sum(ω)
        ω /= η
        δu = torch.sum(ω.view(-1, 1, 1) * ε, dim=0)
        self.u += δu
        self.u = torch.tensor(
            scipy.signal.savgol_filter(self.u.cpu(), window_length=self.T // 2 * 2 - 1, polyorder=1, axis=0),
            dtype=self.dtype, device=self.device
        )
        self.trajectory_rollouts = trajectory_rollouts
        self.obstacle_positions = obstacle_positions
        self.interim_goals = interim_goals

    def is_goal_reachable(self, goal, obstacles):
        num_obstacles = len(obstacles) // 10
        for i in range(num_obstacles):
            obstacle_start = i * 10
            obstacle_end = obstacle_start + 10
            obstacle = obstacles[obstacle_start:obstacle_end]
            obstacle_position = obstacle[0:3]
            obstacle_rotation = obstacle[3:7]
            obstacle_dimensions = obstacle[7:10]
            endeffector_dimensions = np.array([0.04, 0.04, 0.03])

            obstacle_min = torch.tensor(obstacle_position - (obstacle_dimensions + endeffector_dimensions),
                                        dtype=self.dtype, device=self.device)
            obstacle_max = torch.tensor(obstacle_position + (obstacle_dimensions + endeffector_dimensions),
                                        dtype=self.dtype, device=self.device)

            # Calculate Transformed Goal to get the correct result
            r = Rotation.from_quat([np.roll(obstacle_rotation, -1)])
            goal_transformed = np.matmul(r.as_matrix(), goal.numpy())
            goal_transformed = torch.tensor(goal_transformed)

            if torch.all(torch.logical_and(goal_transformed >= obstacle_min, goal_transformed <= obstacle_max)):
                return False
