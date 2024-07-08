import copy
from typing import List

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
from numpy import linalg

from env_ext.fetch import MPCControlGoalEnv
from policies_real.franka_mppi_policy import MPPIPolicy
from policies_real.franka_policy import Policy
from policies_real.franka_rl_policy import RLPolicy


class MPPIRLPolicy(Policy):
    Vector = Policy.Vector
    InfoVector = Policy.InfoVector
    visualize = False

    def __init__(self, args):
        self.rl_policy = RLPolicy(args)
        self.mppi_policy = MPPIPolicy(args)

    def set_envs(self, envs: List[MPCControlGoalEnv]):
        super().set_envs(envs)
        self.rl_policy.set_envs(envs)
        self.mppi_policy.set_envs(envs)

        for env in envs:
            env.disable_action_limit()

    def initial_info(self, obs: Vector) -> InfoVector:
        pass

    def reset(self):
        self.rl_policy.reset()
        self.mppi_policy.reset()

    def predict(self, obs: Vector) -> (Vector, InfoVector):
        action = np.zeros(8)
        [rl_action], _ = self.rl_policy.predict(obs)
        desired_goal = obs[0]["desired_goal"]
        current_pos = obs[0]['observation'][0:3]
        sub_goal = self.env.subgoal(rl_action[0], obs[0])
        obs_sim = copy.deepcopy(obs)
        self.mppi_policy.trajectory = []

        if (linalg.norm(desired_goal - current_pos) > self.env.args.d_goal):
            for i in range(self.mppi_policy.T):
                [rl_action_rollout], _ = self.rl_policy.predict(obs_sim)
                sub_goal_sim = self.env.subgoal_sim(rl_action_rollout, obs_sim[0]['observation'][0:3])
                obs_sim = self.env.observation_sim(obs_sim, sub_goal_sim)  # Step Virtually the observation forward

                self.mppi_policy.trajectory.append(sub_goal_sim)
            print("HGG Subgoals: " + str(self.mppi_policy.trajectory))
            [mppi_action], _ = self.mppi_policy.predict_with_goal(obs, sub_goal)
        else:
            for i in range(self.mppi_policy.T):
                self.mppi_policy.trajectory.append(desired_goal)
            print("Fixed Subgoal: " + str(desired_goal))
            [mppi_action], _ = self.mppi_policy.predict_with_goal(obs, desired_goal)
        if self.visualize == True:
            self.plot_rollouts(self.mppi_policy.trajectory_rollouts, self.mppi_policy.obstacle_positions, sub_goal,
                               self.mppi_policy.target.numpy(), self.mppi_policy.collisions, current_pos, desired_goal)
        action[0:7] = mppi_action[:7]
        action[7] = -0.8  # rl_action[7]

        return [action], _

    def plot_rollouts(self, trajectories, obstacle_positions, hgg_subgoal, mppi_target, collisions, current_pos,
                      episode_goal):
        import pytorch_kinematics
        import torch

        full_urdf = "/home/patrick/PycharmProjects/InverseKinematicsTest/franka_panda_description/robots/panda_arm.urdf"
        file_read = open(full_urdf).read()
        xml = bytes(bytearray(file_read, encoding='utf-8'))
        dtype = torch.double
        chain = pytorch_kinematics.build_serial_chain_from_urdf(xml, end_link_name="panda_link8",
                                                                root_link_name="panda_link0")

        robot_base_pos = torch.tensor([0.8, 0.75, 0.44])
        chain = chain.to(dtype=dtype)

        plt.ioff()
        collision_rollout = None
        collision_free_rollout = None
        fig = plt.figure(figsize=[6.00, 8.40])
        for i in range(self.mppi_policy.K):
            ret = chain.forward_kinematics(np.swapaxes(trajectories[i, 0:7, :], 0, 1), end_only=True)
            eef_matrix = ret.get_matrix()
            eef_pos = eef_matrix[:, :3, 3] + robot_base_pos
            if (collisions[self.mppi_policy.T - 1].numpy()[i]):
                collision_rollout, = plt.plot(eef_pos[:, 0],
                                              eef_pos[:, 1],
                                              zorder=1,
                                              marker='o',
                                              markersize=1)  # ,
                # color="tab:red")
                pass
            else:
                collision_free_rollout, = plt.plot(eef_pos[:, 0],
                                                   eef_pos[:, 1],
                                                   zorder=1,
                                                   marker='o',
                                                   markersize=1,
                                                   color="tab:green")
        # specify the location of (left,bottom),width,height
        obs_0 = mpatches.Rectangle((obstacle_positions[0, 0] - (obstacle_positions[0, 3]),  # - 0.035,
                                    obstacle_positions[0, 1] - (obstacle_positions[0, 4])),  # - 0.035),
                                   (obstacle_positions[0, 3] * 2),  # + (2 * 0.035),
                                   (obstacle_positions[0, 4] * 2),  # + (2 * 0.035),
                                   fill=False,
                                   color="purple",
                                   linewidth=2,
                                   zorder=2)
        for i in range(self.mppi_policy.T):
            # Calculate Transformed State to get the correct result
            #    r = Rotation.from_quat([np.roll(obstacle_positions[i, 3:7], -1)])
            #    translation = np.array([obstacle_positions[i, 0], obstacle_positions[i, 1], 0])
            #    corner_point = np.array([- obstacle_positions[i, 7], - obstacle_positions[i, 8], 0])
            #    corner_point_rotated = np.matmul(r.as_matrix(), corner_point)
            #    corner_point_translated = corner_point_rotated + translation
            #    obs_0 = mpatches.Rectangle((corner_point_translated[0][0],  # - 0.035,
            #                                corner_point_translated[0][1]),  # - 0.035),
            #                               (obstacle_positions[i, 7] * 2),  # + (2 * 0.035),
            #                               (obstacle_positions[i, 8] * 2),  # + (2 * 0.035),
            #                               fill=False,
            #                               color="purple",
            #                               linewidth=3 - (i * 0.1),
            #                               zorder=2,
            #                               angle=r.as_euler('zyx', degrees=True)[0][0])
            #    obs_0_pos = plt.Circle((obstacle_positions[i, 0], obstacle_positions[i, 1]),
            #                           0.005, color='tab:olive', fill=True, zorder=3)
            obs_1 = mpatches.Rectangle((obstacle_positions[i, 0] - (obstacle_positions[i, 7]),  # - 0.035,
                                        obstacle_positions[i, 1] - (obstacle_positions[i, 8])),  # - 0.035),
                                       (obstacle_positions[i, 7] * 2),  # + (2 * 0.035),
                                       (obstacle_positions[i, 8] * 2),  # + (2 * 0.035),
                                       fill=False,
                                       color="purple",
                                       linewidth=3 - (i * 0.1),
                                       zorder=2)
            obs_2 = mpatches.Rectangle((obstacle_positions[i, 10] - (obstacle_positions[i, 17]),  # - 0.035,
                                        obstacle_positions[i, 11] - (obstacle_positions[i, 18])),  # - 0.035),
                                       (obstacle_positions[i, 17] * 2),  # + (2 * 0.035),
                                       (obstacle_positions[i, 18] * 2),  # + (2 * 0.035),
                                       fill=False,
                                       color="purple",
                                       linewidth=3 - (i * 0.1),
                                       zorder=2)
            #    plt.gca().add_patch(obs_0_pos)
            #    plt.gca().add_patch(obs_0)
            plt.gca().add_patch(obs_1)
            plt.gca().add_patch(obs_2)

        curr_pos = plt.Circle((current_pos[0], current_pos[1]),
                              0.005, color='tab:cyan', fill=True, zorder=3)
        plt.gca().add_patch(curr_pos)

        hgg_trajectory = plt.Circle((self.mppi_policy.interim_goals[0, 0], self.mppi_policy.interim_goals[1, 0]),
                                    0.005, color='tab:blue', fill=True, zorder=3)
        plt.gca().add_patch(hgg_trajectory)
        for i in range(self.mppi_policy.T):
            interimgoal = plt.Circle((self.mppi_policy.interim_goals[0, i], self.mppi_policy.interim_goals[1, i]),
                                     0.005, color='tab:blue', fill=True, zorder=3)
            plt.gca().add_patch(interimgoal)
        # subgoal_hgg = plt.Circle((hgg_subgoal[:1], hgg_subgoal[1:2]), 0.01, color='tab:blue', fill=True, zorder=3)
        target_mppi = None
        target_mppi = plt.Circle((mppi_target[0, 0], mppi_target[0, 1]), 0.005, color='tab:orange',
                                 fill=True,
                                 zorder=3)
        plt.gca().add_patch(target_mppi)
        target_episode = plt.Circle((episode_goal[0], episode_goal[1]), 0.005, color='tab:red',
                                    fill=True,
                                    zorder=3)
        plt.gca().add_patch(target_episode)
        # plt.gca().add_patch(subgoal_hgg)

        if collision_rollout is not None and collision_free_rollout is not None:
            plt.gca().legend(
                [collision_rollout, collision_free_rollout, hgg_trajectory, target_mppi, obs_0, curr_pos,
                 target_episode],
                ['Rollouts w Collision', 'Rollouts w/o Collision', 'HGG-Trajectory', 'Goal MPPI',
                 'Obstacles', 'Current Pos', 'Desired Goal'])
        elif collision_rollout is None and collision_free_rollout is not None:
            plt.gca().legend([collision_free_rollout, hgg_trajectory, target_mppi, obs_0, curr_pos, target_episode],
                             ['Rollouts ', 'HGG-Trajectory', 'Goal MPPI', 'Obstacles', 'Current Pos',
                              'Desired Goal'])
        elif collision_rollout is not None and collision_free_rollout is None:
            plt.gca().legend([collision_rollout, hgg_trajectory, target_mppi, obs_0, curr_pos, target_episode],
                             ['Rollouts w Collision', 'HGG-Trajectory', 'Goal MPPI', 'Obstacles', 'Current Pos',
                              'Desired Goal'])
        elif collision_rollout is None and collision_free_rollout is None:
            plt.gca().legend([hgg_trajectory, target_mppi, obs_0, curr_pos, target_episode],
                             ['HGG-Trajectory', 'Goal MPPI', 'Obstacles', 'Current Pos', 'Desired Goal'])
        plt.title("HGG-MPPI with Top View")
        plt.xlim([1.05, 1.55])
        plt.ylim([0.40, 1.10])
        plt.grid()
        plt.draw()
        plt.pause(0.01)

        pass
