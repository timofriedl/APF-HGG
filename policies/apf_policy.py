from typing import List

import numpy as np

from apf.wrapper import control_step
from env_ext.fetch import APFControlGoalEnv
from policies.policy import Policy
from policies.rl_policy import RLPolicy


class APFPolicy(Policy):
    Vector = Policy.Vector
    InfoVector = Policy.InfoVector
    visualize = False

    def __init__(self, args):
        self.rl_policy = RLPolicy(args)
        self.step = 0
        self.rl_action = None
        self.action = np.zeros(8, dtype=np.float32)
        self.dt = 0.0
        self.env = None

    def set_envs(self, envs: List[APFControlGoalEnv]):
        super().set_envs(envs)
        self.rl_policy.set_envs(envs)

        assert len(envs) == 1
        envs[0].disable_action_limit()
        self.dt = envs[0].sim.model.opt.timestep
        self.env = envs[0].env

    def initial_info(self, obs: Vector) -> InfoVector:
        pass

    def reset(self):
        self.rl_policy.reset()
        self.step = 0

    def predict(self, obs: Vector) -> (Vector, InfoVector):
        # RL policy is slow, only execute it at 100Hz = 1000Hz / 10
        if self.step % 10 == 0:
            [self.rl_action], _ = self.rl_policy.predict(obs)

        self.step += 1

        # Extract goal position
        rl_goal_pos = self.rl_action[:3]
        if self.env.block_z and rl_goal_pos[2] > self.env.block_max_z:
            rl_goal_pos[2] = self.env.block_max_z

        # Get target orientation
        [qw, qx, qy, qz] = [0, 1, 0, 0] if self.env.block_orientation else self.rl_action[3:7]
        rl_goal_rot = np.array([qx, qy, qz, qw], dtype=np.float32)
        theta = obs[0]["observation"][7:14]
        obstacle_attributes = obs[0]["capsules"]

        # Compute joint torques
        rl_goal_pos -= np.array([0.8, 0.75, 0.44], dtype=np.float32)  # Robot base frame offset
        torques = control_step(theta, rl_goal_pos, rl_goal_rot, obstacle_attributes, self.dt)

        # Normalize torques to [-1, 1]
        self.action[:7] = torques / self.envs[0].sim.model.actuator_forcerange[:7, 1]

        # Directly use RL gripper action
        self.action[7] = self.rl_action[7]
        return [self.action], None
