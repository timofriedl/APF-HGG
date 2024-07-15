import copy
from typing import List

import numpy as np
import torch

from apf.wrapper import control_step
from env_ext.fetch import MPCControlGoalEnv
from policies.policy import Policy
from policies.rl_policy import RLPolicy
import time

step = 0
durations = []


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

    def set_envs(self, envs: List[MPCControlGoalEnv]):
        super().set_envs(envs)
        self.rl_policy.set_envs(envs)

        for env in envs:
            env.disable_action_limit()
            self.dt = env.sim.model.opt.timestep

    def initial_info(self, obs: Vector) -> InfoVector:
        pass

    def reset(self):
        self.rl_policy.reset()
        self.step = 0

    def predict(self, obs: Vector) -> (Vector, InfoVector):
        if self.step % 10 == 0:  # RL policy is slow, only execute it once every few time steps
            [self.rl_action], _ = self.rl_policy.predict(obs)

        self.step += 1

        rl_goal_pos = self.rl_action[:3]
        [qw, qx, qy, qz] = [0, 1, 0, 0]  # rl_action[3:7]
        rl_goal_rot = np.array([qx, qy, qz, qw], dtype=np.float32)
        theta = obs[0]["observation"][7:14]

        obstacle_attributes = np.array([], dtype=np.float64)  # TODO add obstacles

        rl_goal_pos -= np.array([0.8, 0.75, 0.42], dtype=np.float32)  # Robot base frame offset
        forces = control_step(theta, rl_goal_pos, rl_goal_rot, obstacle_attributes, self.dt)

        max_forces = self.envs[0].sim.model.actuator_forcerange[:7, 1]
        self.action[:7] = forces / max_forces  # Normalize forces to [-1, 1]
        self.action[7] = self.rl_action[7]  # Directly use RL gripper action
        return [self.action], None
