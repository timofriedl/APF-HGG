import copy
from typing import List

import numpy as np
import torch

from apf.wrapper import control_step
from env_ext.fetch import MPCControlGoalEnv
from policies.policy import Policy
from policies.rl_policy import RLPolicy


class APFPolicy(Policy):
    Vector = Policy.Vector
    InfoVector = Policy.InfoVector
    visualize = False

    def __init__(self, args):
        self.rl_policy = RLPolicy(args)

    def set_envs(self, envs: List[MPCControlGoalEnv]):
        super().set_envs(envs)
        self.rl_policy.set_envs(envs)

        for env in envs:
            env.disable_action_limit()

    def initial_info(self, obs: Vector) -> InfoVector:
        pass

    def reset(self):
        self.rl_policy.reset()

    def predict(self, obs: Vector) -> (Vector, InfoVector):
        action = np.zeros(8)
        [rl_action], _ = self.rl_policy.predict(obs)
        desired_goal = obs[0]["desired_goal"]
        ob = obs[0]["observation"]

        q = np.zeros(8)
        q[:7] = ob[7:14]
        # TODO q[8] = ?

        joint_vel = ob[16:23]

        obstacle_attributes = np.array([], dtype=np.float64)  # TODO add obstacles
        forces = control_step(q, desired_goal, 1.0, obstacle_attributes)  # TODO set target_gripper

        # TODO directly apply forces instead of returning [dx, dy, dz, q1, q2, q3, q4]
        # TODO currently is action the demanded positional offset

        action[0:7] = np.zeros(7, dtype=np.float32)
        action[7] = rl_action[7]

        return [action], _
