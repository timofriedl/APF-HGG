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
        desired_goal = np.array([0.5, 0, 0.3], dtype=np.float32)  # TODO tf obs[0]["desired_goal"]
        ob = obs[0]["observation"]

        theta = ob[7:14]

        obstacle_attributes = np.array([], dtype=np.float64)  # TODO add obstacles
        forces = control_step(theta, desired_goal, obstacle_attributes)

        np.set_printoptions(precision=1)
        print(forces)
        assert not np.any(np.isnan(forces))

        max_forces = self.envs[0].sim.model.actuator_forcerange[:7, 1]

        action[:7] = forces[:7] / max_forces  # Normalize forces to [-1, 1]
        action[7] = rl_action[7]  # Directly use RL gripper action

        return [action], _
