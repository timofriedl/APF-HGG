from typing import List

import numpy as np

from policies_real.franka_mpc_policy import MPCPolicy
from policies_real.franka_policy import Policy
from policies_real.franka_rl_policy import RLPolicy


class MPCRLPolicy(Policy):
    Vector = List[torch.Tensor]
    InfoVector = List[dict]

    def __init__(self, args):
        # get current policy from path (restore tf session + graph)
        self.mpc_policy = MPCPolicy(args)
        self.rl_policy = RLPolicy(args)

    # def reset(self):
    #     self.rl_policy.reset()
    #     self.mpc_policy.reset()

    @property
    def model(self):
        return self.mpc_policy.model

    def set_env(self, env):
        super().set_env(env)
        env.disable_action_limit()

    # predicts next actions for given states (observations)
    def predict(self, obs):
        rl_actions = self.rl_policy.predict(obs)
        sub_goal = self.env.subgoal(rl_actions[0], obs[0])
        # print("sub_goal", sub_goal)
        self.mpc_policy.set_sub_goal(sub_goal)

        actions = self.mpc_policy.predict(obs)

        return actions
