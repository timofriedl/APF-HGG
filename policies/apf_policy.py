from typing import List

from env_ext.fetch import APFControlGoalEnv
from policies.policy import Policy
from policies.rl_policy import RLPolicy


class APFPolicy(Policy):
    Vector = Policy.Vector
    InfoVector = Policy.InfoVector
    visualize = False

    def __init__(self, args):
        self.rl_policy = RLPolicy(args)

    def set_envs(self, envs: List[APFControlGoalEnv]):
        super().set_envs(envs)
        self.rl_policy.set_envs(envs)

        for env in envs:
            env.disable_action_limit()

    def initial_info(self, obs: Vector) -> InfoVector:
        pass

    def reset(self):
        self.rl_policy.reset()
        self.step = 0

    def predict(self, obs: Vector) -> (Vector, InfoVector):
        [rl_action], _ = self.rl_policy.predict(obs)
        return [rl_action], _
