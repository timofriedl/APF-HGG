from abc import abstractmethod
from typing import List

import torch


class Policy:
    Vector = List[torch.Tensor]
    InfoVector = List[dict]

    envs = []

    def set_envs(self, envs: List):
        # environments references
        self.envs = envs

    @abstractmethod
    def predict(self, obs: Vector) -> (Vector, InfoVector):
        """Predict the next step"""
        raise NotImplementedError

    @abstractmethod
    def initial_info(self, obs: Vector) -> InfoVector:
        """Get the initial information for debugging"""
        raise NotImplementedError

    @abstractmethod
    def reset(self):
        """Predict the next step"""
        raise NotImplementedError
