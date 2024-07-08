from .policy import Policy
from .rl_policy import RLPolicy
from .mpc_policy import MPCPolicy
from .mpc_rl_policy import MPCRLPolicy
from .mppi_policy import MPPIPolicy
from .mppi_rl_policy import MPPIRLPolicy


def make_policy(args):
    return {
        'MPPIRLPolicy': MPPIRLPolicy,
        'MPPIPolicy': MPPIPolicy,
        'MPCRLPolicy': MPCRLPolicy,
        'MPCPolicy': MPCPolicy,
        'RLPolicy': RLPolicy,
    }[args.play_policy](args)
