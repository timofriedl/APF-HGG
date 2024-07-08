from .franka_mpc_policy import MPCPolicy
from .franka_mpc_rl_policy import MPCRLPolicy
from .franka_mppi_policy import MPPIPolicy
from .franka_mppi_rl_policy import MPPIRLPolicy
from .franka_policy import Policy
from .franka_rl_policy import RLPolicy


def make_policy(args):
    return {
        'MPPIRLPolicy': MPPIRLPolicy,
        'MPPIPolicy': MPPIPolicy,
        'MPCRLPolicy': MPCRLPolicy,
        'MPCPolicy': MPCPolicy,
        'RLPolicy': RLPolicy,
    }[args.play_policy](args)
