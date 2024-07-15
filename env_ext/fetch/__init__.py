from .col_test import ColTestGoalEnv
from .fixobj import FixedObjectGoalEnv
from .ghgg_custom import GHGGCustomGoalEnv
from .interval import IntervalGoalEnv
from .mpc_control import MPCControlGoalEnv
from .vanilla import VanillaGoalEnv
from .apf_control import APFControlGoalEnv


def make_env(args):
    return {
        'vanilla': VanillaGoalEnv,
        'fixobj': FixedObjectGoalEnv,
        'interval': IntervalGoalEnv,
        'mpc': MPCControlGoalEnv,
        'apf': APFControlGoalEnv,
        'col_test': ColTestGoalEnv,
        'ghgg_custom': GHGGCustomGoalEnv
    }[args.goal](args)
