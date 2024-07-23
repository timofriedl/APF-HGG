import env_ext.fetch as fetch_env
from .utils import goal_distance, goal_distance_obs

Robotics_envs_id = [
    'FetchPickStaticSqrObstacle',
    'FetchPickDynSqrObstacle-v1',
    # 'FetchPickDynLabyrinthEnv-v1', 	# was easy for the agent and not symmetric
    'FetchPickDynObstaclesEnv-v1',
    'FetchPickDynObstaclesEnv-v2',
    'FetchPickDynLiftedObstaclesEnv-v1',
    'FetchPickDynObstaclesMaxEnv-v1',
    'FrankaFetchPickDynSqrObstacle-v1',
    'FetchPickDynDoorObstaclesEnv-v1',
    'FrankaFetchPickDynDoorObstaclesEnv-v1',
    'FrankaFetchPickDynLiftedObstaclesEnv-v1',
    'FrankaFetchPickDynObstaclesEnv-v1',
    'FrankaFetchPick3DTarget-v1',
    'FrankaFetchPick3DTargetObstacle-v1',
    'FrankaDirectFetchPickDynLiftedObstaclesEnv-v1',
    'FrankaDirectFetchPick3DTargetObstacle-v1',
    'FrankaDirectFetchPick3DTarget-v1'
]


def make_env(args):
    assert args.env in Robotics_envs_id
    if args.env[:5] == 'Fetch':
        return fetch_env.make_env(args)
    if args.env[:11] == 'FrankaFetch':
        return fetch_env.make_env(args)
    if args.env[:17] == 'FrankaDirectFetch':
        return fetch_env.make_env(args)
    else:
        return None


def clip_return_range(args):
    gamma_sum_min = args.reward_min / (1.0 - args.gamma)
    gamma_sum_max = args.reward_max / (1.0 - args.gamma)
    return {
        'FetchPickStaticSqrObstacle': (gamma_sum_min, gamma_sum_max),
        'FetchPickDynSqrObstacle-v1': (gamma_sum_min, gamma_sum_max),
        'FetchPickDynLabyrinthEnv-v1': (gamma_sum_min, gamma_sum_max),
        'FetchPickDynObstaclesEnv-v1': (gamma_sum_min, gamma_sum_max),
        'FetchPickDynObstaclesEnv-v2': (gamma_sum_min, gamma_sum_max),
        'FetchPickDynLiftedObstaclesEnv-v1': (gamma_sum_min, gamma_sum_max),
        'FetchPickDynObstaclesMaxEnv-v1': (gamma_sum_min, gamma_sum_max),
        'FrankaFetchPickDynSqrObstacle-v1': (gamma_sum_min, gamma_sum_max),
        'FetchPickDynDoorObstaclesEnv-v1': (gamma_sum_min, gamma_sum_max),
        'FrankaFetchPickDynDoorObstaclesEnv-v1': (gamma_sum_min, gamma_sum_max),
        'FrankaFetchPickDynLiftedObstaclesEnv-v1': (gamma_sum_min, gamma_sum_max),
        'FrankaFetchPickDynObstaclesEnv-v1': (gamma_sum_min, gamma_sum_max),
        'FrankaFetchPick3DTarget-v1': (gamma_sum_min, gamma_sum_max),
        'FrankaFetchPick3DTargetObstacle-v1': (gamma_sum_min, gamma_sum_max),
        'FrankaDirectFetchPickDynLiftedObstaclesEnv-v1': (gamma_sum_min, gamma_sum_max),
        'FrankaDirectFetchPick3DTargetObstacle-v1': (gamma_sum_min, gamma_sum_max),
        'FrankaDirectFetchPick3DTarget-v1': (gamma_sum_min, gamma_sum_max)
    }[args.env]
