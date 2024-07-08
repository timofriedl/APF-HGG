import mppi.pick_3d_target_obstacle
import mppi.pick_dyn_door_obstacles
import mppi.pick_dyn_lifted_obstacles
import mppi.pick_dyn_obstacles
import mppi.pick_dyn_sqr_obstacles
import mppi.pick_static_sqr_obstacles


def get_mppi_parameters(args):
    if args.env == 'FetchPickStaticSqrObstacle':
        return pick_static_sqr_obstacles.get_parameters(args)
    elif args.env == 'FetchPickDynSqrObstacle-v1':
        return pick_dyn_sqr_obstacles.get_parameters(args)
    elif args.env == 'FetchPickDynObstaclesEnv-v1':
        # return pick_dyn_sqr_obstacles.get_parameters(args)
        return pick_dyn_obstacles.get_parameters(args)
    elif args.env == 'FetchPickDynObstaclesEnv-v2':
        return pick_dyn_obstacles.get_parameters(args)
    elif args.env == 'FetchPickDynLiftedObstaclesEnv-v1':
        return pick_dyn_lifted_obstacles.get_parameters(args)
    elif args.env == 'FetchPickDynObstaclesMaxEnv-v1':
        pass
    elif args.env == 'FrankaFetchPickDynSqrObstacle-v1':
        pass
    elif args.env == 'FetchPickDynDoorObstaclesEnv-v1':
        return pick_dyn_door_obstacles.get_parameters(args)
    elif args.env == 'FrankaFetchPickDynDoorObstaclesEnv-v1':
        return pick_dyn_door_obstacles.get_parameters(args)
    elif args.env == 'FrankaFetchPickDynLiftedObstaclesEnv-v1':
        return pick_dyn_lifted_obstacles.get_parameters(args)
    elif args.env == 'FrankaFetchPickDynObstaclesEnv-v1':
        return pick_dyn_obstacles.get_parameters(args)
    elif args.env == 'FrankaFetchPick3DTargetObstacle-v1':
        return pick_3d_target_obstacle.get_parameters(args)
    else:
        # TODO throw some form of error
        pass
