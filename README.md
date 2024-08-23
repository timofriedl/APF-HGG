# Dynamic Obstacle Avoidance Using Artificial Potential Field in Reinforcement Learning

## Description

APF-HGG is based on the repository of [MPPI-HGG](https://github.com/phinners/HGG-MPPI) by Patrick Hinners.
Instead of using Model Predictive Path Integral (MPPI) Control to avoid dynamic obstacles, an Artificial Potential Field (APF) approach is used.
APF calculates magnet-like forces that push the robot away from close obstacles. The task-space forces are converted to joint-space torques, added to the PID output and directly applied to the robot.
You can find the C++ implementation of the APF PID controller in `./apf`.


## Usage

Play video using
```bash
python play.py --env FrankaDirectFetchPickDynLiftedObstaclesEnv-v1 --play_path log/ddpg2-FrankaDirectFetchPickDynLiftedObstaclesEnv-v1-hgg/ --play_epoch 19 --goal apf --play_policy APFPolicy --timesteps 5000 --env_n_substeps 1
```

Note that APF environments run at 1000Hz, while other environments are usually set to 500Hz. Therefore, `--timesteps 5000 --env_n_substeps 1` corresponds to 5000 * 1 / 1000Hz = 5s simulation time per rollout. Substeps must be 1 due to real time direct torque control.