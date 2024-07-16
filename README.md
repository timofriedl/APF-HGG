# Dynamic Obstacle Avoidance Using Artificial Potential Field in Reinforcement Learning

## Description

APF-HGG is based on the repository of [MPPI-HGG](https://github.com/phinners/HGG-MPPI) by Patrick Hinners.
Instead of using Model Predictive Path Integral (MPPI) Control to avoid dynamic obstacles, an Artificial Potential Field (APF) approach is used.
You can find the C++ implementation of the APF PID controller in `./apf`.


## Usage

Train using
```bash
python train2.py --alg ddpg2 --epochs 5 --env FrankaDirectFetchPickDynObstaclesEnv-v1 --reward_min -10 --goal apf --timesteps 500 --env_n_substeps 10
```

Play video using
```bash
python play.py --env FrankaDirectFetchPickDynObstaclesEnv-v1 --play_path log/ddpg2-FrankaDirectFetchPickDynObstaclesEnv-v1-hgg/ --play_epoch 19 --goal apf --play_policy APFPolicy --timesteps 1000 --env_n_substeps 10
```