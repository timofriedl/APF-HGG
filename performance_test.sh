#!/bin/bash

# Set environment variables
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/tf/.mujoco/mujoco210/bin:/usr/lib/nvidia
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6:/usr/lib/x86_64-linux-gnu/libGLEW.so
export PYTHONPATH=/home/tf/forces_pro_client/

# Activate the virtual environment
source ~/.pyenv/versions/hgg-mppi/bin/activate

# Execute the script with the given arguments
python test_policy.py --env FrankaFetchPickDynLiftedObstaclesEnv-v1 --play_path "log/ddpg2-FrankaFetchPickDynLiftedObstaclesEnv-v1-hgg/" --play_epoch best --goal mpc --play_policy MPPIRLPolicy --timesteps 1000 --env_n_substeps 5 --episodes 200 --mppi_k 50 --mppi_t 10
python test_policy.py --env FrankaFetchPickDynLiftedObstaclesEnv-v1 --play_path "log/ddpg2-FrankaFetchPickDynLiftedObstaclesEnv-v1-hgg/" --play_epoch best --goal mpc --play_policy MPPIRLPolicy --timesteps 1000 --env_n_substeps 5 --episodes 200 --mppi_k 50 --mppi_t 8
python test_policy.py --env FrankaFetchPickDynLiftedObstaclesEnv-v1 --play_path "log/ddpg2-FrankaFetchPickDynLiftedObstaclesEnv-v1-hgg/" --play_epoch best --goal mpc --play_policy MPPIRLPolicy --timesteps 1000 --env_n_substeps 5 --episodes 200 --mppi_k 50 --mppi_t 6
python test_policy.py --env FrankaFetchPickDynLiftedObstaclesEnv-v1 --play_path "log/ddpg2-FrankaFetchPickDynLiftedObstaclesEnv-v1-hgg/" --play_epoch best --goal mpc --play_policy MPPIRLPolicy --timesteps 1000 --env_n_substeps 5 --episodes 200 --mppi_k 50 --mppi_t 4
python test_policy.py --env FrankaFetchPickDynLiftedObstaclesEnv-v1 --play_path "log/ddpg2-FrankaFetchPickDynLiftedObstaclesEnv-v1-hgg/" --play_epoch best --goal mpc --play_policy MPPIRLPolicy --timesteps 1000 --env_n_substeps 5 --episodes 200 --mppi_k 10 --mppi_t 10
python test_policy.py --env FrankaFetchPickDynLiftedObstaclesEnv-v1 --play_path "log/ddpg2-FrankaFetchPickDynLiftedObstaclesEnv-v1-hgg/" --play_epoch best --goal mpc --play_policy MPPIRLPolicy --timesteps 1000 --env_n_substeps 5 --episodes 200 --mppi_k 10 --mppi_t 8
python test_policy.py --env FrankaFetchPickDynLiftedObstaclesEnv-v1 --play_path "log/ddpg2-FrankaFetchPickDynLiftedObstaclesEnv-v1-hgg/" --play_epoch best --goal mpc --play_policy MPPIRLPolicy --timesteps 1000 --env_n_substeps 5 --episodes 200 --mppi_k 10 --mppi_t 6
python test_policy.py --env FrankaFetchPickDynLiftedObstaclesEnv-v1 --play_path "log/ddpg2-FrankaFetchPickDynLiftedObstaclesEnv-v1-hgg/" --play_epoch best --goal mpc --play_policy MPPIRLPolicy --timesteps 1000 --env_n_substeps 5 --episodes 200 --mppi_k 10 --mppi_t 4
