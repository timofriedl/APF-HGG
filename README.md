# Collision Free Robot Navigation in Dynamic Environments with Reinforcement Learning and Model Predictive Path Integral Control

## Description

This repository contains the implementation of the MPPI-HGG algorithm for trajectory planning with Reinforcement
Learning (RL) and Model Predictive Path Integral Control (MPPI).
The algorithm aims to improve obstacle avoidance in non-differentiable systems.
The RL agent is trained using Hindsight Goal Generation (HGG) in different environments.
To improve the safety of the RL agent, the MPPI controller is used to generate a safe trajectory to the sub-goal
recommended by the RL agent.

## Requirements

The code is tested on a Omen by HP c0169ng Laptop running Ubuntu 22.04.1 LTS.
To install all requirements, create a new pyenv environment using the `requirements.txt` file.

Create Virtualenvironment using pyenv

```bash
pyenv virtualenv 3.7.16 <name>
```

Activate Virtualenv

```bash
pyenv activate <name>
```

Install requirements.txt

```bash
pip install -r requirements.txt
```

## Mujoco

1. Download the MuJoCo version 2.1 binaries for
   [Linux](https://mujoco.org/download/mujoco210-linux-x86_64.tar.gz) or
   [OSX](https://mujoco.org/download/mujoco210-macos-x86_64.tar.gz).
2. Extract the downloaded `mujoco210` directory into `~/.mujoco/mujoco210`.

If you want to specify a nonstandard location for the package,
use the env variable `MUJOCO_PY_MUJOCO_PATH`.

I needed to export the LD_LIBRARY_PATH Variable to

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/.mujoco/mujoco210/bin
```

This Variable is also set in the Pycharm Run Configurations

## ForcesPro

To run the MPC or exHGG-MPC algorithm, first generate a ForcesPro solver by setting up ForcesPro, following the
instructions in the [ForcesPro documentation](https://forces.embotech.com/Documentation/installation/python.html), and
run the following command:

```bash
cd mpc/
python pick_dyn_obstacles.py --mpc_gen t
```

For the other environments, replace `pick_dyn_obstacles.py`
with `pick_dyn_lifted_obstacles.py`, `pick_dyn_door_obstacles.py` or `pick_3d_target_obstacles.py`.

Important is especially to set the environment Variable to be able to find the forcespro Package

```bash
export PYTHONPATH=/path/to/forces_pro_client/
```

## Usage

All the run commands are preloaded as run configurations when importing the project into PyCharm. Therefore it is
recommended to use PyCharm as IDE and run the Configurations from there.

If you want to run from terminal (which is not recommended!) go with the following structure:

1. Train the RL agent on the desired environment.
   There are four environments
   available: `FrankaFetchPickDynObstaclesEnv-v1`, `FrankaFetchPickDynLiftedObstaclesEnv-v1`, `FrankaFetchPickDynDoorObstaclesEnv-v1`,
   and `FrankaFetchPick3DTargetObstacle-v1`.
   ```bash
   python train2.py --alg ddpg2 --epochs 20 --env=FrankaFetchPickDynObstaclesEnv-v1 --reward_min -10 --goal mpc
   ```

2. Run the MPPI-HGG algorithm with the trained RL agent.
   ```bash
   python play.py --env FrankaFetchPickDynObstaclesEnv-v1 --play_path log/ddpg2-FrankaFetchPickDynObstaclesEnv-v1-hgg/ --play_epoch 19 --goal mpc --play_policy MPPIRLPolicy --timesteps 1000 --env_n_substeps 5
   ```
   For the different environments, replace both `env` and `play_path` with the desired environment and the path to the
   trained RL agent.
   To compare the performance of MPPI-HGG to pure MPPI, MPC, HGG, or exHGG-MPC, replace the `play_policy` argument
   with `MPPIPolicy`, `MPCPolicy`, `RLPolicy` or `MPCRLMPolicy` respectively.

## Future Work

- [ ] Add support CUDA for parallel MPPI trajectory rollouts.
