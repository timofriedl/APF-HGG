# Dynamic Obstacle Avoidance Using Artificial Potential Field in Reinforcement Learning

## Description

APF-HGG is based on the repository of [MPPI-HGG](https://github.com/phinners/HGG-MPPI) by Patrick Hinners.
Instead of using Model Predictive Path Integral (MPPI) Control to avoid dynamic obstacles, an Artificial Potential Field (APF) approach is used.
APF calculates magnet-like forces that push the robot away from close obstacles. The task-space forces are converted to joint-space torques, added to the PID output and directly applied to the robot.
You can find the C++ implementation of the APF PID controller in `./apf`.


## Installation

Start with Ubuntu 22.04. Make sure the following packages are installed:
- git: ```sudo apt install git```
- gcc: ```sudo apt install gcc```
- libx11-dev: ```sudo apt-get install libx11-dev```
- libglew-dev: ```sudo apt-get install libglew-dev```
- patchelf: ```sudo apt-get install patchelf```
- ffmpeg: ```sudo apt-get install ffmpeg```
  
- Install Python 3.7:
```
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt update
sudo apt install python3.7
```

- Install cmake-3.30:
```
wget https://github.com/Kitware/CMake/releases/download/v3.30.0/cmake-3.30.0-linux-x86_64.tar.gz
tar -zxvf cmake-3.30.0-linux-x86_64.tar.gz
sudo mv cmake-3.30.0-linux-x86_64 /opt/cmake
sudo ln -s /opt/cmake/bin/* /usr/local/bin/
```

- Install g++:
```
sudo apt-get install g++
export CXX=/usr/bin/g++
```

- Create and activate virtual environment:
```
sudo apt install python3.7-venv 
python3.7 -m venv apf-hgg
source apf-hgg/bin/activate
```

- Install transformations:
```
sudo apt-get install python3-dev
sudo apt-get install python3.7-dev
pip install transformations
```

- Install python packages:
```
pip install -r requirements.txt
```

- Build C++ APF Code:
```
./apf/build.sh
```

- Install Mujoco and Forces Pro (see Patrick Hinner's [MPPI-HGG](https://github.com/phinners/HGG-MPPI) repository)

- Add the following paths to your ```~/.bashrc```:
```
# Mujoco
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/.mujoco/mujoco210/bin

# Nvidia
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/nvidia

# ForcesPro
export PYTHONPATH=$PYTHONPATH:~/forces_pro_client
```

## Usage

- Load environment variables and activate your virtual environment:
```
source ~/.bashrc
source ./apf-hgg/bin/activate
```

- Make sure your models are located in ```./log```

- Play video using
```bash
python play.py --env FrankaDirectFetchPickDynLiftedObstaclesEnv-v1 --play_path log/ddpg2-FrankaDirectFetchPickDynLiftedObstaclesEnv-v1-hgg/ --play_epoch 19 --goal apf --play_policy APFPolicy --timesteps 5000 --env_n_substeps 1
```

Note that APF environments run at 1000Hz, while other environments are usually set to 500Hz. Therefore, `--timesteps 5000 --env_n_substeps 1` corresponds to 5000 * 1 / 1000Hz = 5s simulation time per rollout. Substeps must be 1 due to real time direct torque control.
