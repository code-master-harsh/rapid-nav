# Setup

## Install ROS 2 Jazzy and Gazebo Harmonic

On Ubuntu 24.04, follow the official
[ROS 2 Jazzy install guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), then install the
packages this project uses:

```bash
sudo apt install ros-jazzy-desktop ros-jazzy-ros-gz python3-colcon-common-extensions python3-rosdep
```

## Get and build the workspace

```bash
mkdir -p ~/ros_ws && cd ~/ros_ws
git clone https://github.com/code-master-harsh/rapid-nav.git
cd rapid-nav
sudo rosdep init   # only once per machine; skip if it says it is already initialised
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

Every new terminal needs `source /opt/ros/jazzy/setup.bash` and `source install/setup.bash`.

## Updating

Run `git pull`, then `rosdep install --from-paths src --ignore-src -y` and `colcon build --symlink-install` again.

## System check

If something does not build or run, paste the output of these commands when you ask for help:

```bash
lsb_release -ds; printenv ROS_DISTRO; gz sim --versions; nproc; free -h | head -2
nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv 2>/dev/null || echo "no NVIDIA GPU"
python3 -c "import torch; print('torch', torch.__version__, 'cuda', torch.cuda.is_available())" 2>/dev/null || echo "no torch"
```

## Python packages for perception

PyTorch setup is described with the perception package.
