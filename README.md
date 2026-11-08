# RAPID-NAV

*Reliability-Aware Perception for Intelligent Dynamic Navigation*

[![ci](https://github.com/code-master-harsh/rapid-nav/actions/workflows/ci.yml/badge.svg)](https://github.com/code-master-harsh/rapid-nav/actions/workflows/ci.yml)

A differential-drive mobile robot in Gazebo that uses optical flow to see which things move. Fast flow (FlowNetS) is
used by default, and accurate flow (RAFT) is used when a reliability check fails. The robot's own motion is removed
with depth, the IMU and wheel odometry, and the remaining motion becomes hazards in the Nav2 local costmap.

A project for AR522 Robot Vision and AR511 Autonomous Mobile Robots at CAIR, IIT Mandi.

## Status

The robot and the simulation are built and in testing; the other parts are planned.

| Part | Stage | Status |
|---|---|---|
| Robot and simulation | S1 | in testing |
| Perception | S2 | planned |
| Ego-motion compensation | S3 | planned |
| Dynamic scenarios | S4 | planned |
| Navigation baseline | S5 | planned |
| Hazard costmap | S6 | planned |
| Adaptive flow | S7 | planned |
| Experiments | S8 | planned |

## Pipeline

```mermaid
flowchart LR
    cam[RGB-D camera] --> flow[FlowNetS flow]
    flow --> gate{reliability gate}
    gate -- keep --> sel[selected flow]
    gate -- unreliable --> raft[RAFT flow]
    raft --> sel
    sel --> ego[ego-motion removal<br/>depth, IMU + wheel odometry EKF]
    ego --> res[residual motion]
    res --> haz[hazard points]
    haz --> cost[Nav2 local costmap]
    cost --> dwb[DWB]
    dwb --> cmd[cmd_vel]
```

See [docs/architecture.md](docs/architecture.md) for the design.

## Repository layout

- `src/`: ROS 2 packages: the robot description, the Gazebo arena and bridge, the launch files and the `sim_check` tool.
- `prototypes/`: reference code, not built.
- `docs/`: setup, development and architecture notes.
- `docker/`: optional build-and-test image for machines without a native ROS 2 install.
- `scripts/`: `dev.sh`, which drives the Docker image.

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic (`ros-jazzy-ros-gz`)

The full steps are in [docs/setup.md](docs/setup.md).

## Quick start

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

Start the simulation (Gazebo with the arena and the robot, the odometry and RViz):

```bash
ros2 launch rapid_nav_bringup sim.launch.py
```

Drive the robot with the keyboard, in a second terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Run the automatic check, in a third terminal (the robot drives by itself, so keep clear of it):

```bash
ros2 run rapid_nav_tools sim_check
```

The robot is described in [docs/robot.md](docs/robot.md). The step-by-step check procedure for testers is in
[docs/sim-checks.md](docs/sim-checks.md).

## Team

Rishi Atreya, Kamalesh Kumar B, Jaspinder Kaur, Harsh Vardhan (CAIR, IIT Mandi).

## Licence

MIT, see [LICENSE](LICENSE).
