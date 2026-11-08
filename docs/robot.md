# The robot and the simulation

RAPID-NAV's robot is a round indoor differential-drive robot, about 0.45 m across, in the style of a service robot. It
has a dark graphite body, an off-white top deck, an orange bumper band, a lidar on a short turret and an RGB-D camera
at the front, under the lidar's scan plane. The model is built from primitives (cylinders, boxes and spheres), with no
meshes, so it loads the same on every machine. It is written in xacro and lives in `rapid_nav_description`. The
Gazebo side (the arena, the spawn and the topic bridge) is in `rapid_nav_gazebo`; the EKF and the launch files are in
`rapid_nav_bringup`.

## Frames

All positions are relative to the parent link. `base_link` sits on the wheel axis, 0.05 m above the ground;
`base_footprint` is on the ground under the robot centre. Units are metres, radians and kilograms.

| link | parent | joint | origin xyz / rpy | geometry | mass (kg) |
|---|---|---|---|---|---|
| `base_footprint` | (root) | `base_link_joint` (fixed) | `0 0 0.05` / `0 0 0` | none | — |
| `base_link` | `base_footprint` | `base_link_joint` (fixed) | `0 0 0.05` / `0 0 0` | chassis cylinder, r 0.20, length 0.14 | 6.0 |
| `left_wheel_link` | `base_link` | `left_wheel_joint` (continuous, axis `0 1 0`) | `0 0.225 0` / `0 0 0` | cylinder, r 0.05, length 0.035 | 0.4 |
| `right_wheel_link` | `base_link` | `right_wheel_joint` (continuous, axis `0 1 0`) | `0 -0.225 0` / `0 0 0` | as the left wheel | 0.4 |
| `front_caster_link` | `base_link` | `front_caster_joint` (fixed) | `0.15 0 -0.025` / `0 0 0` | sphere, r 0.025 | 0.1 |
| `rear_caster_link` | `base_link` | `rear_caster_joint` (fixed) | `-0.15 0 -0.025` / `0 0 0` | sphere, r 0.025 | 0.1 |
| `turret_link` | `base_link` | `turret_joint` (fixed) | `0 0 0.18` / `0 0 0` | cylinder, r 0.04, length 0.06 | 0.05 |
| `lidar_link` | `base_link` | `lidar_joint` (fixed) | `0 0 0.225` / `0 0 0` | cylinder, r 0.035, length 0.03 | 0.17 |
| `camera_link` | `base_link` | `camera_joint` (fixed) | `0.175 0 0.175` / `0 0 0` | box 0.025 × 0.09 × 0.025 | 0.07 |
| `camera_optical_frame` | `camera_link` | `camera_optical_joint` (fixed) | `0 0 0` / `-1.5708 0 -1.5708` | none | — |
| `imu_link` | `base_link` | `imu_joint` (fixed) | `0 0 0.10` / `0 0 0` | none (inside the body) | 0.01 |

The total mass is 7.3 kg. The deck, the bumper and the camera mount are visual only (no mass, no collision).

The camera sits below the lidar's scan plane, so the lidar sees over it. With the default 69.4° horizontal field of
view at 640 × 480, the floor becomes visible about 0.43 m ahead.

The frame tree as seen by TF is `odom → base_footprint → base_link → {camera_link → camera_optical_frame, imu_link,
lidar_link, turret_link, left_wheel_link, right_wheel_link, front_caster_link, rear_caster_link}`. The transform
`odom → base_footprint` is published by one node only: the EKF by default, or the DiffDrive plugin (bridged to
`/tf`) when `odom_tf:=wheel`. The `map` frame comes with localisation in S5.

## Sensors

| sensor | on link | type | rate | settings | ROS topic(s) | frame |
|---|---|---|---|---|---|---|
| RGB-D camera | `camera_link` | `rgbd_camera` | 30 Hz | 640 × 480 `R8G8B8`, 69.4° horizontal field of view, clip 0.1–10 m, image noise σ 0.007 | `/camera/image`, `/camera/depth_image` (32FC1, metres), `/camera/camera_info`, `/camera/points` | `camera_optical_frame` |
| IMU | `imu_link` | `imu` | 200 Hz | gyro noise σ 0.0025 rad/s, bias σ 0.0005; accelerometer noise σ 0.02 m/s², bias σ 0.01 | `/imu` | `imu_link` |
| 2D lidar | `lidar_link` | `gpu_lidar` | 10 Hz | 360 beams over a full turn, range 0.12–12 m, noise σ 0.01 m | `/scan` | `lidar_link` |
| wheel odometry | (DiffDrive plugin) | — | 50 Hz | wheel separation 0.45 m, wheel radius 0.05 m | `/odom` | `odom` → `base_footprint` |
| ground truth | (simulator) | — | 50 Hz | pose of the robot in the world, for checks and metrics only | `/ground_truth/odom` | `world` → `base_footprint` |

The camera's size, rate and field of view are xacro arguments: `camera_width`, `camera_height`, `camera_rate`,
`camera_hfov` and `camera_pitch`.

## Topics

| ROS topic | type | direction | notes |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo → ROS | simulated time |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | ROS → Gazebo | drive command |
| `/odom` | `nav_msgs/msg/Odometry` | Gazebo → ROS | wheel odometry, 50 Hz |
| `/joint_states` | `sensor_msgs/msg/JointState` | Gazebo → ROS | the two wheel joints, 50 Hz |
| `/imu` | `sensor_msgs/msg/Imu` | Gazebo → ROS | 200 Hz |
| `/scan` | `sensor_msgs/msg/LaserScan` | Gazebo → ROS | 10 Hz |
| `/camera/image` | `sensor_msgs/msg/Image` | Gazebo → ROS | `rgb8`, 30 Hz |
| `/camera/depth_image` | `sensor_msgs/msg/Image` | Gazebo → ROS | `32FC1` in metres |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Gazebo → ROS | intrinsics |
| `/camera/points` | `sensor_msgs/msg/PointCloud2` | Gazebo → ROS | bridged lazily (only when someone subscribes) |
| `/ground_truth/odom` | `nav_msgs/msg/Odometry` | Gazebo → ROS | simulator pose, for checks and metrics only |
| `/tf` | `tf2_msgs/msg/TFMessage` | Gazebo → ROS | only with `odom_tf:=wheel`: the DiffDrive transform |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | EKF output | the default odometry (`odom_tf:=ekf`) |
| `/tf`, `/tf_static` | | robot_state_publisher | the robot's own transforms, from `/joint_states` |
| `/robot_description` | `std_msgs/msg/String` | robot_state_publisher | the model as a string |

## Launch files

`ros2 launch rapid_nav_bringup sim.launch.py` starts Gazebo with the arena and the robot, the topic bridge, the
robot_state_publisher, the odometry source and RViz.

| argument | default | meaning |
|---|---|---|
| `world` | the installed `arena.sdf` | the SDF world to load |
| `headless` | `false` | no window and no RViz; software rendering (used by the Docker check) |
| `rviz` | `true` | start RViz with `sim.rviz` (not when headless) |
| `odom_tf` | `ekf` | `ekf`: the EKF publishes `odom → base_footprint`; `wheel`: the DiffDrive transform is bridged instead. Any other value stops the launch with a message |
| `x`, `y`, `yaw` | `0.0`, `0.0`, `0.0` | spawn pose (m, m, rad) |

The camera arguments of `rapid_nav_description` (`camera_width`, `camera_height`, `camera_rate`, `camera_hfov`,
`camera_pitch`) can be passed on the same command line.

`ros2 launch rapid_nav_description display.launch.py` shows the model in RViz without Gazebo, with a joint slider
window to move the wheels and the turret. It takes the same camera arguments.

## Odometry: wheels for velocity, gyro for rotation

The EKF (`robot_localization`, two-dimensional mode) takes the forward and lateral velocity from the wheel odometry,
and the yaw rate and the forward acceleration from the IMU, with gravity removed. The gyro measures rotation directly,
while wheel odometry is unreliable for rotation when the wheels slip, so the yaw rate comes from the gyro.
