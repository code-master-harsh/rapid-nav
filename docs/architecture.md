# Architecture

This document describes the design of the system. Parts marked *(planned)* are not built yet; they arrive in the
stage named in the [README](../README.md) status table.

## Pipeline

1. **Sensors (planned).** RGB-D camera at 640×480 and 30 Hz with metric depth; IMU at 200 Hz; 2D lidar at 10 Hz;
   wheel odometry.

2. **Adaptive flow.** FlowNetS computes the optical flow by default. The reliability statistic is the 95th
   percentile of the flow magnitude. When that statistic says FlowNetS is unreliable, RAFT computes the flow instead.
   On Sintel Clean (team study), routing sends 42.5% of frames to RAFT: end-point error (EPE) 0.95 px at 14.5 FPS.
   FlowNetS alone gives 5.15 px at 65 FPS; RAFT alone gives 0.63 px at 7.9 FPS.

3. **Ego-motion.** The expected flow of a static scene is computed from depth, the camera intrinsics and the camera
   motion between frames. The rotation comes from the gyro, which needs no depth. The translation comes from the EKF
   velocity and the depth. The residual flow is `F_res = F* − F_ego`, where `F*` is the selected flow and `F_ego` is
   the expected flow of a static scene. Regions that move on their own have a large residual.

4. **Hazards.** Residual motion, depth, reliability and persistence over several frames give hazard points. Each
   hazard point is projected ahead along its estimated motion and written into the Nav2 local costmap.

5. **Navigation.** Nav2 with DWB plans and follows the path. AMCL localises the robot on a known map using the lidar.

6. **Evaluation.** Configurations A–F come from the proposal:
   - A–C compare flow sources on a static costmap;
   - D adds residual motion and a dynamic costmap;
   - E adds a global planner;
   - F (stretch goal) adds semantics.

   The geometry-only baseline is Nav2's standard obstacle layer. The measures are collision rate, minimum clearance,
   success, time to goal, path length, replanning count, perception latency and RAFT share.

## Frames (planned)

```
map → odom → base_footprint → base_link → { camera_link → camera_optical_frame,
                                            imu_link, lidar_link, wheels, casters }
```
