# RAFT scene-flow prototype

- A static RGB-D camera in Gazebo (`worlds/test_scene.sdf`, 160×128 pixels, 30 Hz).
- RAFT-Small (torchvision) runs on the CPU between consecutive RGB frames.
- The flow is looked up in the previous and the current point cloud.
- Velocity per pixel: `v = (P2 − P1) / dt`, using every 4th pixel.
- Published topics: `/optical_flow` (32FC2 image), `/raft/points` (coloured source points) and
  `/raft/velocity_field` (`x y z vx vy vz`); RViz line markers on `/raft_flow/markers`.

Status: kept for reference, not built. It is being ported to `src/rapid_nav_perception` with time-synchronised
inputs, back-projection from the depth image and vectorised maths.

## Known limits

- RGB and point cloud are paired as "latest of each" without time synchronisation.
- Per-pixel Python loops.
- The launch file expects a workspace at `~/Robot vision` with a `.venv`, and the package at `src/optical_flow`.
- The test scene has no moving object.

Written by Kamalesh Kumar B.
