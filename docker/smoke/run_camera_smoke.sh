#!/usr/bin/env bash
# Inside the dev image: start headless Gazebo with the camera smoke world, bridge the camera
# topic to ROS 2, save one frame, measure the topic rate, then stop both processes.
# Run it through scripts/dev.sh:  scripts/dev.sh run 'bash /repo/docker/smoke/run_camera_smoke.sh'
set -uo pipefail

OUT=/repo/sim_check_docker_spike
mkdir -p "$OUT"

gz sim -s -r --headless-rendering -v 2 /repo/docker/smoke/camera_smoke.sdf > "$OUT/gz.log" 2>&1 &
GZ_PID=$!

ros2 run ros_gz_bridge parameter_bridge \
    '/smoke/camera@sensor_msgs/msg/Image[gz.msgs.Image' > "$OUT/bridge.log" 2>&1 &
BRIDGE_PID=$!

cleanup() {
    kill "$BRIDGE_PID" "$GZ_PID" 2>/dev/null
    pkill -f parameter_bridge 2>/dev/null
    wait "$BRIDGE_PID" "$GZ_PID" 2>/dev/null
}
trap cleanup EXIT

python3 /repo/docker/smoke/grab_frame.py /smoke/camera "$OUT/frame.png"
GRAB_STATUS=$?

RATE=$(timeout 15 ros2 topic hz /smoke/camera 2>&1 | grep 'average rate' | tail -n 1 || true)
echo "rate: ${RATE:-no rate measured}"

exit "$GRAB_STATUS"
