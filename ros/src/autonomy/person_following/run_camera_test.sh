#!/usr/bin/env bash
# Run INSIDE the Jetson ROS container with the whole checkout at /workspace.
# Reuses Enid's camera/registration/YOLO/tracking/dashboard pipeline unchanged.
set -Eeuo pipefail

APP_DIR=/workspace/ros/src/autonomy/person_following
PIPELINE=/workspace/enid/social_perception/jetson_pipeline.sh

if [[ ! -f "$APP_DIR/ros_camera_test.py" || ! -f "$PIPELINE" ]]; then
  echo 'Mount the entire repository at /workspace; see person_following/README.md.' >&2
  exit 1
fi

# ROS setup scripts read optional environment variables that may be unset.
# Relax nounset only while sourcing them; keep errexit active so setup failures
# still abort startup, then restore strict checking for our own launcher.
set +u
source /opt/ros/jazzy/setup.bash
set -u
ros2 pkg prefix orbbec_camera >/dev/null
ros2 pkg prefix depth_image_proc >/dev/null
python3 -c 'import rclpy, cv_bridge, message_filters, std_srvs.srv, scipy, ultralytics'

# Enid's one-second default would expire IDs before our two-second loss window.
export TRACK_MAX_AGE=2.0
export DASHBOARD_HOST="${DASHBOARD_HOST:-127.0.0.1}"

pids=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${pids[@]}"; do kill "$pid" 2>/dev/null || true; done
  for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

bash "$PIPELINE" &
pids+=("$!")
python3 -B -u "$APP_DIR/ros_camera_test.py" --enable-on-start &
pids+=("$!")

echo 'Camera-only following test started. No drive commands are published.'
set +e
wait -n "${pids[@]}"
exit_code=$?
set -e
echo "A camera-test component exited (code $exit_code); stopping the remaining processes." >&2
exit 1
