#!/usr/bin/env bash
# Run social perception inside the team's already-running container.
# Logs stay in this terminal. Ctrl+C stops the pipeline, not the team container.
set -Eeuo pipefail
CONTAINER="${SOCIAL_CONTAINER:-navigoat-dev}"
if [[ "$(sudo docker inspect --format '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" != true ]]; then
  echo "Start the team container ($CONTAINER) first." >&2
  exit 1
fi
exec sudo docker exec -it "$CONTAINER" bash -c '
  source /opt/ros/jazzy/setup.bash
  if [[ -f /workspace/install/setup.bash ]]; then
    source /workspace/install/setup.bash
  fi
  export DASHBOARD_HOST="${DASHBOARD_HOST:-0.0.0.0}"
  export YOLO_CONF="${YOLO_CONF:-0.40}"
  exec bash /workspace/ros/src/autonomy/perception/social_perception/jetson_pipeline.sh
'
