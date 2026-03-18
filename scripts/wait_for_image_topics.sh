#!/bin/bash

WAIT_TIMEOUT="${IMAGE_VIEW_WAIT_TIMEOUT_SEC:-30}"
PUBLISH_DEBUG_IMAGE="${PUBLISH_DEBUG_IMAGE:-true}"
PUBLISH_DEBUG_MAP="${PUBLISH_DEBUG_MAP:-true}"
LAUNCH_DEBUG_MAP="${LAUNCH_DEBUG_MAP:-false}"

topics_to_wait=()
[[ "$PUBLISH_DEBUG_IMAGE" == "true" ]] && topics_to_wait+=("/detect_image")
[[ "$PUBLISH_DEBUG_MAP" == "true" ]] && topics_to_wait+=("/nyush_map_image")
[[ "$LAUNCH_DEBUG_MAP" == "true" ]] && topics_to_wait+=("/map_2d")

if ((${#topics_to_wait[@]} == 0)); then
  echo ">>> No debug image topics enabled. Opening rqt_image_view immediately."
  exec ros2 run rqt_image_view rqt_image_view
fi

echo ">>> Waiting up to ${WAIT_TIMEOUT}s for image topics: ${topics_to_wait[*]}"

start_ts=$(date +%s)
while true; do
  topic_list="$(ros2 topic list 2>/dev/null || true)"
  missing_topics=()

  for topic in "${topics_to_wait[@]}"; do
    if ! grep -Fxq "$topic" <<< "$topic_list"; then
      missing_topics+=("$topic")
    fi
  done

  if ((${#missing_topics[@]} == 0)); then
    echo ">>> Image topics ready: ${topics_to_wait[*]}"
    exec ros2 run rqt_image_view rqt_image_view
  fi

  now_ts=$(date +%s)
  if (( now_ts - start_ts >= WAIT_TIMEOUT )); then
    echo ">>> Timed out waiting for: ${missing_topics[*]}"
    echo ">>> Opening rqt_image_view anyway."
    exec ros2 run rqt_image_view rqt_image_view
  fi

  sleep 1
done
