#!/usr/bin/env bash
set -e

source /opt/ros/noetic/setup.bash

if [ -f /opt/relay_explore_ws/devel/setup.bash ]; then
  source /opt/relay_explore_ws/devel/setup.bash
fi

exec "$@"
