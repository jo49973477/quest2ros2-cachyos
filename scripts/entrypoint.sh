#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source /opt/quest_ws/install/setup.bash
exec "$@"
