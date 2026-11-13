#!/usr/bin/env bash
# ROS-generated setup files intentionally probe optional unset variables, so
# nounset cannot be enabled while sourcing them.
set -eo pipefail
source /opt/ros/humble/setup.bash
source /opt/quest_ws/install/setup.bash
source /opt/ur_calibration_ws/install/setup.bash
exec "$@"
