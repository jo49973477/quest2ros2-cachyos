# ROS 2 Humble runs on Ubuntu 22.04 (Jammy) inside this container.
# v3: official ROS 2 endpoint + targeted upstream sender API typo fix.
# The CachyOS host needs only Docker Engine + Docker Compose.
FROM ros:humble-ros-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    ROS_LOCALHOST_ONLY=0 \
    ROS_DOMAIN_ID=30 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Includes the official Universal Robots ROS 2 driver and a mock-only,
# fail-closed Quest-pose-to-MoveIt-Servo adapter copied below.
RUN apt-get update && apt-get install -y --no-install-recommends \
    git ca-certificates bash iproute2 iputils-ping netcat-openbsd \
    python3-numpy python3-colcon-common-extensions \
    ros-humble-rosidl-default-generators \
    ros-humble-geometry-msgs ros-humble-visualization-msgs \
    ros-humble-control-msgs ros-humble-tf2-ros \
    ros-humble-tf-transformations \
    ros-humble-ros2controlcli \
    ros-humble-rosbag2 \
    ros-humble-ur \
    ros-humble-moveit-servo ros-humble-ur-moveit-config \
    && rm -rf /var/lib/apt/lists/*

COPY scripts/fix_tcp_sender.py /opt/quest-tools/fix_tcp_sender.py

ARG TAOKT_REF=main
ARG QUEST_REF=ros2
ARG TCP_REF=main-ros2
WORKDIR /opt/quest_ws

RUN mkdir -p src && \
    git clone --depth 1 --branch "${TAOKT_REF}" \
      https://github.com/Taokt/Quest2ROS2.git src/Quest2ROS2 && \
    git clone --depth 1 --branch "${QUEST_REF}" \
      https://github.com/Quest2ROS/quest2ros.git src/quest2ros && \
    git clone --depth 1 --branch "${TCP_REF}" \
      https://github.com/MWelle77/ROS-TCP-Endpoint.git src/ROS-TCP-Endpoint && \
    python3 /opt/quest-tools/fix_tcp_sender.py \
      src/ROS-TCP-Endpoint/ros_tcp_endpoint/tcp_sender.py && \
    touch src/Quest2ROS2/Files_for_msg_pkg/COLCON_IGNORE && \
    sed -i 's#<depend>quest2ros2_msg</depend>#<depend>quest2ros</depend>#' \
      src/Quest2ROS2/package.xml && \
    grep -q '<depend>quest2ros</depend>' src/Quest2ROS2/package.xml && \
    source /opt/ros/humble/setup.bash && \
    colcon build --merge-install --event-handlers console_direct+ \
      --cmake-args -DBUILD_TESTING=OFF && \
    source install/setup.bash && \
    ros2 pkg prefix quest2ros && \
    ros2 pkg prefix q2r2_bringup && \
    ros2 pkg prefix ros_tcp_endpoint

COPY scripts/entrypoint.sh scripts/launch_real_ur.sh \
     scripts/alias_topics.py scripts/quest_target_node.py \
     scripts/pose_to_twist_bridge.py scripts/teleop_math.py \
     scripts/ur10_jacobian.py scripts/check_mock_servo.sh \
     scripts/mock_servo_integration_test.py scripts/check_mock_joint_state.py \
     /opt/quest-tools/
COPY config/ur10_mock_initial_positions.yaml \
     /opt/quest-tools/ur10_mock_initial_positions.yaml
RUN chmod +x /opt/quest-tools/entrypoint.sh \
    /opt/quest-tools/launch_real_ur.sh \
    /opt/quest-tools/check_mock_servo.sh \
    /opt/quest-tools/ur10_jacobian.py \
    /opt/quest-tools/mock_servo_integration_test.py \
    /opt/quest-tools/check_mock_joint_state.py \
    && mkdir -p /bags

ENTRYPOINT ["/opt/quest-tools/entrypoint.sh"]
CMD ["ros2", "run", "ros_tcp_endpoint", "default_server_endpoint", "--ros-args", "-p", "ROS_IP:=0.0.0.0", "-p", "ROS_TCP_PORT:=10000"]
