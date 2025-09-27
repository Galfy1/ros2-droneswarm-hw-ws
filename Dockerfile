FROM arm64v8/ros:humble

RUN --mount=target=/var/lib/apt/lists,type=cache,sharing=locked \
    --mount=target=/var/cache/apt,type=cache,sharing=locked \
    apt-get update && apt-get -y --no-install-recommends install \
        git cmake build-essential

WORKDIR /
RUN git clone  --depth 1 --branch humble https://github.com/micro-ROS/micro-ROS-Agent.git
WORKDIR /micro-ROS-Agent
RUN --mount=target=/var/lib/apt/lists,type=cache,sharing=locked \
    --mount=target=/var/cache/apt,type=cache,sharing=locked \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths . --ignore-src -y --dependency-types build && \
    apt-get -y --no-install-recommends install ros-humble-fastcdr
RUN . /opt/ros/humble/setup.sh && colcon build

WORKDIR /our_ros2_ws