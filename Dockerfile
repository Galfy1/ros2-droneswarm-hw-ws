FROM arm64v8/ros:humble

# THIS FILE IS BASED ON: https://ardupilot.org/dev/docs/ros2-pi.html#cross-compile-an-application-with-docker 

# The "--mount" in the following RUN command is used to cache stuff for apt-get (it only applies to this RUN command, not to the whole Dockerfile)
RUN --mount=target=/var/lib/apt/lists,type=cache,sharing=locked \
    --mount=target=/var/cache/apt,type=cache,sharing=locked \
    apt-get update && apt-get -y --no-install-recommends install git cmake build-essential

WORKDIR /
RUN git clone  --depth 1 --branch humble https://github.com/micro-ROS/micro-ROS-Agent.git
WORKDIR /micro-ROS-Agent
RUN --mount=target=/var/lib/apt/lists,type=cache,sharing=locked \                   
    --mount=target=/var/cache/apt,type=cache,sharing=locked \
    apt-get update && \
    rosdep update && \
    # install dependencies for micro-ROS-Agent:
    rosdep install --from-paths . --ignore-src -y --dependency-types build && \     
    # install fastcdr needed for micro-ROS-Agent:
    apt-get -y --no-install-recommends install ros-humble-fastcdr                   
 # source underlay and build micro-ROS-Agent
RUN . /opt/ros/humble/setup.sh && colcon build                                     

 # copy the whole workspace into the container:
COPY . /our_ros2_ws  
WORKDIR /our_ros2_ws 
# (we dont need to --mount cache stuff here, because we dont use apt-get inside this RUN command)
RUN rosdep update && \     
    # install dependencies for our workspace (in they way we normally do for a ROS2 workspace):
    # (https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#resolve-dependencies)
    rosdep install -i --from-path src --rosdistro humble -y
RUN . /opt/ros/humble/setup.sh && colcon build

