FROM arm64v8/ros:humble

ARG BUILD_MICRO_ROS_AGENT="yes"
ARG BUILD_OUR_ROS2_WS="yes"

# THIS FILE IS BASED ON: https://ardupilot.org/dev/docs/ros2-pi.html#cross-compile-an-application-with-docker 

# The "--mount" in the following RUN command is used to cache stuff for apt-get (it only applies to this RUN command, not to the whole Dockerfile)
RUN --mount=target=/var/lib/apt/lists,type=cache,sharing=locked \
    --mount=target=/var/cache/apt,type=cache,sharing=locked \
    apt-get update && apt-get -y --no-install-recommends install git cmake build-essential


# WORKDIR /
# RUN git clone --depth 1 --branch humble https://github.com/micro-ROS/micro-ROS-Agent.git
# WORKDIR /micro-ROS-Agent
# RUN --mount=target=/var/lib/apt/lists,type=cache,sharing=locked \                   
#     --mount=target=/var/cache/apt,type=cache,sharing=locked \
#     apt-get update && \
#     rosdep update && \
#     # install dependencies for micro-ROS-Agent:
#     rosdep install --from-paths . --ignore-src -y --dependency-types build && \     
#     # install fastcdr needed for micro-ROS-Agent:
#     apt-get -y --no-install-recommends install ros-humble-fastcdr                   
#  # source underlay and build micro-ROS-Agent
# RUN . /opt/ros/humble/setup.sh && colcon build     


# Note: for commands within RUN, we need \ for each line (except the last), else docker will think its the end of RUN

WORKDIR /
RUN --mount=target=/var/lib/apt/lists,type=cache,sharing=locked \
    --mount=target=/var/cache/apt,type=cache,sharing=locked \
    if [ "$BUILD_MICRO_ROS_AGENT" = "yes" ]; then \
        git clone --depth 1 --branch humble https://github.com/micro-ROS/micro-ROS-Agent.git && \
        cd micro-ROS-Agent && \
        apt-get update && \
        rosdep update && \
        # install dependencies for micro-ROS-Agent:
        rosdep install --from-paths . --ignore-src -y --dependency-types build && \
        # install fastcdr needed for micro-ROS-Agent:
        apt-get -y --no-install-recommends install ros-humble-fastcdr && \
        # source underlay and build micro-ROS-Agent
        . /opt/ros/humble/setup.sh && colcon build; \
    else \
        echo "Skipping micro-ROS-Agent"; \
    fi


#  # copy the whole workspace into the container:
# COPY . /our_ros2_ws  
# WORKDIR /our_ros2_ws 
# # (we dont need to --mount cache stuff here, because we dont use apt-get inside this RUN command)
# RUN rosdep update && \     
#     # install dependencies for our workspace (in they way we normally do for a ROS2 workspace):
#     # (https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#resolve-dependencies)
#     rosdep install -i --from-path src --rosdistro humble -y
# RUN . /opt/ros/humble/setup.sh && colcon build


WORKDIR /
# copy the whole workspace into the container:
COPY . /our_ros2_ws
WORKDIR /our_ros2_ws
# (we dont need to --mount cache stuff here, because we dont use apt-get inside this RUN command)
RUN if [ "$BUILD_OUR_ROS2_WS" = "yes" ]; then \
        rosdep update && \
        # install dependencies for our workspace - in they way we normally do for a ROS2 workspace EXCEPT:
            # we we only need build dependencies (we can specify with --dependency-types) - exec dependencies are only needed and installed on the PI itself
        rosdep install -i --from-path src --rosdistro humble -y --dependency-types build && \
        . /opt/ros/humble/setup.sh && colcon build; \
    else \
        echo "Skipping our ROS2 workspace"; \
    fi

