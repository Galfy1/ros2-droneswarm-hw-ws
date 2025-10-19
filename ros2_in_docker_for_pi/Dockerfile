# most minimal ros humble image:
FROM ros:humble-ros-core 
#FROM ros:humble-ros-base
#FROM arm64v8/ros:humble
# (https://hub.docker.com/_/ros)

# RUN vs CMD vs ENTRYPOINT:
# https://www.geeksforgeeks.org/linux-unix/difference-between-run-vs-cmd-vs-entrypoint-docker-commands/
# https://www.youtube.com/watch?v=U1P7bqVM7xM (om CMD vs ENTRYPOINT)
# RUN is only used when building the image (to execute commands and create layers), and CMD is used when running the container (to define the default command to execute).
# only one CMD instruction is allowed per Dockerfile. If you list more than one CMD then only the last CMD will take effect.
# CMD will be overridden when providing a different command when running the container with docker run.
# you can use ENTRYPOINT to set a command that will always be executed when the container starts, regardless of any command line arguments passed to docker run.
# you can have both ENTRYPOINT and CMD in the same Dockerfile. then, CMD will provide additional arguments to the ENTRYPOINT command (i.e. the CMD arguments can still be overwritten if params are provided when docker run).

# rosdep is not part of ros-core, se we install it manually
#RUN apt-get update && apt-get install -y python3-rosdep
#RUN rosdep init

#RUN apt install ros-dev-tools

# The following is a dependency for micro-ROS-Agent.. its in its depency list... but rosdep just cant seem to find the micro-ros-msgs package (even though it should be a part of the index: https://index.ros.org/p/micro_ros_msgs/#humble-overview)
# (Also. installing depencies using rosdep on micro agent seem to be succesfull... but running the agent will fail with an error that hints that the micro-ros-msgs package is not installed.. and checking apt/rosdep manually it sure is not installed. so we install it manually here


# Regarding the "python3-rosdep" package:
    # rosdep is not part of ros-core, se we install it manually
# Regarding the "ros-humble-micro-ros-msgs package:
    # The package is a dependency for micro-ROS-Agent. It's listed as a dependency in micro-ROS-Agent's dependency list (package.xml), 
    # but rosdep cannot seem to locate the micro-ros-msgs package, even though it should be included in the ROS index:  https://index.ros.org/p/micro_ros_msgs/#humble-overview   
    # Note: While rosdep seem to  install all dependencies for the micro-ROS-Agent (no errors), 
    #       running the agent may still fail with an error indicating 
    #       that the micro-ros-msgs package is missing. Manual checks via apt/rosdep 
    #       confirm that it is indeed not installed (idk why...). Therefore, we install it manually here:
RUN apt update && apt install -y python3-rosdep ros-humble-micro-ros-msgs

# TODO: overvej at rosdep init herinde (tror det kræver underlay source men ikke sikker)


# BRUG RUN TIL AT KØRE TING DER SKAL KØRE UNDER BYGNINGEN AF CONTAINEREN
# BRUG CMD TIL AT KØRE TING DER SKAL KØRE NÅR CONTAINEREN STARTER
