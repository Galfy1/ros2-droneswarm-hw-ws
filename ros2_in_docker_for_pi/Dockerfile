# most minimal ros humble image:
FROM ros:humble-ros-core 
#FROM ros:humble-ros-base
#FROM arm64v8/ros:humble
# (https://hub.docker.com/_/ros)

# ROS2 underlay is already sourced in this base image (i.e. source /opt/ros/humble/setup.sh)

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


# Regarding the "python3-rosdep" package:
    # rosdep is not part of ros-core, se we install it manually
RUN apt update && apt install -y python3-rosdep nano

RUN rosdep init

# TODO: overvej at rosdep init herinde (tror det kræver underlay source men ikke sikker)


# BRUG RUN TIL AT KØRE TING DER SKAL KØRE UNDER BYGNINGEN AF CONTAINEREN
# BRUG CMD TIL AT KØRE TING DER SKAL KØRE NÅR CONTAINEREN STARTER
