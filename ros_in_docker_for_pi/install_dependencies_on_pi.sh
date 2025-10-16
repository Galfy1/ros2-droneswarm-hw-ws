#!/bin/bash
# THE ABOVE LINE IS REQUIRED FOR THE SCRIPT TO BE EXECUTABLE AS A BASH SCRIPT - DO NOT REMOVE IT!
# Make this file executable with: chmod +x <script_name>.sh (just has to be done once)
# Run this script with: ./<script_name>.sh

# WILL INSTALL DEPENDENCIES IN THE ROS2 DOCKER CONTAINER ON THE PI
# THE SCRIPT ASSUMES THAT THE DOCKER CONTAINER IS ALREADY RUNNING ON THE PI


# create variables
pi_hostname="raspberrypi.local" # this is the assumed hostname of the pi, change if needed.
our_ws_target_dir=~/ros2_droneswarm/workspaces/our_ws
microros_ws_target_dir=~/ros2_droneswarm/workspaces/microros_ws


ssh -t $pi_hostname docker exec ros2 bash -c "
    source /opt/ros/humble/setup.bash &&
    rosdep init || true &&
    apt update &&
    rosdep update &&
    cd $microros_ws_target_dir &&
        rosdep install --from-paths install --dependency-types exec &&
    cd $our_ws_target_dir &&
        rosdep install --from-paths install --dependency-types exec"
