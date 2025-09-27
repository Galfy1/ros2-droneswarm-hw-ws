#!/bin/bash
# THE ABOVE LINE IS REQUIRED FOR THE SCRIPT TO BE EXECUTABLE AS A BASH SCRIPT - DO NOT REMOVE IT!
# Make this file executable with: chmod +x our_flash_script.sh (just has to be done once)
# Run this script with: ./our_flash_script.sh

# This script is created based on the instructions from: https://ardupilot.org/dev/docs/ros2-pi.html#cross-compile-an-application-with-docker

set -e  # Stop the script if any command fails 

# Check for required arguments
if [ -z "$1" ]; then
    echo "Error: Missing required argument <pi_username>"
    echo "Usage: $0 <pi_username>"
    exit 1
fi
if [ -z "$2" ]; then
    echo "Error: Missing required argument <install_dependencies>"
    echo "Usage: $0 yes|no"
    exit 1
fi

# Get the arguments
pi_username=$1
install_dependencies=$2

# Build with multiplatform support.
docker build . --platform linux/arm64 -t micro_ros_agent

# Create a temporary directories to hold the installation files.
rm -rf temp_micro_agent temp_our_ws  # Remove any existing temp directory to avoid conflicts.
mkdir -p temp_micro_agent/install temp_our_ws/install  # Create the temp directory.

# Copy the installation directory from the docker image to the host.
docker create --name dummy micro_ros_agent
docker cp dummy:/micro-ROS-Agent/install temp_micro_agent/install
docker rm -f dummy

# And now, from host to target's home directory, but ignore COLCON_IGNORE.
rsync -aRv --exclude temp_micro_agent/install/COLCON_IGNORE temp_micro_agent/install $pi_username@ubuntu.local:/home/$pi_username/microros_ws

# Clean up the temporary directories.
rm -rf temp_micro_agent
rm -rf temp_our_ws

# Optionally install dependencies on the target.
if [ "$install_dependencies" == "yes" ]; then
    ssh $pi_username@ubuntu.local "cd ~/microros_ws && \
    sudo rosdep init && \
    apt update && \
    rosdep update && \
    rosdep install --from-paths install --dependency-types exec && \
    cd ~/our_ros2_ws && \
    sudo rosdep init && \
    apt update && \
    rosdep update && \
    rosdep install --from-paths install --dependency-types exec"
fi

