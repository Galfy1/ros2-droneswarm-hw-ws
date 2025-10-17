#!/bin/bash
# THE ABOVE LINE IS REQUIRED FOR THE SCRIPT TO BE EXECUTABLE AS A BASH SCRIPT - DO NOT REMOVE IT!
# Make this file executable with: chmod +x our_flash_script.sh (just has to be done once)
# Run this script with: ./our_flash_script.sh

# This script is created based on the instructions from: https://ardupilot.org/dev/docs/ros2-pi.html#cross-compile-an-application-with-docker

set -e  # Stop the script if any command fails 

# Check for required arguments
# if [ -z "$1" ]; then
#     echo "Error in first arg: Missing required argument <pi_hostname> (e.g. raspberrypi.local)"
#     exit 1
# fi
if [[ -z "$1" || ( "$1" != "yes" && "$1" != "no" ) ]]; then
    echo "Error in second arg: Missing argument or invalid value. <build_application> (values: yes or no)"
    exit 1
fi
if [[ -z "$2" || ( "$2" != "yes" && "$2" != "no" ) ]]; then
    echo "Error in third arg: Missing argument or invalid value. <build_micro_ros_agent> (values: yes or no)"
    exit 1
fi
if [[ -z "$3" || ( "$3" != "yes" && "$3" != "no" ) ]]; then
    echo "Error in fourth arg: Missing argument or invalid value. <install_dependencies> (values: yes or no)"
    exit 1
fi

# Get the arguments

build_application=$1
build_micro_ros_agent=$2
install_dependencies=$3

# create variables
pi_hostname="raspberrypi.local" # this is the assumed hostname of the pi, change if needed.

ros2_container_base_dir=~/ros2_droneswarm
our_ws_target_dir=$ros2_container_base_dir/workspaces/our_ws
microros_ws_target_dir=$ros2_container_base_dir/workspaces/microros_ws
ros2_container_name="ros2"

# Make sure any already existing container named "dummy" is removed before we start.
docker rm -f dummy || true  # "|| true" will ignore error if dummy does not exist (remember, we are usings "set -e" at the top of the script)
                            # this will give an "ERROR: default sources list file already exists" error if the container does not exist - we can ignore that error!

# Build with multiplatform support.
docker build . --platform linux/arm64 -t build_image \
    --build-arg BUILD_MICRO_ROS_AGENT=$build_micro_ros_agent \
    --build-arg BUILD_OUR_ROS2_WS=$build_application 
# Create a dummy container from the image
docker create --name dummy build_image

# Create a temporary directories to hold the installation files.
rm -rf temp_our_ws temp_micro_agent  # Remove any existing temp directory to avoid conflicts.
mkdir -p temp_our_ws/install temp_micro_agent/install  # Create the temp directory.

# Only copy to target if it was actually built.
if [ "$build_application" = "yes" ]; then
    # Copy the installation directory from the docker image to the host.
    docker cp dummy:/our_ros2_ws/install/. temp_our_ws/install 
    # And now, from host to target's directory, but ignore COLCON_IGNORE.
    rsync -av --mkpath --exclude temp_our_ws/install/COLCON_IGNORE temp_our_ws/install $pi_hostname:$our_ws_target_dir
fi
if [ "$build_micro_ros_agent" = "yes" ]; then
    docker cp dummy:/micro-ROS-Agent/install/. temp_micro_agent/install
    rsync -av --mkpath --exclude temp_micro_agent/install/COLCON_IGNORE temp_micro_agent/install $pi_hostname:$microros_ws_target_dir
fi

docker rm -f dummy

# Clean up the temporary directories.
rm -rf temp_our_ws
rm -rf temp_micro_agent


ssh -t $pi_hostname " cd $ros2_container_base_dir && \
                      docker compose down || true && \
                      docker compose up -d " # -d to run in detached mode




# The paths that the docker volumes are mapped to INSIDE THE DOCKER CONTAINER (defined in docker-compose.yml)
our_ws_target_dir_in_docker=~/ros2_droneswarm/workspaces/our_ws
microros_ws_target_dir_in_docker=~/ros2_droneswarm/workspaces/microros_ws

# Re-run docker container to apply new build files (the container will run the command defined docker-compose.yml, that will run/launch to ros2 nodes from th new files)
# Optionally install dependencies on the target.
# (https://www.cyberciti.biz/faq/unix-linux-execute-command-using-ssh/)
ssh $pi_hostname << EOF   # (no quotes around EOF to allow variable expansion on local machine)
    cd $ros2_container_base_dir
    docker compose down || true
    docker compose up -d
    if [ "$install_dependencies" = "yes" ]; then
        docker exec $ros2_container_name bash -c "
            source /opt/ros/humble/setup.bash &&
            rosdep init || true &&
            apt update &&
            rosdep update &&
            if [ "$build_micro_ros_agent" = "yes" ]; then
                cd $microros_ws_target_dir_in_docker &&
                rosdep install --from-paths install --dependency-types exec &&
            fi &&
            if [ "$build_application" = "yes" ]; then
                cd $our_ws_target_dir_in_docker &&
                rosdep install --from-paths install --dependency-types exec &&
            fi "
    else
        echo "Dependencies will NOT be installed on the target device."
    fi
EOF

#!!!!!!!!!!!!!!  TODO ISSUE. når vi upper docker containeren ovenfor, så starter den nodesne... selvom vi måske ikke har installeret dependencies endnu.


# # Optionally install dependencies on the target.
# # we only want to ssh once... thats why the if statements are a bit complex.
# if [ "$install_dependencies" = "yes" ]; then
#     if [[ "$build_application" = "yes" && "$build_micro_ros_agent" = "no" ]]; then
#         ssh -t $pi_hostname "cd $our_ws_target_dir && \
#             sudo rosdep init || true && \
#             sudo apt update && \
#             rosdep update && \
#             rosdep install --from-paths install --dependency-types exec && \
#             exit"
#     fi
#     if [[ "$build_application" = "no" && "$build_micro_ros_agent" = "yes" ]]; then
#         ssh -t $pi_hostname "cd $microros_ws_target_dir && \
#             sudo rosdep init || true && \
#             sudo apt update && \
#             rosdep update && \
#             rosdep install --from-paths install --dependency-types exec && \
#             exit"
#     fi
#     if [[ "$build_application" = "yes" && "$build_micro_ros_agent" = "yes" ]]; then
#         ssh -t $pi_hostname "cd $microros_ws_target_dir && \
#             sudo rosdep init || true && \
#             sudo apt update && \
#             rosdep update && \
#             rosdep install --from-paths install --dependency-types exec && \
#             cd $our_ws_target_dir && \
#             sudo rosdep init || true && \
#             sudo apt update && \
#             rosdep update && \
#             rosdep install --from-paths install --dependency-types exec && \
#             exit"
#     fi
#     if [[ "$build_application" = "no" && "$build_micro_ros_agent" = "no" ]]; then
#         echo "Error, you need to build either the application or the micro-ROS agent to install dependencies."
#         exit 1
#     fi
# else
#     echo "Dependencies will NOT be installed on the target device."
# fi

