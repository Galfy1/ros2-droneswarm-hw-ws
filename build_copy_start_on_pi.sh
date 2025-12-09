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
if [[ -z "$4" || ( "$4" != "yes" && "$4" != "no" ) ]]; then
    echo "Error in fourth arg: Missing argument or invalid value. <force_rebuild_of_ros2_docker> (values: yes or no). 
          If the ros2 docker container is already running on the pi, changes to the ros2 dockerfile or docker-compose file will NOT be applied automatically.
          Therefore, you need to force re-build the docker if you have made changes to the ros2 dockerfile or docker-compose file.
          If you set force_rebuild_of_ros2_docker to yes, you probably also want to set install_dependencies to yes 
          (since all installed dependencies are removed as the docker compose is brought down and up again)."
    exit 1
fi

# Get the arguments
build_application=$1
build_micro_ros_agent=$2
install_dependencies=$3
force_rebuild_of_ros2_docker=$4

# create variables
pi_username="devboard"
pi_hostname="raspberrypi.local" # this is the assumed hostname of the pi, change if needed.

ros2_container_base_dir="~/ros2_droneswarm"
our_ws_target_dir="$ros2_container_base_dir/workspaces/our_ws"
microros_ws_target_dir="$ros2_container_base_dir/workspaces/microros_ws"
ros2_container_name="ros2_droneswarm-ros2-1" # (a name given by docker. cus its in the ros2_droneswarm folder and is called ros2 in the compose file. the "1" is cus its the first container of that name in the compose)
# # main_ros2_package_name="droneswarm"
# # main_ros2_launchfile="tsunami_swarm.launch.py"  # our main "application" launch file to run inside the container

# Make sure any already existing container named "dummy" is removed before we start.
sudo docker rm -f dummy || true  # "|| true" will ignore error if dummy does not exist (remember, we are usings "set -e" at the top of the script)
                            # this will give an "ERROR: default sources list file already exists" error if the container does not exist - we can ignore that error!

# Build with multiplatform support.
sudo docker build . --platform linux/arm64 -t build_image \
    --build-arg BUILD_MICRO_ROS_AGENT=$build_micro_ros_agent \
    --build-arg BUILD_OUR_ROS2_WS=$build_application 
# Create a dummy container from the image
sudo docker create --name dummy build_image
# Create a temporary directories to hold the installation files.
sudo rm -rf temp_our_ws temp_micro_agent  # Remove any existing temp directory to avoid conflicts.
mkdir -p temp_our_ws/install temp_micro_agent/install  # Create the temp directory.

# Only copy to target if it was actually built.
if [ "$build_application" = "yes" ]; then
    # Copy the installation directory from the docker image to the host.
    sudo docker cp dummy:/our_ros2_ws/install/. temp_our_ws/install 
    # And now, from host to target's directory, but ignore COLCON_IGNORE.
    rsync -av --mkpath --exclude temp_our_ws/install/COLCON_IGNORE temp_our_ws/install $pi_username@$pi_hostname:$our_ws_target_dir
fi
if [ "$build_micro_ros_agent" = "yes" ]; then
    sudo docker cp dummy:/micro-ROS-Agent/install/. temp_micro_agent/install
    rsync -av --mkpath --exclude temp_micro_agent/install/COLCON_IGNORE temp_micro_agent/install $pi_username@$pi_hostname:$microros_ws_target_dir
fi

sudo docker rm -f dummy

# Clean up the temporary directories.
sudo rm -rf temp_our_ws
sudo rm -rf temp_micro_agent

# # The paths that the docker volumes are mapped to INSIDE THE DOCKER CONTAINER (defined in docker-compose.yml)
microros_ws_target_dir_in_docker=/root/ros2_droneswarm/workspaces/microros_ws
our_ws_target_dir_in_docker=/root/ros2_droneswarm/workspaces/our_ws

# Step 1: Re-run the container (down/up cus of compose) to make sure any previous ros2 nodes are stopped 
#         (--build the container when upping, to make sure any changes in its docker/compose file is applyed).
# Step 2: (optional) Install dependencies inside the container.
# Step 3: Reboot Pi. This will ensure that nodes are relaunched using new build files (nodes are launched in a systemd service at boot - cus we set it up that way).
# (https://www.cyberciti.biz/faq/unix-linux-execute-command-using-ssh/)
ssh $pi_username@$pi_hostname << EOF   # (no quotes around EOF to allow variable expansion on local machine)
    cd $ros2_container_base_dir
    if [ "$force_rebuild_of_ros2_docker" = "yes" ]; then
        sudo docker compose down || true
        sudo docker compose up --build -d
    else
        sudo docker compose up -d
    fi
    if [ "$install_dependencies" = "yes" ]; then
        sudo docker exec $ros2_container_name bash -c "
            source /opt/ros/humble/setup.bash &&
            apt update &&
            rosdep update &&
            if [ "$build_micro_ros_agent" = "yes" ]; then
                cd $microros_ws_target_dir_in_docker &&
                rosdep install --from-paths install --dependency-types exec &&
                apt install -y ros-humble-micro-ros-msgs
            fi
            if [ "$build_application" = "yes" ]; then
                cd $our_ws_target_dir_in_docker &&
                rosdep install --from-paths install --dependency-types exec
            fi "
    else
        echo "Dependencies will NOT be installed on the target device."
    fi
    # sudo docker exec $ros2_container_name bash -c "
    #     cd $microros_ws_target_dir_in_docker &&
    #     source install/setup.bash &&
    #     ros2 run micro_ros_agent micro_ros_agent serial -v4 -b 115200 -D /dev/ttyS0 &&
    #     cd $our_ws_target_dir_in_docker &&
    #     source install/setup.bash &&
    #     ros2 launch $main_ros2_package_name $main_ros2_launchfile "
    sudo reboot
EOF

# Regarding the "ros-humble-micro-ros-msgs package in the above script:
    # The package is a dependency for micro-ROS-Agent. It's listed as a dependency in micro-ROS-Agent's dependency list (package.xml), 
    # but rosdep cannot seem to locate the micro-ros-msgs package, even though it should be included in the ROS index:  https://index.ros.org/p/micro_ros_msgs/#humble-overview   
    # Note: While rosdep seem to  install all dependencies for the micro-ROS-Agent (no errors), 
    #       running the agent may still fail with an error indicating 
    #       that the micro-ros-msgs package is missing. Manual checks via apt/rosdep 
    #       confirm that it is indeed not installed (idk why...). Therefore, we install it manually..


# TODO den skal også også kunne install requirements.txt dependencies fra vores workspace... (det kræver ogås requirements.txt er i install foldere.. ikke bare i source koden)

# Debugging info: use "sudo docker exec -it ros2_droneswarm-ros2-1 /bin/bash" to get a bash terminal in the running ros2 container on the pi.
