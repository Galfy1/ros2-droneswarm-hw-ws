THIS CODE IS NOT COMPLETE

# ros2-droneswarm-hw-ws
ROS2 workspace made for a Ardupilot + Raspberry Pi setup. The Pi is running ROS2.

# Get Started (basic)
1. Clone repo:
    - `git clone {this repo}`
    - `cd ros2-droneswarm-hw-ws`
1. Install dependencies (stand in /ros2-droneswarm-hw-ws):
    - `sudo apt update`
    - `rosdep update`
    - `source /opt/ros/humble/setup.bash`
    - `rosdep install -i --from-path src --rosdistro humble -y`
    - `python3 -m pip install -r requirements.txt `
1. Build ROS2 workspace (stand in /ros2-droneswarm-hw-ws):
    - `source /opt/ros/humble/setup.bash`
    - `colcon build --packages-up-to droneswarm`
  
NOTE: If rosdep... outputs an error, you might need to call "sudo rosdep init" and try again

## How to Run / Launch
1. Open a new terminal (never run/launch in the same terminal you build the workspace)
1. Source overlay:
    - `cd {path to the workspace root}`
    - `source install/setup.bash`
1. Run or Launch whatever you want. e.g.:
    - `ros2 launch {package name} {launch file name}`

# Get Started (for use on Raspberry Pi)
1. Install Linux on the SD card using “Raspberry Pi Imager”
    - Raspberry Pi Imager can be found: https://www.raspberrypi.com/software/ 
    - Select “Raspberry Pi Zero 2W”
    - Select “Raspberry Pi OS Lite (64 bit)”
    - Select your SD-card
    - For “OS Customisation”:
        - In “General tap”
            - Set hostname to “raspberrypi”
            - Set username and password (don’t use an empty password)
            - Configure wireless LAN
        - In “Services” tap:
            - Enable SSH
    - Flash the SD-card
1. Setup computer for Arm cross-compilation:
    - It is assumed that you are using a Linux machine.
        - (if you use WSL, see "If Using WSL in Windows" section bellow) 
    - If you don't already have docker, install it from: https://docs.docker.com/engine/install/ubuntu/ (example link for Ubuntu)
    - Follow the sections:
        - “Uninstall old versions”
        - “Install using the apt repository”
    - Install QEMU for Docker Multi-platform builds:
        - `docker run --privileged --rm tonistiigi/binfmt --install all`
1. Setup Pi to auto start ROS2 nodes:
    - SSH into the pi using:
        - `ssh raspberrypi.local`
    - (In the following scripts, remember to replace `<pi_username\>` with the username of the Pi)
    - Create a systemd service to auto start the Micro ROS Agent and the droneswarm (place the following content in each file):
        - `mkdir -p ~/.config/systemd/user`
        - `nano ~/.config/systemd/user/micro-ros-agent.service`
            - ```
              [Unit]
              Description=Start Micro-ROS Agent over serial /dev/ttyS0 (in docker)
              After=network-online.target
              Wants=network-online.target

              [Service]
              WorkingDirectory=/home/<pi_username>
              Type=simple
              ExecStart=/home/<pi_username>/.local/lib/start-micro-ros-agent.sh
              Restart=on-failure

              [Install]
              WantedBy=default.target
              ```
        - `nano ~/.config/systemd/user/our-ws.service`
            - ```
              [Unit]
              Description=Used to autostart a custom ROS2 application (i.e. our_ws) (in docker)
              After=network-online.target micro-ros-agent.service
              Wants=network-online.target

              [Service]
              WorkingDirectory=/home/<pi_username>
              Type=simple
              ExecStart=/home/<pi_username>/.local/lib/start-our-ws.sh
              Restart=on-failure

              [Install]
              WantedBy=default.target
              ```
    - Create the startup scripts that launch the ROS2 nodes (place the following content in each file):
        - `nano ~/.local/lib/start-micro-ros-agent.sh`
            - ```
              #!/bin/bash

              ros2_container_name="ros2_droneswarm-ros2-1"
              microros_ws_target_dir_in_docker="/root/ros2_droneswarm/workspaces/microros_ws"

              sudo docker exec $ros2_container_name bash -c "
                cd $microros_ws_target_dir_in_docker &&
                source install/setup.bash &&
                ros2 run micro_ros_agent micro_ros_agent serial -v4 -b 115200 -D /dev/ttyS0 "
              ```
        - `nano ~/.local/lib/start-our-ws.sh`
            - ```
              #!/bin/bash

              ros2_container_name="ros2_droneswarm-ros2-1"
              our_ws_target_dir_in_docker="/root/ros2_droneswarm/workspaces/our_ws"
              main_ros2_package_name="droneswarm"
              main_ros2_launchfile="tsunami_swarm.launch.py"

              sudo docker exec $ros2_container_name bash -c "
                cd $our_ws_target_dir_in_docker &&
                source install/setup.bash &&
                ros2 launch $main_ros2_package_name $main_ros2_launchfile "
              ```
    - Finally, make the scripts executable, reload services, and start the new systemd services:
        - ```
          chmod +x ~/.local/lib/start-micro-ros-agent.sh
          systemctl --user daemon-reload
          systemctl --user enable micro-ros-agent.service
          systemctl --user start micro-ros-agent.service
          systemctl --user status micro-ros-agent.service
          chmod +x ~/.local/lib/start-our-ws.sh
          systemctl --user daemon-reload
          systemctl --user enable our-ws.service
          systemctl --user start our-ws.service
          systemctl --user status our-ws.service
          ```
    - Because systemctl user services won’t start until someone logs in, enable linger for login:
        - `loginctl enable-linger $USER`
1. Install and setup ROS2 Docker on Pi:
    - (this is done on the computer, not the Pi)
    - Run the following scripts in the following order (all scripts can be found in this repo)
    - Stand in the "ros2_in_docker_for_pi" folder
        - `./install_docker_on_pi.sh`
        - `./copy_ros2_os_to_pi.sh`
    - Stand in the root of this repo
        - `./build_copy_start_on_pi.sh yes yes yes no`

## How to update the Pi when you change the source code?
Simply run ./build_copy_start_on_pi.sh with your desired yes/no flag options:
 - `build_copy_start_on_pi.sh <build_application> <build_micro_ros_agent> <install_dependencies> <force_rebuild_of_ros2_docker>`
     - _<build_application>_: Build the droneswarm workspace.
     - _<build_micro_ros_agent>_: Build the micro ros agent. You probably only want to set this to "yes" if there are updates to the micro ros agent.
     - _<install_dependencies>_: Install dependencies on the Pi. You need to set this to "yes" if you add to the dependencies of a ROS2 package.
     - _<force_rebuild_of_ros2_docker>_: Force a rebuild of the ros2 Docker Compose/Dockerfile. If the ros2 docker container is already running on the pi, changes to the ros2 dockerfile or docker-compose file will NOT be applied automatically. Therefore, you need to force re-build the docker if you have made changes to the ros2 dockerfile or docker-compose file. If you set force_rebuild_of_ros2_docker to yes, you probably also want to set install_dependencies to yes (since all installed dependencies are removed as the docker compose is brought down and up again).

A Typical command will look like this:
- `./build_copy_start_on_pi.sh yes no no no`

If you make any changes to the Dockerfile or docker-compose.yml file in the "ros2_in_docker_for_pi" folder, you need to:
- Run the copy_ros2_os_to_pi.sh script.
- Run the build_copy_start_on_pi.sh scipt with <force_rebuild_of_ros2_docker> set to "yes".

  
# About Dependencies

To add a dependency:
- add it to package.xml within the ROS2 package

If you need a Python package that is NOT part of rosdep 
- (see https://docs.ros.org/en/humble/How-To-Guides/Using-Python-Packages.html for lists of available Python packages in rosdep):
- add it to requirements.txt
  
# If Using WSL in Windows
- Make sure to set “Networking Mode” to “mirrored” in the "WSL Settings" app (to allow for SSH to the Pi)
- Make sure you don't have Docker Engine running on Windows before starting WSL! If you don’t do this, WSL will use your Windows Docker Engine!
    - (you can check/close the docker engine in the Windows system tray)
    - (if you had docker engine running in Windows before starting WSL, you need to close the docker engine, and then restart WSL using “WSL --shutdown”)
    - (you can check if your WSL is using Windows or Linux Docker Engine by typing “docker info” and looking for the “operating system” line. If it says “: Docker Desktop”, its using the Windows Docker Engine)

# Miscellaneous Notes
- If you get "credential" issues while running the bash scripts, try running them with sudo. If that does not work: in ~/.docker/config.json change credsStore to credStore.
- If you get "exec /bin/sh: exec format error" while running the "cross compile" bash script, you Qemu might be broken - try installing it again using the command mentioned earlier in this readme file




