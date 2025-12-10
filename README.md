THIS CODE IS NOT COMPLETE

# ros2-droneswarm-hw-ws
ROS2 workspace made for a Ardupilot + Raspberry Pi setup. The Pi is running ROS2.

# Get Started (basic)
1. Clone repo:
    - `git clone --recursive https://github.com/Galfy1/ros2-droneswarm-hw-ws.git`
    - `cd ros2-droneswarm-hw-ws`
1. Install ROS2 Humble (if you don't already have it installed)
    - Go to: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html and follow the **"Setup Sources"** and **"Install ROS 2 packages"** sections
1. Install dependencies (stand in /ros2-droneswarm-hw-ws):
    - `sudo apt update`
    - `sudo rosdep init`
    - `rosdep update`
    - `source /opt/ros/humble/setup.bash`
    - `rosdep install -i --from-path src --rosdistro humble -y`
    - `sudo apt install python3-pip`
    - `python3 -m pip install -r requirements.txt `
1. Setup some required packages for Ardupilot build environment (stand in /ros2-droneswarm-hw-ws):
    - `cd src/ardupilot`
    - `Tools/environment_install/install-prereqs-ubuntu.sh -y`
    - `. ~/.profile`
1. Build MicroXRCEDDSGen (stand in /ros2-droneswarm-hw-ws):
    -  `cd src/Micro-XRCE-DDS-Gen/`
    -  `./gradlew assemble`
    -  `echo "export PATH=\$PATH:$PWD/scripts" >> ~/.bashrc`
    -  `source ~/.bashrc`
1. Build packages (stand in /ros2-droneswarm-hw-ws):
    - `source /opt/ros/humble/setup.bash`
    - `colcon build --packages-up-to ardupilot_dds_tests`
1. Build ROS2 workspace (stand in /ros2-droneswarm-hw-ws):
    - `source /opt/ros/humble/setup.bash`
    - `colcon build --packages-up-to droneswarm`
  

## How to Run / Launch
1. Open a new terminal (never run/launch in the same terminal you build the workspace)
1. Source overlay:
    - `cd {path to the workspace root}`
    - `source install/setup.bash`
1. Run or Launch whatever you want. e.g.:
    - `ros2 launch {package name} {launch file name}`

# Get Started (for use on Raspberry Pi)
1. Clone repo:
    - `git clone --recursive https://github.com/Galfy1/ros2-droneswarm-hw-ws.git`
1. Install Linux on the SD card using “Raspberry Pi Imager”
    - Raspberry Pi Imager can be found: https://www.raspberrypi.com/software/ 
    - Select “Raspberry Pi Zero 2W”
    - Select “Raspberry Pi OS Lite (64 bit)”
    - Select your SD-card
    - For “OS Customisation”:
        - In “General tap”
            - Set hostname to “raspberrypi”
            - Set username to "devboard"
            - Set password (don’t use an empty password)
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
1. Install and setup ROS2 Docker on Pi:
    - (this is done on the computer, not the Pi)
    - Run the following scripts in the following order (all scripts can be found in this repo)
    - Stand in the "ros2_in_docker_for_pi" folder
        - `./install_docker_on_pi.sh`
        - `./copy_ros2_os_to_pi.sh`
    - Stand in the root of this repo
        - `./build_copy_start_on_pi.sh yes yes yes no`
1. Setup Pi to auto start ROS2 nodes:
    - SSH into the pi using:
        - `ssh devboard@raspberrypi.local`
    - Create a systemd service to auto start the Micro ROS Agent and the droneswarm (place the following content in each file):
        - `mkdir -p ~/.config/systemd/user`
        - `nano ~/.config/systemd/user/micro-ros-agent.service`
            - ```
              [Unit]
              Description=Start Micro-ROS Agent over serial /dev/ttyS0 (in docker)
              After=network-online.target
              Wants=network-online.target

              [Service]
              WorkingDirectory=/home/devboard
              Type=simple
              ExecStart=/home/devboard/.local/lib/start-micro-ros-agent.sh
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
              WorkingDirectory=/home/devboard
              Type=simple
              ExecStart=/home/devboard/.local/lib/start-our-ws.sh
              Restart=on-failure

              [Install]
              WantedBy=default.target
              ```
    - Create the startup scripts that launch the ROS2 nodes (place the following content in each file):
        - `mkdir -p ~/.local/lib/`
        - `nano ~/.local/lib/start-micro-ros-agent.sh`
            - ```
              #!/bin/bash

              ros2_container_name="ros2_droneswarm-ros2-1"
              microros_ws_target_dir_in_docker="/root/ros2_droneswarm/workspaces/microros_ws"

              sudo docker exec $ros2_container_name bash -c "
                cd $microros_ws_target_dir_in_docker &&
                source install/setup.bash &&
                ros2 run micro_ros_agent micro_ros_agent serial -v4 -b 115200 -D /dev/ttyAMA0 "
              ```
        - `nano ~/.local/lib/start-our-ws.sh`
            - ```
              #!/bin/bash

              ros2_container_name="ros2_droneswarm-ros2-1"
              our_ws_target_dir_in_docker="/root/ros2_droneswarm/workspaces/our_ws"
              main_ros2_package_name="droneswarm"
              main_ros2_launchfile="droneswarm.launch.py"

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
1. Setup UART:
   - SSH into the pi using:
        - `ssh devboard@raspberrypi.local`
   - Enable UART: 
        - Open config tool (works in headless also):
            - `sudo raspi-config`
        - in the utility, select "Interfacing Options" --> "Serial":
            - When prompted, select "no" to “Would you like a login shell to be accessible over serial?”
            - When prompted, select "yes" to “Would you like the serial port hardware to be enabled?”.
   - Disable Bluetooth (so that the UART0 can use the primary UART hardware - i.e. /dev/ttyAMA0)
        - Open a terminal in the Pi Docker:
            - `sudo docker exec "ros2_droneswarm-ros2-1 bash`
        - Open config file:
            - `sudo nano /boot/firmware/config.txt`
        - Add the following line:
            - `dtoverlay=disable-bt`
        - Save and close the file
    - Now, Reboot the Pi.
1. Setup Pi AI Cam drivers/tcp_publisher on Pi (outside of its docker)
    - SSH into the pi using:
        - `ssh devboard@raspberrypi.local`
    - Clone the source code:
        - `git clone https://github.com/alroe19/droneswarm_host_side.git`
        - `cd droneswarm_host_side`
        - `sudo ./install_dependencies.sh`
 

## How to update the Pi when you change the source code?
Simply run ./build_copy_start_on_pi.sh with your desired yes/no flag options:
 - `build_copy_start_on_pi.sh <build_application> <build_micro_ros_agent> <install_dependencies> <force_rebuild_of_ros2_docker>`
     - _<build_application>_: Build the droneswarm workspace.
     - _<build_micro_ros_agent>_: Build the micro ros agent. You probably only want to set this to "yes" if there are updates to the micro ros agent.
     - _<install_dependencies>_: Install dependencies on the Pi. You need to set this to "yes" if you add to the dependencies of a ROS2 package. Dependencies will only be installed in the workspaces that are specified with "yes" in the <build…> flags.
     - _<force_rebuild_of_ros2_docker>_: Force a rebuild of the ros2 Docker Compose/Dockerfile. If the ros2 docker container is already running on the pi, changes to the ros2 dockerfile or docker-compose file will NOT be applied automatically. Therefore, you need to force re-build the docker if you have made changes to the ros2 dockerfile or docker-compose file. If you set force_rebuild_of_ros2_docker to yes, you probably also want to set install_dependencies to yes (since all installed dependencies are removed as the docker compose is brought down and up again).

A Typical command will look like this:
- `./build_copy_start_on_pi.sh yes no no no`

_Note: don't use `sudo` in front of `build_copy_start_on_pi.sh`! (doing so will cause directories created on the Pi to become root-owned, which can lead to permission issues)._

If you make any changes to the Dockerfile or docker-compose.yml file in the "ros2_in_docker_for_pi" folder, you need to:
- Run the copy_ros2_os_to_pi.sh script.
- Run the build_copy_start_on_pi.sh scipt with <force_rebuild_of_ros2_docker> set to "yes".

  
# About Dependencies

To add a dependency:
- add it to package.xml within the ROS2 package

For a list of Python packages in rosdep, see:
- https://docs.ros.org/en/humble/How-To-Guides/Using-Python-Packages.html

For a list of general ROS2 packages, see:
- [ROS Index](https://index.ros.org/?search_packages=true)
- (you can also depend on these in the package.xml file)
  
# If Using WSL in Windows
- Make sure to set “Networking Mode” to “mirrored” in the "WSL Settings" app (to allow for SSH to the Pi)
- Make sure you don't have Docker Engine running on Windows before starting WSL! If you don’t do this, WSL will use your Windows Docker Engine!
    - (you can check/close the docker engine in the Windows system tray)
    - (if you had docker engine running in Windows before starting WSL, you need to close the docker engine, and then restart WSL using “WSL --shutdown”)
    - (you can check if your WSL is using Windows or Linux Docker Engine by typing “docker info” and looking for the “operating system” line. If it says “: Docker Desktop”, its using the Windows Docker Engine)
- If you incounter failed "concol build ..." commands, , try increasing the WSL memory in the "WSL Settings" Windows app
 
# Additional Setup for Gazebo Simulation
If you need to run an ArduPilot/ROS2/Gazebo simulation, you need the additional setup covered in this section.  
It is assumed that you have followed [Get Started (basic)](#get-started-basic).

1. Install some required Python packages
    -  `sudo apt update`
    -  `python3 -m pip install pexpect`
1. Build Ardupilot SITL for ROS2 (stand in /ros2-droneswarm-hw-ws)
    -  `source /opt/ros/humble/setup.bash`
    -  `colcon build --packages-up-to ardupilot_sitl`
1. Setup ROS2 with Gazebo (stand in /ros2-droneswarm-hw-ws)
    1. Install Gazebo: https://gazebosim.org/docs/harmonic/install_ubuntu/
    1.  Setup ROS2+Gazebo:
        -  ```
           vcs import --input https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos --recursive src
           export GZ_VERSION=harmonic
           sudo apt install wget
           sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
           echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
           sudo apt update
           sudo wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list -O /etc/ros/rosdep/sources.list.d/00-gazebo.list
           rosdep update
           source /opt/ros/humble/setup.bash
           sudo apt update
           rosdep update
           rosdep install --from-paths src --ignore-src -y
           ```
1. Build ardupilot_gz_bringup (stand in /ros2-droneswarm-hw-ws)
    -  `source /opt/ros/humble/setup.bash`
    -  `colcon build --packages-up-to ardupilot_gz_bringup`
    -  (if colcon build fails... simply try again. If it still fails, and you use WSL, try increasing the WSL memory in the "WSL Settings" Windows app)
  
*NOTE: If you use WSL, ROS2 topics might not work correctly unless you set the WSL Networking Mode to "None" (within the Windows WSL Settings app)... for some reason...*
  
## How to Run / Launch a Single-UAV Simulation
1. Open a new terminal (never run/launch in the same terminal you build the workspace)
1. Source overlay:
    - `cd {path to the workspace root}`
    - `source install/setup.bash`
1. Launch the simulations environment using ardupilot_gz_bringup:
    - `ros2 launch ardupilot_gz_bringup iris_runway.launch.py`
    - (feel free to use the other bring-up launch files for different copter and world setups)
1. Run or Launch whatever you want. e.g.:
    - `ros2 launch {package name} {launch file name}`
  

# Known Issues Regarding Pi Setup
This section outlines known issues related to the scripts in this repo that are used to set up, cross-compile, copy files, etc. to the Pi.

## Redundant Micro-ROS Agent
The scripts create and use a Micro-ROS agent in its own workspace on the Pi. However, we also have a Micro-ROS agent package in our droneswarm workspace that is not being used. Using the Micro-ROS agent within the droneswarm workspace instead could simplify the setup, dependency installation, and related processes.

## Behavior of <force_rebuild_of_ros2_docker> and Dependencies
Using `<force_rebuild_of_ros2_docker>` will remove all dependencies installed in the Docker container running on the Pi, requiring them to be reinstalled.

## Regarding the "autostart files" Using systemd
These files are not fully tested.

## Issues Related to Automatic Installation of Dependencies
The `<install_dependencies>` functionality in `build_copy_start_on_pi.sh` does not fully work.

# Troubleshooting tips
- If you get "credential" issues while running the bash scripts, try running them with sudo. If that does not work: in ~/.docker/config.json change credsStore to credStore.
- If you get "exec /bin/sh: exec format error" while running  `build_copy_start_on_pi.sh`, you QEMU is proabably broken - try installing it again using the command mentioned earlier in this readme file.
- If you encounter weird build errors when using `build_copy_start_on_pi.sh`, your system may not have enough resources. Try building `<build_application>` and `<build_micro_ros_agent>` separately instead of running both at the same time.





