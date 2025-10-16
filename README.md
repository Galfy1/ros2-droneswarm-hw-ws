THIS CODE IS NOT COMPLETE

# ros2-droneswarm-hw-ws
ROS2 workspace made for a Ardupilot + Raspberry Pi setup. The Pi is running ROS2.

# Get Started
1. Clone repo:
    - git clone {this repo}
    - cd ros2-droneswarm-hw-ws
1. Install dependencies (stand in /ros2-droneswarm-hw-ws):
    - sudo apt update
    - rosdep update
    - source /opt/ros/humble/setup.bash
    - rosdep install -i --from-path src --rosdistro humble -y
    - python3 -m pip install -r requirements.txt
1. Build ROS2 workspace (stand in /ros2-droneswarm-hw-ws):
    - source /opt/ros/humble/setup.bash 
    - colcon build --packages-up-to droneswarm
  
NOTE: If rosdep... outputs an error, you might need to call "sudo rosdep init" and try again

# How to Run / Launch
1. Open a new terminal (never run/launch in the same terminal you build the workspace)
1. Source overlay:
    - cd {path to the workspace root}
    - source install/setup.bash
1. Run or Launch whatever you want. e.g.:
    - ros2 launch {package name} {launch file name}
  
# About Dependencies

To add a dependency:
- add it to package.xml within the ROS2 package

If you need a Python package that is NOT part of rosdep 
- (see https://docs.ros.org/en/humble/How-To-Guides/Using-Python-Packages.html for lists of available Python packages in rosdep):
- add it to requirements.txt
  
# Setup for build_and_copy_to_pi.sh script
It is assumed that you are using a Linux machine.
1. If you don't already have docker, install it from: https://docs.docker.com/engine/install/ubuntu/ (example link for Ubuntu)
    - Follow the sections:
        - “Uninstall old versions”
        - “Install using the apt repository”
1. Install QEMU for Docker Multi-platform builds:
    - docker run --privileged --rm tonistiigi/binfmt --install all
  
## If Using WSL in Windows
- Make sure to set “Networking Mode” to “mirrored” in the "WSL Settings" app (to allow for SSH to the Pi)
- Make sure you don't have Docker Engine running on Windows before starting WSL! If you don’t do this, WSL will use your Windows Docker Engine!
    - (you can check/close the docker engine in the Windows system tray)
    - (if you had docker engine running in Windows before starting WSL, you need to close the docker engine, and then restart WSL using “WSL --shutdown”)
    - (you can check if your WSL is using Windows or Linux Docker Engine by typing “docker info” and looking for the “operating system” line. If it says “: Docker Desktop”, its using the Windows Docker Engine)

## Miscellaneous Notes
- If you get "credential" issues while running the bash scripts, try running them with sudo. If that does not work: in ~/.docker/config.json change credsStore to credStore.
- If you get "exec /bin/sh: exec format error" while running the "cross compile" bash script, you Qemu might be broken - try installing it again using the command mentioned earlier in this readme file




