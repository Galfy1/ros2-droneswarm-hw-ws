THIS CODE IS NOT COMPLETE

# ros2-droneswarm-hw-ws
ROS2 workspace made for a Ardupilot + Raspberry Pi setup. The Pi is running ROS2.

# Get started
1. Clone repo:
    - git clone {this repo}
    - cd ros2-droneswarm-hw-ws
1. Install dependencies (stand in /ros2-droneswarm-hw-ws):
    - sudo apt update
    - rosdep update
    - source /opt/ros/humble/setup.bash
    - rosdep install -i --from-path src --rosdistro humble -y
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
