from launch import LaunchDescription
from launch_ros.actions import Node
#from launch.actions import DeclareLaunchArgument
#from launch.substitutions import LaunchConfiguration



def generate_launch_description():

    return LaunchDescription([
        Node(
            package='droneswarm',
            namespace='drone',
            executable='tcp_camera_bridge_node',
            name='tcp_camera_bridge_node',
            output='screen',
        ),
    ])