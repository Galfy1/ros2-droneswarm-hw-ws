from launch import LaunchDescription
from launch_ros.actions import Node
#from launch.actions import DeclareLaunchArgument
#from launch.substitutions import LaunchConfiguration



def generate_launch_description():

    return LaunchDescription([
        Node(
            package='droneswarm',
            namespace='drone',
            executable='copter_controller_node',
            name='copter_controller_node',
            output='screen',
        ),
    ])