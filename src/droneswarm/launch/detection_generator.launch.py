from launch import LaunchDescription
from launch_ros.actions import Node
#from launch.actions import DeclareLaunchArgument
#from launch.substitutions import LaunchConfiguration



def generate_launch_description():

    return LaunchDescription([
        Node(
            package='droneswarm',
            namespace='drone',
            executable='detection_generator_node',
            name='detection_generator_node',
            output='screen',
        ),
    ])