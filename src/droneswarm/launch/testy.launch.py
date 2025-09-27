from launch import LaunchDescription
from launch_ros.actions import Node
#from launch.actions import DeclareLaunchArgument
#from launch.substitutions import LaunchConfiguration



def generate_launch_description():

    return LaunchDescription([
        Node(
            package='py_playground',
            namespace='drone',
            executable='testy_node',
            name='drone',
        ),
    ])