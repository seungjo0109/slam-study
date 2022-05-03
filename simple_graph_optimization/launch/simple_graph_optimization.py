from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os.path

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='simple_graph_optimization',
            executable="simple_graph_optimization",
            output="screen"
        ),
        Node(
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(os.getcwd(), 'rviz', 'simple_graph_optimization.rviz')]
        )
    ])