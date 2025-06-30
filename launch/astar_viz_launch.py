# launch/astar_viz_launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('hybrid_astar')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        # Python environment publisher: just publishes map, start & goal
        Node(
            package='hybrid_astar',
            executable='env_publisher.py',
            name='env_publisher',
            output='screen'
        ),

        # C++ Hybrid A* planner node
        Node(
            package='hybrid_astar',
            executable='hybrid_astar_node',
            name='hybrid_astar_node',
            output='screen',
            # parameters=[params_file]
        ),

        # Python Matplotlib visualizer: subscribe to /map and /planned_path
        Node(
            package='hybrid_astar',
            executable='path_viz.py',
            name='path_viz',
            output='screen'
        ),
    ])
