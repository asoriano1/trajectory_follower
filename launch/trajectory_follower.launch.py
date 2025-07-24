from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Encuentra la ruta absoluta al YAML de waypoints
    config_path = os.path.join(
        get_package_share_directory('trajectory_follower'),
        'config',
        'waypoints.yaml'
    )

    return LaunchDescription([
        Node(
            package='trajectory_follower',
            executable='trajectory_follower',
            name='trajectory_follower',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[config_path]
        )
    ])

