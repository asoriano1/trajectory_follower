from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_dir = [get_package_share_directory('trajectory_follower'), 'config']

    yaml_filename_arg = DeclareLaunchArgument(
        'yaml_filename',
        default_value='grid_map_waypoints.yaml',
        description='Nombre del archivo YAML con los waypoints'
    )

    yaml_path = PathJoinSubstitution([
        *config_dir,
        LaunchConfiguration('yaml_filename')
    ])

    return LaunchDescription([
        yaml_filename_arg,
        Node(
            package='trajectory_follower',
            executable='trajectory_follower',  # O solo 'trajectory_follower' si tienes el ejecutable compilado
            name='trajectory_follower',
            output='screen',
            arguments=[yaml_path],  # <= Tu nodo espera el YAML como argumento
            parameters=[{'use_sim_time': True}]
        )
    ])
