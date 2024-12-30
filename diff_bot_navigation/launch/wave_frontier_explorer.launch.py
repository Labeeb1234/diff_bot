import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    exploration_algo_dir = get_package_share_directory('explore_lite')
    nav2_dir = get_package_share_directory('diff_bot_navigation')
    exploration_params = os.path.join(nav2_dir, 'config/', 'explore_params.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description="Use simulation/Gazebo clock"
    )

    exploration_algo_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(exploration_algo_dir, 'launch/', 'explore.launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': exploration_params
        }.items()

    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(exploration_algo_launcher)
    
    return ld