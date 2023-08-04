from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share_dir = get_package_share_directory('diff_bot_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'diff_bot.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    nav2_params_file = os.path.join(share_dir, 'config/nav2_params.yaml')
    map_file = os.path.join(share_dir, 'maps/turtlebot3_world.yaml')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )
    navigation_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'

            ])
        ]),
        launch_arguments={
            'map': map_file,
            'params_file': nav2_params_file,
            'auto_start': 'true'
        }.items()
    )

    return LaunchDescription(
        [
            #rviz_node,
            navigation_stack,

        ])

