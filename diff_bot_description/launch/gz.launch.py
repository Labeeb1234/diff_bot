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
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    share_dir = get_package_share_directory('diff_bot_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'diff_bot.xacro')
    robot_doc = xacro.parse(open(xacro_file))
    xacro.process_doc(robot_doc)
    robot_description = robot_doc.toxml()

    bridge_params = os.path.join(share_dir, 'config/bridge_params.yaml')

    use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='use simulation clock if set to true' 
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    gz_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            share_dir,
            'worlds',
            f'empty.sdf -r', 
        ])}.items(),
    )

    urdf_spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_doc.toxml(),
                   '-name', 'diff_bot',
                   '-allow_renaming', 'true',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '-0.0626'
        ]
    )

    ign_ros2_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )


    ld = LaunchDescription()
    ld.add_action(use_sim_time_cmd)
    ld.add_action(ign_ros2_bridge)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(gz_launcher)
    ld.add_action(urdf_spawn_node)

    return ld
