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
    #controller_params = os.path.join(share_dir, 'config', 'my_controller.yaml')


    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='use simulation clock if set to true'
    )


    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'pause': 'true'
        }.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'diff_bot',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    diff_bot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_bot_controller', '--controller-manager', '/controller_manager']
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad', '--controller-manager', '/controller_manager']
    )

    static_state_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link_tf',
        output='screen',
        arguments=['0.0','0.0','0.0','0.0','0.0','0.0', 'odom', 'map']
    )

    return LaunchDescription([
        use_sim_time_cmd,
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo_server,
        gazebo_client,
        urdf_spawn_node,
        joint_broad_spawner,
        diff_bot_controller_spawner,
        static_state_transform_publisher_node,        
    ])