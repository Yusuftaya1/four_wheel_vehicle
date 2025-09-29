import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    urdf_path = LaunchConfiguration(
        'urdf_path',
        default=os.path.join(
            get_package_share_directory('four_wheel_vehicle'),
            'urdf',
            'vehicle.urdf.xacro'
        ),
    )

    world_path = os.path.join(
        get_package_share_directory('robot_bringup'),
        'world',
        'world_farm.world',
    )

    rviz_config_path = LaunchConfiguration(
        'rviz_config_path',
        default=os.path.join(
            get_package_share_directory('robot_bringup'),
            'rviz',
            'gzb_rviiz.rviz',
        ),
    )

    controllers_yaml = os.path.join(
        get_package_share_directory('four_wheel_vehicle'),
        'config',
        'controller.yaml',
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world_path,
            'verbose': 'true',
            'debug': 'true',
        }.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        ),
    )

    load_forward_position_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'forward_position_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', controllers_yaml,
        ],
        output='screen',
    )

    load_forward_velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'forward_velocity_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', controllers_yaml,
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('urdf_path', default_value=urdf_path, description='Path to URDF file'),
        DeclareLaunchArgument('rviz_config_path', default_value=rviz_config_path, description='Path to RViz config file'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': launch.substitutions.Command(['xacro ', urdf_path]), 'use_sim_time': True}],
        ),
        gzserver,
        gzclient,

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            arguments=[
                '-topic', '/robot_description',
                '-entity', 'vehicle',
                '-x', '0.0', '-y', '0.0', '-z', '0.0',
                '-Y', '0',
            ],
            parameters=[{'use_sim_time': True}],
        ),
        load_forward_position_controller,
        load_forward_velocity_controller,
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': True}],
        ),
    ])