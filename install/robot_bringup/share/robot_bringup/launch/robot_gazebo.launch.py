import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    urdf_path = LaunchConfiguration('urdf_path', default=os.path.join(get_package_share_directory('four_wheel_vehicle'), 'urdf', 'vehicle.urdf.xacro'))
    rviz_config_path = LaunchConfiguration('rviz_config_path', default=os.path.join(get_package_share_directory('robot_bringup'), 'rviz', 'gzb_rviz.rviz'))
    world_path = LaunchConfiguration('world_path', default=os.path.join(get_package_share_directory('robot_bringup'), 'world', 'world_farm.world'))
    map_path = LaunchConfiguration('map_path', default=os.path.join(get_package_share_directory('robot_bringup'), 'map', 'farm_map.yaml'))


    return LaunchDescription([
        DeclareLaunchArgument('urdf_path', default_value=urdf_path, description='Path to URDF file'),
        DeclareLaunchArgument('rviz_config_path', default_value=rviz_config_path, description='Path to RViz config file'),
        DeclareLaunchArgument('world_path', default_value=world_path, description='Path to world file'),
        DeclareLaunchArgument('map_path', default_value=map_path, description='Path to map file'),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('robot_bringup'), 'launch', 'gazebo_launch.py')),
            launch_arguments={'world': world_path}.items()
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': launch.substitutions.Command(['xacro ', urdf_path]), 'use_sim_time': True}]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(FindPackageShare('gazebo_ros').find('gazebo_ros'), 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': world_path}.items()),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            arguments=['-topic', 'robot_description', '-entity', 'vehicle'],
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': True}]
        ),
    ])
