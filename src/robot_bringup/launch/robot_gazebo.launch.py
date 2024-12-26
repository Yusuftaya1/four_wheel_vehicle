from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_path = LaunchConfiguration('urdf_path')
    rviz_config_path = LaunchConfiguration('rviz_config_path')

    # Declare arguments
    declare_urdf_path_cmd = DeclareLaunchArgument(
        'urdf_path',
        default_value=os.path.join(
            get_package_share_directory('four_wheel_vehicle'), 'urdf', 'vehicle.urdf.xacro'
        ),
        description='Path to the URDF file'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_config_path',
        default_value=os.path.join(
            get_package_share_directory('robot_bringup'), 'rviz', 'gzb_rviz.rviz'
        ),
        description='Path to the RViz config file'
    )

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', urdf_path])}]
    )

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
            )
        )
    )

    # Spawn entity node
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot'],
        output='screen'
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # Return LaunchDescription
    return LaunchDescription([
        declare_urdf_path_cmd,
        declare_rviz_config_path_cmd,
        robot_state_publisher_node,
        gazebo_launch,
        spawn_entity_node,
        rviz_node
    ])
