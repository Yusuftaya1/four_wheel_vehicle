import os
import launch 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory 

def generate_launch_description():
    urdf_path = LaunchConfiguration('urdf_path', default=os.path.join(get_package_share_directory('four_wheel_vehicle'), 'urdf', 'vehicle.urdf.xacro'))
    rviz_config_path = LaunchConfiguration('rviz_config_path', default=os.path.join(get_package_share_directory('robot_bringup'), 'rviz', 'gzb_rviiz.rviz'))

    # Launch arguments
    declare_urdf_path = DeclareLaunchArgument('urdf_path', default_value=urdf_path, description='Path to the robot URDF')
    declare_rviz_config_path = DeclareLaunchArgument('rviz_config_path', default_value=rviz_config_path, description='Path to the RViz config file')

    # Include Gazebo launch description
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        )
    )

    # Create robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': launch.substitutions.Command(['xacro ', urdf_path]), 'use_sim_time': True}]
    )

    # Create joint state publisher GUI node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Create Gazebo spawn entity node
    gazebo_ros_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-topic', 'robot_description', '-entity', 'vehicle'],
        parameters=[{'use_sim_time': True}]
    )

    # Create RViz2 node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'deadzone': 0.05,
            'autorepeat_rate': 1.0,
            'coalesce_interval': 100
        }],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # Teleop node for controlling the robot with joystick
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[
            {'enable_button': -1},
            {'require_enable_button': False},
            {'axis_linear.x': 1},
            {'axis_linear.y': 2},
            {'axis_linear.z': 6},
            {'axis_angular.pitch': 5},
            {'axis_angular.roll': 7},
            {'axis_angular.yaw': 3},
            {'scale_linear.x': 1.0},
            {'scale_linear.y': 1.0},
            {'scale_linear.z': 1.0},
            {'scale_angular.pitch': 1.0},
            {'scale_angular.roll': 1.0},
            {'scale_angular.yaw': 1.0}
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )

    return LaunchDescription([
        declare_urdf_path,
        declare_rviz_config_path,
        gazebo_launch,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        gazebo_ros_node,
        rviz2_node,
        joy_node,
        teleop_twist_joy_node
    ])
