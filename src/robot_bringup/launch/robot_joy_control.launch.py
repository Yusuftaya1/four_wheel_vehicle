import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    urdf_path = LaunchConfiguration('urdf_path', default=os.path.join(get_package_share_directory('four_wheel_vehicle'), 'urdf', 'vehicle.urdf.xacro'))
    rviz_config_path = LaunchConfiguration('rviz_config_path', default=os.path.join(get_package_share_directory('robot_bringup'), 'rviz', 'gzb_rviz.rviz'))
    world_path = LaunchConfiguration('world_path', default=os.path.join(get_package_share_directory('robot_bringup'), 'world', 'world_farm.world'))
    
    nav2_map = os.path.join(get_package_share_directory('robot_bringup'), 'map','farm_map.yaml')
    nav2_params = os.path.join(get_package_share_directory('nav2_bringup'), 'params', 'nav2_params.yaml')

    nav2_path = get_package_share_directory('nav2_bringup')
    nav2_launch_path = os.path.join(nav2_path, 'launch', 'bringup_launch.py')

    use_sim_time = LaunchConfiguration("use_sim_time", default=True)

    return LaunchDescription([
        DeclareLaunchArgument('urdf_path', default_value=urdf_path, description='Path to URDF file'),
        DeclareLaunchArgument('rviz_config_path', default_value=rviz_config_path, description='Path to RViz config file'),
        DeclareLaunchArgument('world_path', default_value=world_path, description='Path to world file'),
        DeclareLaunchArgument('use_sim_time', default_value='True', description='Use simulation time'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(FindPackageShare('gazebo_ros').find('gazebo_ros'), 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': world_path}.items()),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': launch.substitutions.Command(['xacro ', urdf_path]), 'use_sim_time': use_sim_time}]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{'use_sim_time': True}, {'initial_pose_x': 0.0}, {'initial_pose_y': 0.0}, {'initial_pose_a': 0.0}]
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_localization',
            output='screen',
            parameters=[{'use_sim_time': True}, {'odom_frame': 'odom'}, {'base_frame': 'base_link'}]    
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            arguments=['-topic', 'robot_description', '-entity', 'vehicle'],
            parameters=[{'use_sim_time': True}]
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(nav2_launch_path),
            launch_arguments = {
                'use_sim_time' : use_sim_time,
                'map' : nav2_map,
                'params_file' : nav2_params
            }.items()),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'deadzone': 0.05,
                'autorepeat_rate': 1.0,
                'coalesce_interval': 100
            }],
            arguments=['--ros-args', '--log-level', 'info']
        ),

        Node(
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
        ),
    ])