# EECE 5554, Group 4
# Date created: 12/2/24
# Date last modified: 12/2/24
# Description: launch file to launch all necessary components for physical navigation

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_path = os.path.join(get_package_share_directory('basic_mobile_robot'))
    tbot_path = get_package_share_directory('turtlebot3_bringup')
    model_file = os.path.join(pkg_path, 'models', 'turtlebot_burger.urdf')
    robot_localization_file_path = os.path.join(pkg_path, 'config', 'ekf.yaml')
    default_rviz_config_path = os.path.join(pkg_path, 'rviz', 'teleop.rviz')
    slam_params_file = os.path.join(pkg_path, 'config', 'doslam.yaml')

    model = LaunchConfiguration('model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    remappings = [('/tf', 'tf'),
                ('/tf_static', 'tf_static')]

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use sim time if true'
    )

    declare_model_path_cmd = DeclareLaunchArgument(
        name='model', 
        default_value=model_file, 
        description='Absolute path to robot urdf file'
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use'
    )

    start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_file_path, 
        {'use_sim_time': use_sim_time}]
    )

    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 
        'robot_description': Command(['xacro ', model])}],
        remappings=remappings,
        arguments=[model_file]
    )

    start_lidar_cmd = Node(
        name='rplidar_composition',
        package='rplidar_ros',
        executable='rplidar_composition',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'base_scan',
            'angle_compensate': True,
            'auto_standby': True,
            'scan_mode': 'Boost'
        }],
        output='screen'
    )

    start_lidar_filter_cmd = Node(
        package='navigator',
        executable='filterlidar',
        name='filterlidar'
    )

    start_slam_cmd = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    ) 

    start_turtlebot3_odom_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tbot_path, 'launch', 'robot.launch.py'))
    )

    # Launch!
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_model_path_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    ld.add_action(start_robot_localization_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_lidar_cmd)
    # ld.add_action(start_lidar_filter_cmd)
    ld.add_action(start_turtlebot3_odom_cmd)
    ld.add_action(start_slam_cmd)
    ld.add_action(start_rviz_cmd)

    return ld