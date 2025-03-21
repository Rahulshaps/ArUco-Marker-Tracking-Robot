# EECE 5554, Group 4
# Date created: 12/2/24
# Date last modified: 12/2/24
# Description: launch file to launch all necessary components for physical navigation

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    pkg_path = os.path.join(get_package_share_directory('basic_mobile_robot'))
    nav2_dir = get_package_share_directory('nav2_bringup')
    tbot_dir = get_package_share_directory('turtlebot3_bringup')

    model_file = os.path.join(pkg_path, 'models', 'turtlebot_burger.urdf')
    robot_localization_file_path = os.path.join(pkg_path, 'config', 'ekf.yaml')
    default_rviz_config_path = os.path.join(pkg_path, 'rviz', 'turtlebot.rviz')
    slam_params_file = os.path.join(pkg_path, 'config', 'doslam.yaml')
    nav2_params_file = os.path.join(pkg_path, 'config', 'nav2_params.yaml')

    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    model = LaunchConfiguration('model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz')

    remappings = [('/tf', 'tf'),
                ('/tf_static', 'tf_static')]

    declare_autostart_cmd = DeclareLaunchArgument(
        name='autostart', 
        default_value='True',
        description='Automatically startup the nav2 stack'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        name='params_file',
        default_value=nav2_params_file,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
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

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='rviz',
        default_value='False',
        description='Whether to use RViz or not'
    )

    start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
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
    )

    start_lidar_filter_cmd = Node(
        package='navigation',
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
    )

    start_image_pub_cmd = Node(
        package='aruco',
        executable='target_pos',
        name='img_pub',
    )

    start_nav2pose_cmd = Node(
        package='navigation',
        executable='nav2pose',
        name='nav2pose'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    ) 

    start_search_cmd = Node(
        package='navigation',
        executable='searchtargets',
        name='searchtargets'
    )

    start_rviz_cmd = GroupAction(
        condition=IfCondition(PythonExpression([use_rviz])),
        actions = [rviz_node]
    )

    start_ros2_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch', 'navigation_launch.py')),
        launch_arguments = {'use_sim_time': use_sim_time,
                            'params_file': params_file,
                            'autostart': autostart,
                            'log_level': 'info'}.items()
    )

    start_turtlebot3_odom_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tbot_dir, 'launch', 'robot.launch.py'))
    )

    # Launch!
    ld = LaunchDescription()

    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_model_path_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)

    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_cmd)

    ld.add_action(start_lidar_cmd)
    # ld.add_action(start_lidar_filter_cmd)
    
    ld.add_action(start_robot_localization_cmd)

    ld.add_action(start_slam_cmd)
    ld.add_action(start_ros2_navigation_cmd)
    ld.add_action(start_turtlebot3_odom_cmd)
    # ld.add_action(start_search_cmd)

    # ld.add_action(start_image_pub_cmd)
    ld.add_action(start_nav2pose_cmd)

    ld.add_action(start_rviz_cmd)

    return ld