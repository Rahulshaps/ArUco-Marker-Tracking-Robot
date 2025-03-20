# EECE 5554, Group 4
# Date created: 12/2/24
# Date last modified: 12/2/24
# Description: launch file to launch all necessary components for target tracking integration with navigation

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    start_target_pub_cmd = Node(
        package='py_img_stream',
        executable='target_pub',
        name='target_pub',
    )

    start_image_pub_cmd = Node(
        package='py_img_stream',
        executable='img_pub',
        name='img_pub',
    )

    start_serial_cmd = Node(
        package='py_serial',
        executable='serial_handler',
        name='serial_handler',
    )

    # Launch!
    ld = LaunchDescription()

    ld.add_action(start_target_pub_cmd)
    ld.add_action(start_image_pub_cmd)
    ld.add_action(start_serial_cmd)

    return ld