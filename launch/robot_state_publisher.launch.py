import os
from ament_index_python.packages import get_package_share_directory
import xacro
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():

    xacro_file_path = os.path.join(get_package_share_directory('line_following_robot'), 'description', 'robot.xacro')
    processed_xacro_file = xacro.process_file(xacro_file_path)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': processed_xacro_file.toxml(),
            'use_sim_time': True
        }]
    )

    return LaunchDescription([
        robot_state_publisher
    ])