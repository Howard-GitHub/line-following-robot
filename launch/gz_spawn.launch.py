from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    # Initialize launch description that runs robot_state_publisher
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('line_following_robot'), 'launch', 'robot_state_publisher.launch.py')])
    )

    # Initialize launch description that starts up gazebo 
    world = os.path.join(get_package_share_directory('line_following_robot'), 'worlds', 'track.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': ['-r ', world], 'on_exit_shutdown': 'true'}.items()
    )

    # Initialize node to spawn vehicle in gazebo world using robot_description topic
    spawn_vehicle = Node(
        package='ros_gz_sim', 
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-z', '1.0'
        ],
        output='screen'
    )

    # Initialize node to bridge topics listed in bridge.yaml between ROS and Gazebo
    bridge_file = os.path.join(get_package_share_directory('line_following_robot'), 'config', 'bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_file}'
        ]
    )

    # Initialize node to bridge image topic from Gazebo to ROS
    ros_gz_image = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_vehicle,
        ros_gz_bridge,
        ros_gz_image
    ])