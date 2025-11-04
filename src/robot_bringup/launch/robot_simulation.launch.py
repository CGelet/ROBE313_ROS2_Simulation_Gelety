from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # Paths
    bringup_share = get_package_share_directory('robot_bringup')
    urdf_file = os.path.join(bringup_share, 'urdf', 'simple_bot.urdf')
    rviz_config = os.path.join(bringup_share, 'rviz', 'simulation.rviz')

    # Robot description param (load URDF file)
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # 1) C++ odometry node
        Node(
            package='robot_simulator_cpp',
            executable='odometry_node',
            name='odometry_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # 2) Python controller node
        Node(
            package='robot_simulator_py',
            executable='controller_node',
            name='controller_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # 3) robot_state_publisher with your URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time,
                 'robot_description': robot_description_content}
            ],
        ),

        # 4) RViz2 with your config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
