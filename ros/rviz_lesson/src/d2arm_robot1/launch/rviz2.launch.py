import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

PACKAGE_NAME = 'd2arm_robot1'

def generate_launch_description():
    rviz_config_file = os.path.join(
        get_package_share_directory(PACKAGE_NAME),
        'config',
        'robot.rviz'
    )
    default_rviz_fixed_frame = 'base_link'

    urdf_file = os.path.join(
        get_package_share_directory(PACKAGE_NAME),
        'urdf',
        'robot.urdf'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=urdf_file,
            description='Absolute path to robot urdf file'
        ),

        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_config_file,
            description='Absolute path to rviz config file'
        ),

        DeclareLaunchArgument(
            'rviz_fixed_frame',
            default_value=default_rviz_fixed_frame,
            description='RViz2 fixed frame option'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config'), "-f", LaunchConfiguration('rviz_fixed_frame')],
            output='screen'
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[LaunchConfiguration('model')]
        )
    ])
