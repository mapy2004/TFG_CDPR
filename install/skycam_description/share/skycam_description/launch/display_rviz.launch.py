import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    pkg = get_package_share_directory('skycam_description')
    xacro_file = os.path.join(pkg, 'urdf', 'skycam_rviz.xacro')

    robot_desc = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}]
        ),

        Node(
            package='skycam_control',
            executable='skycam_dynamics',
            output='screen'
        ),

        Node(
            package='skycam_control',
            executable='skycam_controller',
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        )
    ])
