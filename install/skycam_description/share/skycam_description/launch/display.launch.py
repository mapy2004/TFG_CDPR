import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    pkg_desc = 'skycam_description'
    pkg_share = get_package_share_directory(pkg_desc)

    # =========================
    # XACRO → URDF
    # =========================
    xacro_file = os.path.join(pkg_share, 'urdf', 'skycam.xacro')
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toxml()

    # =========================
    # Gazebo
    # =========================
    gazebo_pkg = get_package_share_directory('gazebo_ros')
    gazebo_launch = os.path.join(
        gazebo_pkg, 'launch', 'gazebo.launch.py'
    )

    return LaunchDescription([

        # -------------------------
        # Gazebo
        # -------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch)
        ),

        # -------------------------
        # Robot State Publisher
        # -------------------------
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc
            }]
        ),

        # -------------------------
        # Spawn del robot en Gazebo
        # -------------------------
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'skycam',
                '-topic', 'robot_description'
            ],
            output='screen'
        ),

        # -------------------------
        # Dinámica (bajo nivel)
        # -------------------------
        Node(
            package='skycam_control',
            executable='skycam_dynamics',
            name='skycam_dynamics',
            output='screen'
        ),

        # -------------------------
        # Control alto nivel
        # -------------------------
        Node(
            package='skycam_control',
            executable='skycam_controller',
            name='skycam_controller',
            output='screen'
        ),

        # -------------------------
        # RViz (opcional pero útil)
        # -------------------------
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        )
    ])
