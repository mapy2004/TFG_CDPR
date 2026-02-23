import os
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Rutas
    pkg_share = get_package_share_directory('skycam_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'simple_sphere.xacro')

    # Procesar Xacro
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toprettyxml()

    # Lanzar Gazebo (Mundo vacío por defecto para aislar errores de tu mundo custom)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
    )

    # Spawn de la esfera (Usamos spawn_entity directo con el XML procesado)
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'simple_sphere',
            '-topic', 'robot_description', # Truco: no hace falta fichero temporal si usas topic
            '-x', '0', '-y', '0', '-z', '1.0' # 1 Metro de altura
        ],
        output='screen'
    )
    
    # Publicar el estado del robot (necesario para spawnear desde topic)
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo,
        robot_state_pub,
        spawn
    ])