import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # 1. Obtener directorios
    pkg_share = get_package_share_directory('skycam_description')
    
    # 2. Archivos XACRO y WORLD
    skycam_xacro_file = os.path.join(pkg_share, 'urdf', 'skycam_gazebo.xacro')
    ball_xacro_file = os.path.join(pkg_share, 'urdf', 'ball.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'normal.world')

    # 3. Procesar XACRO -> URDF (Skycam)
    doc_skycam = xacro.process_file(skycam_xacro_file)
    tmp_urdf_skycam = tempfile.NamedTemporaryFile(delete=False, suffix='.urdf')
    tmp_urdf_skycam.write(doc_skycam.toprettyxml().encode('utf-8'))
    tmp_urdf_skycam.close()

    # 3.1 Procesar XACRO -> URDF (Pelota)
    doc_ball = xacro.process_file(ball_xacro_file)
    tmp_urdf_ball = tempfile.NamedTemporaryFile(delete=False, suffix='.urdf')
    tmp_urdf_ball.write(doc_ball.toprettyxml().encode('utf-8'))
    tmp_urdf_ball.close()

    # -------------------------
    # 4. Lanzar Gazebo
    # -------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # -------------------------
    # 5. Spawns
    # -------------------------
    spawn_skycam = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'skycam_v3', '-file', tmp_urdf_skycam.name, '-x', '0', '-y', '0', '-z', '8'],
        output='screen'
    )

    # Spawn de la pelota desplazada del centro para que la cámara tenga que buscarla
    spawn_ball = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'soccer_ball', '-file', tmp_urdf_ball.name, '-x', '0.0', '-y', '0.0', '-z', '0.2'],
        output='screen'
    )

    # -------------------------
    # 6. Nodos Base de la Skycam
    # -------------------------
    four_cables_node = Node(
        package='skycam_control',       
        executable='skycam_four_cables',
        name='skycam_four_cables',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    trajectory_planner_node = Node(
        package='skycam_control',       
        executable='skycam_trajectory_planner', 
        name='skycam_trajectory_planner',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # -------------------------
    # 7. Nodos del Escenario IA
    # -------------------------
    # Nodo que mueve la pelota en círculos
    ball_mover_node = Node(
        package='skycam_control', # Asumo que lo pones en el mismo paquete
        executable='ball_mover', 
        name='ball_mover',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Nodo IA de YOLOv8
    ai_tracker_node = Node(
        package='skycam_control',       
        executable='skycam_ai_tracker', 
        name='skycam_ai_tracker',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo,
        spawn_skycam,
        spawn_ball,
        four_cables_node,
        trajectory_planner_node,
        # Retrasamos un par de segundos el arranque de los nodos IA/Pelota para 
        # dejar que la cámara termine su "Soft-Start" y se estabilice en Z=6m
        TimerAction(
            period=5.0,
            actions=[ai_tracker_node]
        ),
        TimerAction(
            period=25.0,
            actions=[ball_mover_node]
        )
    ])