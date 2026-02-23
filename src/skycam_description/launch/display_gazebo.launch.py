import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # 1. Obtener directorios
    pkg_share = get_package_share_directory('skycam_description')
    
    # 2. Archivos XACRO y WORLD
    # Usamos el XACRO con las 4 patas invisibles y los 4 plugins de fuerza
    xacro_file = os.path.join(pkg_share, 'urdf', 'skycam_gazebo.xacro')
    
    # Tu mundo personalizado
    world_file = os.path.join(pkg_share, 'worlds', 'normal.world')

    # 3. Procesar XACRO -> URDF
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toprettyxml()

    # Guardar en archivo temporal para que SpawnEntity lo pueda leer
    tmp_urdf = tempfile.NamedTemporaryFile(delete=False, suffix='.urdf')
    tmp_urdf.write(robot_desc.encode('utf-8'))
    tmp_urdf.close()

    # -------------------------
    # 4. Lanzar Gazebo
    # -------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={'world': world_file}.items()
    )

    # -------------------------
    # 5. Spawn del Robot
    # -------------------------
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'skycam_v3',
            '-file', tmp_urdf.name,
            '-x', '0',
            '-y', '0',
            '-z', '0.5' # Empezamos un poco elevados
        ],
        output='screen'
    )

    # -------------------------
    # 6. Nodo de Control (4 CABLES)
    # -------------------------
    # Este nodo calcula la cinemática inversa y la matriz J
    # para distribuir la fuerza entre los 4 motores.
    four_cables_node = Node(
        package='skycam_control',       
        executable='skycam_four_cables', # Debe coincidir con setup.py
        name='skycam_four_cables',
        output='screen',
        parameters=[{'use_sim_time': True}] # Vital para sincronización
    )

    return LaunchDescription([
        gazebo,
        spawn,
        four_cables_node   # El nuevo controlador
    ])