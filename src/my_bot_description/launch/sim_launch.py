import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    package_name = 'my_bot_description'
    pkg_path = get_package_share_directory(package_name)

    # 1. PROCESAR EL URDF (Igual que antes)
    xacro_file = os.path.join(pkg_path, 'urdf', 'mi_robot_autonomo.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file])
    params = {'robot_description': robot_description_config, 'use_sim_time': True}
    # Nota: 'use_sim_time' es CRÍTICO. Le dice a ROS que el reloj no es el de tu PC, sino el del simulador.

    # 2. ROBOT STATE PUBLISHER (El que publica el robot)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # 3. LANZAR GAZEBO (El Mundo Vacío)
    # Incluimos el launch file que viene con el paquete gazebo_ros
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # 4. SPAWN ENTITY (El acto de nacimiento)
    # Este nodo toma el URDF del topic 'robot_description' y lo mete en Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'mi_robot_autonomo'], # El nombre que tendrá en Gazebo
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
    ])
