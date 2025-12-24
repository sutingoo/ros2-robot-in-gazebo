import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('my_bot_description')
    # Ruta al archivo XACRO
    xacro_file = os.path.join(pkg_path, 'urdf', 'mi_robot_autonomo.urdf.xacro')
    # PROCESAR XACRO
    robot_description_config = Command(['xacro ', xacro_file])
    params = {'robot_description': robot_description_config}

    # NODOS
    # Publica la estructura física del robot
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # mover las ruedas manualmente en una ventanita
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # RViz2
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )
    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz
    ])
