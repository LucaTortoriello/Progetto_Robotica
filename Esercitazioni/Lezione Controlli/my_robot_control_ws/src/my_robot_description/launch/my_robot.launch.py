import os
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    # Percorsi ai file URDF e RViz
    pkg_path = get_package_share_directory('my_robot_description')
    pkg_path2 = get_package_share_path('my_robot_description')
    urdf_path = os.path.join(pkg_path, 'urdf', 'my_manipulator.urdf.xacro')
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'urdf_config.rviz')

    # Elaborazione del file xacro
    robot_description_config = xacro.process_file(urdf_path).toxml()

    # Nodo per pubblicare lo stato del robot
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config}]
    )

    # Nodo per il Joint State Publisher GUI
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    # Nodo per avviare RViz con il file di configurazione
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',       
        arguments=['-d', rviz_config_path], 
        output='screen'
    )

    # Restituzione della Launch Description
    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz
    ])

