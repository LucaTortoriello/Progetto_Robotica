import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # ====================================
    # INCLUSIONE LAUNCH FILE TIAGO GAZEBO
    # ====================================
    # Ottiene il percorso del pacchetto tiago_gazebo_simulation
    tiago_gazebo_pkg = get_package_share_directory('tiago_gazebo_simulation')
    
    # Percorso al launch file di tiago_gazebo
    tiago_gazebo_launch = os.path.join(
        tiago_gazebo_pkg,
        'launch',
        'tiago_gazebo.launch.py'
    )
    
    # Include il launch file di tiago_gazebo con il parametro is_public_sim=True
    tiago_gazebo_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tiago_gazebo_launch),
        launch_arguments={
            'is_public_sim': 'True'
        }
    )

    # ====================================
    # NODO ACTION SERVER - SCANSIONE TESTA
    # ====================================
    # Lancia il nodo det_task1 che implementa l'action server per la scansione
    det_task1_node = Node(
        package='progetto_robotica',
        executable='det_task1',
        output='screen',
        name='action_server_head_scan'
    )

    # ====================================
    # NODO DETECTOR ARUCO
    # ====================================
    # Lancia il nodo aruco_detector_all_markers che rileva i marker ArUco
    aruco_detector_node = Node(
        package='progetto_robotica',
        executable='aruco_detector_all_markers',
        output='screen',
        name='aruco_detector'
    )

    # ====================================
    # RESTITUZIONE LAUNCH DESCRIPTION
    # ====================================
    # Ritorna la launch description con tutti gli elementi
    return LaunchDescription([
        tiago_gazebo_launch_description,
        det_task1_node,
        aruco_detector_node
    ])
