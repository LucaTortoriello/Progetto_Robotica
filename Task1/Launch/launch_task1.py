import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _resolve_tiago_gazebo_launch_path():
    """Trova il launch TIAGo allineato al comando: ros2 launch tiago_gazebo ..."""

    # 1) Priorità al package usato nelle esercitazioni
    try:
        pkg_share = get_package_share_directory('tiago_gazebo')
        launch_path = os.path.join(pkg_share, 'launch', 'tiago_gazebo.launch.py')
        if os.path.exists(launch_path):
            return launch_path
    except PackageNotFoundError:
        pass

    # 2) Fallback a percorsi locali reali trovati in /home/ubuntu
    local_candidates = [
        '/home/ubuntu/TIAGo/launch/tiago_gazebo.launch.py',
        '/home/ubuntu/Cinematica Diretta/Progetto_Robotica/Esercitazioni/TIAGo/launch/tiago_gazebo.launch.py',
        '/home/ubuntu/tiago_public_ws/src/tiago_simulation/tiago_gazebo/launch/tiago_gazebo.launch.py',
    ]

    for launch_path in local_candidates:
        if os.path.exists(launch_path):
            return launch_path

    raise RuntimeError(
        "Impossibile trovare 'tiago_gazebo.launch.py'. "
        "Usa un ambiente con package 'tiago_gazebo' sourcato (come nelle esercitazioni) "
        "oppure verifica i percorsi locali sotto /home/ubuntu."
    )

def generate_launch_description():
    # ====================================
    # INCLUSIONE LAUNCH FILE TIAGO GAZEBO
    # ====================================
    # Trova dinamicamente il launch file del simulatore TIAGo.
    tiago_gazebo_launch = _resolve_tiago_gazebo_launch_path()
    
    # Include il launch file di tiago_gazebo con il parametro is_public_sim=True
    tiago_gazebo_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tiago_gazebo_launch),
        launch_arguments={
            'is_public_sim': 'True'
        }.items()
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
