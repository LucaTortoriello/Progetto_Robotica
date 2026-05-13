from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'det_node_name',
            default_value='action_server_head_scan'
        ),
        DeclareLaunchArgument(
            'aruco_node_name',
            default_value='aruco_detector'
        ),
        DeclareLaunchArgument(
            'scan_min',
            default_value='-0.6'
        ),
        DeclareLaunchArgument(
            'scan_max',
            default_value='0.6'
        ),
        DeclareLaunchArgument(
            'scan_step',
            default_value='0.2'
        ),
        Node(
            emulate_tty=True,
            package='task1_pkg',
            executable='det_task1',
            name=LaunchConfiguration('det_node_name'),
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='task1_pkg',
            executable='aruco_detector_all_markers',
            name=LaunchConfiguration('aruco_node_name'),
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='task1_pkg',
            executable='move_head_client_task1',
            name='move_head_client',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'scan_min': LaunchConfiguration('scan_min')},
                {'scan_max': LaunchConfiguration('scan_max')},
                {'scan_step': LaunchConfiguration('scan_step')}
            ]
        )
    ])
