from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'aruco_node_name',
            default_value='aruco_detector_all_markers'
        ),
        DeclareLaunchArgument(
            'det_node_name',
            default_value='action_server_head_scan'
        ),
        DeclareLaunchArgument(
            'urdf_path',
            default_value='/home/ubuntu/progetto_12_ws/src/my_robot_description/urdf/tiago_robot.urdf'
        ),
        DeclareLaunchArgument(
            'run_unfold_arm',
            default_value='true'
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
        DeclareLaunchArgument(
            'scan_wait_seconds',
            default_value='15.0'
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
        ),
        Node(
            package='task2_pkg',
            executable='pick_and_place_task2',
            name='tiago_pick_place_task2',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'urdf_path': ParameterValue(LaunchConfiguration('urdf_path'), value_type=str)},
                {'run_unfold_arm': ParameterValue(LaunchConfiguration('run_unfold_arm'), value_type=bool)},
                {'scan_wait_seconds': ParameterValue(LaunchConfiguration('scan_wait_seconds'), value_type=float)}
            ]
        )
    ])
