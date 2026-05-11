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
            default_value='~/progetto_12_ws/src/my_robot_description/urdf/tiago_robot.urdf'
        ),
        DeclareLaunchArgument(
            'run_unfold_arm',
            default_value='true'
        ),
        DeclareLaunchArgument(
            'aruco_A',
            default_value='26'
        ),
        DeclareLaunchArgument(
            'aruco_B',
            default_value='63'
        ),
        DeclareLaunchArgument(
            'aruco_C',
            default_value='4'
        ),
        DeclareLaunchArgument(
            'aruco_D',
            default_value='3'
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
            package='task2_pkg',
            executable='pick_and_place_task2',
            name='tiago_pick_place_task2',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'aruco_A': ParameterValue(LaunchConfiguration('aruco_A'), value_type=int)},
                {'aruco_B': ParameterValue(LaunchConfiguration('aruco_B'), value_type=int)},
                {'aruco_C': ParameterValue(LaunchConfiguration('aruco_C'), value_type=int)},
                {'aruco_D': ParameterValue(LaunchConfiguration('aruco_D'), value_type=int)},
                {'urdf_path': ParameterValue(LaunchConfiguration('urdf_path'), value_type=str)},
                {'run_unfold_arm': ParameterValue(LaunchConfiguration('run_unfold_arm'), value_type=bool)}
            ]
        )
    ])
