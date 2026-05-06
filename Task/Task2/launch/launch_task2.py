from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'aruco_node_name',
            default_value='aruco_detector_all_markers'
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
                {'aruco_A': 1},
                {'aruco_B': 2},
                {'aruco_C': 3},
                {'aruco_D': 4}
            ]
        )
    ])
