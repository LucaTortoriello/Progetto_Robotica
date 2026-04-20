#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class DetectionTask1Node(Node):
    def __init__(self):
        super().__init__('detection_task_1')

        # Riferimento blocco ES4:
        # - Esercitazioni/Esercitazione4-Action/codes/action_server.py lines 27-31
        # Client verso il controller testa TIAGo, usando FollowJointTrajectory.
        self.head_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/head_controller/follow_joint_trajectory',
        )

        # Riferimento blocco ES4:
        # - Esercitazioni/Esercitazione4-Action/codes/action_server.py lines 33
        # Nomi giunti testa
        self.joint_names = ['head_1_joint', 'head_2_joint']

        # Parametri del nodo (scansione ambiente con movimento testa).
        self.declare_parameter('scan_positions', [-0.6, -0.3, 0.0, 0.3, 0.6])
        self.declare_parameter('scan_tilt', -0.57)
        self.declare_parameter('scan_period_sec', 1.2)
        self.declare_parameter('move_duration_sec', 1.0)

        self.scan_positions = [
            float(x) for x in self.get_parameter('scan_positions').value
        ]
        self.scan_tilt = float(self.get_parameter('scan_tilt').value)
        self.scan_period_sec = float(self.get_parameter('scan_period_sec').value)
        self.move_duration_sec = float(self.get_parameter('move_duration_sec').value)

        self.scan_index = 0
        self.head_goal_in_progress = False
        self.warned_head_server_unavailable = False

        self.scan_timer = self.create_timer(
            self.scan_period_sec,
            self.scan_if_needed,
        )

        self.get_logger().info(
            'Nodo detection_task_1 avviato: scansione testa attiva.'
        )

    def scan_if_needed(self):
        if not self.scan_positions:
            return

        if self.head_goal_in_progress:
            return

        # Riferimento blocco ES4:
        # - Esercitazioni/Esercitazione4-Action/codes/action_server.py lines 39-46
        # - action/codes/action_server.py lines 38-46
        # Verifica disponibilita action server prima dell'invio goal.
        if not self.head_client.wait_for_server(timeout_sec=0.2):
            if not self.warned_head_server_unavailable:
                self.warned_head_server_unavailable = True
                self.get_logger().warn(
                    'Action server /head_controller/follow_joint_trajectory non disponibile.'
                )
            return

        self.warned_head_server_unavailable = False

        pan = self.scan_positions[self.scan_index]
        self.scan_index = (self.scan_index + 1) % len(self.scan_positions)

        self.get_logger().info(
            f'Scansione ambiente -> pan={pan:.2f}, tilt={self.scan_tilt:.2f}'
        )
        self._send_head_goal(pan, self.scan_tilt)

    def _send_head_goal(self, pan, tilt):
        # Riferimento blocco ES4:
        # - Esercitazioni/Esercitazione4-Action/codes/action_server.py lines 39-56
        # - action/codes/action_server.py lines 38-52
        # Costruzione trajectory con JointTrajectory + JointTrajectoryPoint.
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [float(pan), float(tilt)]

        duration = max(0.1, self.move_duration_sec)
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)

        traj.points.append(point)

        # Riferimento blocco ES4:
        # - Esercitazioni/Esercitazione4-Action/codes/action_server.py lines 52-56
        # - action/codes/action_server.py lines 52-56
        # Goal FollowJointTrajectory inviato al controller della testa.
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj

        self.head_goal_in_progress = True
        send_future = self.head_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._head_goal_response_callback)

    def _head_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.head_goal_in_progress = False
            self.get_logger().warn(f'Errore invio goal testa: {exc}')
            return

        if not goal_handle.accepted:
            self.head_goal_in_progress = False
            self.get_logger().warn('Goal testa rifiutato dal controller.')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._head_goal_result_callback)

    def _head_goal_result_callback(self, future):
        self.head_goal_in_progress = False

        try:
            result = future.result().result
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Errore risultato action testa: {exc}')
            return

        if result.error_code != 0:
            self.get_logger().warn(
                f'Goal testa completato con errore_code={result.error_code}: {result.error_string}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = DetectionTask1Node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
