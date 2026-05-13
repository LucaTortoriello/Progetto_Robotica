#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from my_robot_interfaces.action import MoveHead


class MoveHeadClient(Node):
    def __init__(self):
        super().__init__('move_head_client')

        self._action_client = ActionClient(self, MoveHead, '/move_head')
        self.get_logger().info('MoveHeadClient pronto per inviare goal.')

    def send_goal(self, min_pos, max_pos, step):
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Attendo il server /move_head...')

        goal_msg = MoveHead.Goal()
        goal_msg.min = min_pos
        goal_msg.max = max_pos
        goal_msg.step = step

        self.get_logger().info(
            f"Invio goal scan: min={min_pos:.3f}, max={max_pos:.3f}, step={step:.3f}"
        )

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        rclpy.spin_until_future_complete(self, self._send_goal_future)
        self._goal_handle = self._send_goal_future.result()

        if not self._goal_handle.accepted:
            self.get_logger().warn('Goal rifiutato dal server.')
            return

        self.get_logger().info('Goal accettato dal server.')

        self._get_result_future = self._goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self._get_result_future)
        result = self._get_result_future.result().result

        if result.success:
            self.get_logger().info(f"Scansione completata: {result.message}")
        else:
            self.get_logger().warn(f"Scansione fallita: {result.message}")

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"Scanning: pos={feedback.current_position:.3f}, moving={feedback.moving}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = MoveHeadClient()

    node.send_goal(-0.6, 0.6, 0.2)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
