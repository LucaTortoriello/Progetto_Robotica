#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node

from my_robot_interfaces.action import MoveHead

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class HeadActionServer(Node):

    def __init__(self):
        super().__init__('head_action_server')

        # Action server
        self._action_server = ActionServer(
            self,
            MoveHead,  # interfaccia personalizzata
            'move_head',
            self.execute_callback,  # callback da eseguire quando arriva un goal
        )

        # Client verso TIAGO
        self.follow_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/head_controller/follow_joint_trajectory'
        )

        self.joint_names = ['head_1_joint', 'head_2_joint']  # nome giunti
        self.head2_fixed = -0.57  # posizione fissa per il secondo giunto della testa
        self.get_logger().info("Action server custom pronto.")

    # ------------------------------------------------------
    def move_head(self, pos1):  # funzione per muovere la testa, pos1 è la posizione del primo giunto della testa
        while not self.follow_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Attendo controller...")

        # Creazione del messaggio di traiettoria
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [pos1, self.head2_fixed]
        point.time_from_start.sec = 1

        traj.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        future = self.follow_client.send_goal_async(goal)  # serve per avere un feedback quando il goal è stato accettato o rifiutato
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        return True

    # ------------------------------------------------------
    def execute_callback(self, goal_handle):  # callback da eseguire quando arriva un goal, goal_handle contiene tutte le informazioni del goal ricevuto
        self.get_logger().info("Goal ricevuto!")

        feedback = MoveHead.Feedback()
        result = MoveHead.Result()

        # Normalizzo i limiti per evitare problemi se il goal arriva con min e max invertiti.
        min_pos = min(goal_handle.request.min, goal_handle.request.max)
        max_pos = max(goal_handle.request.min, goal_handle.request.max)
        step = abs(goal_handle.request.step)

        # Se lo step non è valido non è possibile eseguire la scansione.
        if step == 0.0:
            self.get_logger().warn("Step nullo: impossibile eseguire la scansione completa.")
            goal_handle.abort()
            result.success = False
            return result

        # Costruisco tutta la sequenza di scansione:
        # 1. da sinistra a destra (min -> max)
        # 2. ritorno da destra a sinistra (max-step -> min)
        # In questo modo il robot copre tutto l'ambiente senza fermarsi a metà.
        scan_positions = []

        current = min_pos
        while current <= max_pos:
            scan_positions.append(current)
            current += step

        # Se max non cade esattamente su uno step, lo aggiungo comunque per completare la scansione.
        if scan_positions[-1] < max_pos:
            scan_positions.append(max_pos)

        current = max_pos - step
        while current >= min_pos:
            scan_positions.append(current)
            current -= step

        for position in scan_positions:
            # Movimento testa
            if not self.move_head(position):
                self.get_logger().warn("Movimento testa fallito durante la scansione.")
                goal_handle.abort()
                result.success = False
                return result

            # Feedback reale: pubblico la posizione raggiunta durante la scansione.
            feedback.current_position = position
            goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        result.success = True

        return result


def main(args=None):
    rclpy.init(args=args)
    node = HeadActionServer()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
