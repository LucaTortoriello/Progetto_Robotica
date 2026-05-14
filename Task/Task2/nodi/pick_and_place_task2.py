#!/usr/bin/env python3

# ==================================================
# Task 2 - Pick & Place per TIAGo usando T_place
# ==================================================
# Questo script è una variante del nodo Task2 in cui
# la fase di piazzamento usa una trasformazione dedicata
# T_place, come richiesto dalle slide.
#
# - T_pick allinea il gripper per il pick
# - T_place allinea il gripper per il place
#
# In base alle slide:
#   x_G = -x_A
#   y_G = -z_A
#   z_G = -y_A
# la rotazione corrispondente è:
#   R_place = Rx(pi/2) * Rz(pi)
# ==================================================

import os
import time
import functools
import subprocess

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import PoseStamped
from gazebo_ros_link_attacher.srv import Attach
from roboticstoolbox import ERobot
from spatialmath import SE3
from scipy.spatial.transform import Rotation as R
import numpy as np


class TiagoPickAndPlaceTask2WithTPlace(Node):
    def __init__(self):
        super().__init__('tiago_pick_place_task2_with_T_place')

        self.joint_names = [
            'torso_lift_joint',
            'arm_1_joint',
            'arm_2_joint',
            'arm_3_joint',
            'arm_4_joint',
            'arm_5_joint',
            'arm_6_joint',
            'arm_7_joint'
        ]

        self.declare_parameter('urdf_path', '/home/ubuntu/progetto_12_ws/src/my_robot_description/urdf/tiago_robot.urdf')
        self.declare_parameter('run_unfold_arm', True)
        self.declare_parameter('scan_wait_seconds', 15.0)

        self.marker_ids = {
            'A': 2,
            'B': 1,
            'C': 3,
            'D': 4
        }

        self.tasks = [
            {
                'name': 'cocacola',
                'pick': 'A',
                'place': 'C',
                'model_name': 'cocacola_with_aruco::cocacola',
                'link_name': 'link'
            },
            {
                'name': 'pringles',
                'pick': 'B',
                'place': 'D',
                'model_name': 'pringles_with_aruco::pringles',
                'link_name': 'link'
            }
        ]

        self.create_subscription(
            JointTrajectoryControllerState,
            '/arm_controller/controller_state',
            self.arm_callback,
            10
        )
        self.create_subscription(
            JointTrajectoryControllerState,
            '/torso_controller/controller_state',
            self.torso_callback,
            10
        )

        self.marker_poses = {}
        for label, marker_id in self.marker_ids.items():
            self.create_subscription(
                PoseStamped,
                f'/aruco_base_{marker_id}',
                functools.partial(self.pose_callback, label),
                10
            )

        self.arm_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )
        self.torso_pub = self.create_publisher(
            JointTrajectory,
            '/torso_controller/joint_trajectory',
            10
        )
        self.gripper_pub = self.create_publisher(
            JointTrajectory,
            '/gripper_controller/joint_trajectory',
            10
        )

        urdf_loc = os.path.expanduser(str(self.get_parameter('urdf_path').value))
        self.robot = ERobot.URDF(urdf_loc)

        self.T_pick = SE3.Ry(np.pi / 2) * SE3.Rz(-np.pi / 2)
        self.T_place = SE3.Rx(np.pi / 2) * SE3.Rz(np.pi)

        self.q0_ready = False
        self.q0_arm = None
        self.q0_torso = None
        self.init_saved = False
        self.ready_pose = None
        self.ready_pose_after_unfold = None
        self.q_sent = None
        self.max_joint_jump = 0.6
        self.max_joint_jump_fallback = 1.2
        self.lower_pick_duration = 7.0
        self.close_delay_after_lower = 14.0
        self.lift_after_pick_duration = 5.0

        self.current_task_index = 0
        self.state = 'WAIT_SCAN_MARKERS'
        self.wait_until = 0.0
        self.last_wait_marker_log = 0.0
        self.last_scan_wait_log = 0.0
        self.scan_wait_until = time.time() + float(self.get_parameter('scan_wait_seconds').value)

        self.attach_client = self.create_client(Attach, '/attach')
        self.detach_client = self.create_client(Attach, '/detach')

        while not self.attach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Attesa servizio /attach...')
        while not self.detach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Attesa servizio /detach...')

        self.timer = self.create_timer(0.5, self.loop)
        self.get_logger().info('Nodo Task2 Pick & Place con T_place avviato')

    def arm_callback(self, msg):
        self.q0_arm = np.array(msg.actual.positions)
        self.check_q0_ready()

    def torso_callback(self, msg):
        self.q0_torso = np.array(msg.actual.positions)
        self.check_q0_ready()

    def check_q0_ready(self):
        self.q0_ready = self.q0_arm is not None and self.q0_torso is not None
        if self.q0_ready and not self.init_saved:
            self.ready_pose = np.concatenate((self.q0_torso, self.q0_arm))
            self.init_saved = True
            self.q_sent = self.ready_pose.copy()
            self.get_logger().info('Configurazione iniziale salvata')

    def pose_callback(self, label, msg):
        self.marker_poses[label] = msg
        self.get_logger().debug(f'Pose marker {label} ricevuta')

    def open_gripper(self):
        traj = JointTrajectory()
        traj.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
        point = JointTrajectoryPoint()
        point.positions = [0.045, 0.045]
        point.time_from_start.sec = 2
        traj.points.append(point)
        self.gripper_pub.publish(traj)
        self.get_logger().info('Gripper aperto')

    def close_gripper(self):
        traj = JointTrajectory()
        traj.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0]
        point.time_from_start.sec = 2
        traj.points.append(point)
        self.gripper_pub.publish(traj)
        self.get_logger().info('Gripper chiuso')

    def loop(self):
        if not self.q0_ready:
            return

        if time.time() < self.scan_wait_until:
            now = time.time()
            if now - self.last_scan_wait_log > 5.0:
                remaining = self.scan_wait_until - now
                self.get_logger().info(f"Attendo fine scansione: {remaining:.1f}s rimanenti")
                self.last_scan_wait_log = now
            return

        if self.state == 'WAIT_SCAN_MARKERS':
            if not self.have_all_markers():
                self.log_missing_all_markers()
                return
            self.get_logger().info('Marker iniziali rilevati. Avvio configurazione comoda.')
            self.state = 'UNFOLD_ARM'
            return

        if self.state == 'UNFOLD_ARM':
            if not self.init_saved:
                return
            self.run_unfold_arm_motion()
            self.wait_until = time.time() + 4
            self.state = 'WAIT_READY'
            return

        if self.state == 'WAIT_READY':
            if time.time() < self.wait_until:
                return
            if self.q0_ready:
                self.ready_pose_after_unfold = np.concatenate((self.q0_torso, self.q0_arm))
                self.ready_pose = self.ready_pose_after_unfold.copy()
                self.q_sent = self.ready_pose.copy()
            self.get_logger().info('Configurazione comoda raggiunta. Inizio Task2')
            self.state = 'WAIT_MARKER'
            return

        if self.state == 'WAIT_MARKER':
            if time.time() < self.wait_until:
                return
            if not self.have_task_poses():
                self.log_missing_task_poses()
                return
            if not self.plan_pick_approach():
                self.get_logger().warn('IK fallita per approach pick: ritento tra 2s')
                self.wait_until = time.time() + 2.0
                return
            self.state = 'MOVE_ABOVE_PICK'
            self.wait_until = time.time() + 4
            return

        if self.state == 'MOVE_ABOVE_PICK':
            if time.time() < self.wait_until:
                return
            self.get_logger().info('Posiziono il gripper sopra il pick')
            if not hasattr(self, 'gripper_opened_for_pick'):
                self.open_gripper()
                self.gripper_opened_for_pick = True
            if not self.lower_to_pick():
                self.state = 'ERROR'
                return
            self.state = 'LOWER_PICK'
            self.wait_until = time.time() + self.lower_pick_duration + self.close_delay_after_lower
            return

        if self.state == 'LOWER_PICK':
            if time.time() < self.wait_until:
                return
            self.get_logger().info('Abbasso verso l\'oggetto')
            self.close_gripper()
            self.state = 'CLOSE_GRIPPER'
            self.wait_until = time.time() + 2.5
            return

        if self.state == 'CLOSE_GRIPPER':
            if time.time() < self.wait_until:
                return
            self.attach_object()
            self.get_logger().info('Oggetto attaccato al gripper')
            if not self.lift_after_pick():
                self.state = 'ERROR'
                return
            self.state = 'LIFT_AFTER_PICK'
            self.wait_until = time.time() + self.lift_after_pick_duration + 0.5
            return

        if self.state == 'LIFT_AFTER_PICK':
            if time.time() < self.wait_until:
                return
            if not self.plan_place_approach():
                self.state = 'ERROR'
                return
            self.state = 'MOVE_TO_PLACE'
            self.wait_until = time.time() + 4
            return

        if self.state == 'MOVE_TO_PLACE':
            if time.time() < self.wait_until:
                return
            self.get_logger().info('Sposto il pezzo verso la posizione di place')
            if not self.lower_to_place():
                self.state = 'ERROR'
                return
            self.state = 'LOWER_PLACE'
            self.wait_until = time.time() + 3
            return

        if self.state == 'LOWER_PLACE':
            if time.time() < self.wait_until:
                return
            self.get_logger().info('Abbasso l\'oggetto nella posizione finale')
            self.open_gripper()
            self.state = 'OPEN_GRIPPER'
            self.wait_until = time.time() + 2.5
            return

        if self.state == 'OPEN_GRIPPER':
            if time.time() < self.wait_until:
                return
            self.detach_object()
            self.open_gripper()
            self.go_to_ready_pose()
            self.state = 'RETURN_SAFE'
            self.wait_until = time.time() + 3
            return

        if self.state == 'RETURN_SAFE':
            if time.time() < self.wait_until:
                return
            if hasattr(self, 'gripper_opened_for_pick'):
                del self.gripper_opened_for_pick
            self.current_task_index += 1
            if self.current_task_index >= len(self.tasks):
                self.get_logger().info('Task2 completato: tutti gli oggetti sono stati spostati')
                self.state = 'DONE'
                return
            self.get_logger().info('Passo successivo: nuovo oggetto da spostare')
            self.state = 'WAIT_MARKER'
            return

        if self.state == 'ERROR':
            self.get_logger().error('Task2 interrotto: pianificazione o IK fallita.')
            self.state = 'DONE'
            return

    def have_task_poses(self):
        task = self.tasks[self.current_task_index]
        return task['pick'] in self.marker_poses and task['place'] in self.marker_poses

    def have_all_markers(self):
        return all(label in self.marker_poses for label in self.marker_ids.keys())

    def log_missing_all_markers(self):
        now = time.time()
        if now - self.last_wait_marker_log < 5.0:
            return
        self.last_wait_marker_log = now
        missing = [
            f"{label} (/aruco_base_{self.marker_ids[label]})"
            for label in self.marker_ids.keys()
            if label not in self.marker_poses
        ]
        received = sorted(self.marker_poses.keys())
        self.get_logger().info(
            f"Attendo marker iniziali: mancanti {missing}; ricevuti {received}"
        )

    def log_missing_task_poses(self):
        now = time.time()
        if now - self.last_wait_marker_log < 5.0:
            return
        self.last_wait_marker_log = now
        task = self.tasks[self.current_task_index]
        missing = [
            f"{label} (/aruco_base_{self.marker_ids[label]})"
            for label in [task['pick'], task['place']]
            if label not in self.marker_poses
        ]
        received = sorted(self.marker_poses.keys())
        self.get_logger().info(f"Attendo marker per {task['name']}: mancanti {missing}; ricevuti {received}")

    def get_current_task_poses(self):
        task = self.tasks[self.current_task_index]
        return self.marker_poses[task['pick']], self.marker_poses[task['place']]

    def plan_pick_approach(self):
        pick_pose, _ = self.get_current_task_poses()
        for dy, dz in ((-0.045, -0.09), (-0.040, -0.09), (-0.050, -0.08)):
            pos_above = self._get_offset_position(pick_pose, dx=-0.0035, dy=dy, dz=dz)
            target = self._make_SE3(pick_pose, pos_above) * self.T_pick
            q_target = self.solve_ik(target)
            if q_target is not None:
                self.send_trajectory(q_target)
                self.q_sent = q_target
                return True
        return False

    def lower_to_pick(self):
        pick_pose, _ = self.get_current_task_poses()
        for dz in (-0.10, -0.12, -0.14):
            pos_lower = self._get_offset_position(pick_pose, dx=-0.0035, dy=-0.045, dz=dz)
            target = self._make_SE3(pick_pose, pos_lower) * self.T_pick
            q_target = self.solve_ik(target)
            if q_target is not None:
                # Discesa più lenta per ridurre urti laterali sulla lattina.
                self.send_trajectory(q_target, duration_sec=self.lower_pick_duration)
                self.q_sent = q_target
                return True
            self.get_logger().warn(f'IK fallita per lower_pick con dz={dz:.3f}')
        return False

    def lift_after_pick(self):
        pick_pose, _ = self.get_current_task_poses()
        pos_above = self._get_offset_position(pick_pose, dx=-0.0035, dy=-0.045, dz=-0.08)
        target = self._make_SE3(pick_pose, pos_above) * self.T_pick
        q_target = self.solve_ik(target)
        if q_target is not None:
            self.send_trajectory(q_target, duration_sec=self.lift_after_pick_duration)
            self.q_sent = q_target
            return True
        return False

    def plan_place_approach(self):
        _, place_pose = self.get_current_task_poses()
        pos_above = self._get_offset_position(place_pose, dx=-0.0035, dy=-0.045, dz=-0.10)
        target = self._make_SE3(place_pose, pos_above) * self.T_place
        q_target = self.solve_ik(target)
        if q_target is not None:
            self.send_trajectory(q_target)
            self.q_sent = q_target
            return True
        return False

    def lower_to_place(self):
        _, place_pose = self.get_current_task_poses()
        pos_lower = self._get_offset_position(place_pose, dx=-0.0035, dy=-0.045, dz=-0.14)
        target = self._make_SE3(place_pose, pos_lower) * self.T_place
        q_target = self.solve_ik(target)
        if q_target is not None:
            self.send_trajectory(q_target)
            self.q_sent = q_target
            return True
        return False

    def solve_ik(self, pose):
        q_seed = self.q_sent if self.q_sent is not None else self.ready_pose
        if q_seed is None:
            return None

        candidates = []
        for _ in range(8):
            try:
                solution = self.robot.ik_NR(pose, q0=q_seed, pinv=True)
                if solution is None or len(solution) == 0:
                    raise ValueError('IK fallita')
                q = np.array(solution[0]).flatten()
                if len(q) != len(self.joint_names):
                    raise ValueError(f'IK ha restituito {len(q)} giunti, attesi {len(self.joint_names)}')
            except Exception:
                q_seed = q_seed + np.random.normal(0.0, 0.03, size=len(q_seed))
                continue

            if self.q_sent is None:
                return q

            jump = float(np.max(np.abs(q - self.q_sent)))
            candidates.append((jump, q.copy()))
            if jump <= self.max_joint_jump:
                return q

            q_seed = q + np.random.normal(0.0, 0.02, size=len(q))

        if candidates:
            best_jump, best_q = min(candidates, key=lambda item: item[0])
            if best_jump <= self.max_joint_jump_fallback:
                if best_jump > self.max_joint_jump:
                    self.get_logger().warn(
                        f'IK con salto alto: {best_jump:.3f} rad (fallback consentito)'
                    )
                return best_q

        self.get_logger().warn('IK fallita: nessuna soluzione stabile trovata')
        return None

    def _get_offset_position(self, pose, dx=0.0, dy=0.0, dz=0.0):
        return np.array([
            pose.pose.position.x + dx,
            pose.pose.position.y + dy,
            pose.pose.position.z + dz
        ])

    def _make_SE3(self, pose, position):
        quat = [
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w
        ]
        R_mat = R.from_quat(quat).as_matrix()
        return SE3.Rt(R_mat, position)

    def send_trajectory(self, q, duration_sec=3.0):
        q = np.array(q).flatten()
        traj_arm = JointTrajectory()
        traj_torso = JointTrajectory()
        traj_arm.joint_names = self.joint_names[1:]
        traj_torso.joint_names = [self.joint_names[0]]
        point_arm = JointTrajectoryPoint()
        point_torso = JointTrajectoryPoint()
        point_arm.positions = [float(x) for x in q[1:]]
        point_torso.positions = [float(q[0])]
        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)
        point_arm.time_from_start.sec = sec
        point_arm.time_from_start.nanosec = nanosec
        point_torso.time_from_start.sec = sec
        point_torso.time_from_start.nanosec = nanosec
        traj_arm.points.append(point_arm)
        traj_torso.points.append(point_torso)
        self.arm_pub.publish(traj_arm)
        self.torso_pub.publish(traj_torso)

    def run_unfold_arm_motion(self):
        if not self._as_bool(self.get_parameter('run_unfold_arm').value):
            self.get_logger().info('unfold_arm automatico disabilitato: uso la configurazione corrente')
            self.go_to_ready_pose()
            return
        self.get_logger().info('Eseguo automaticamente play_motion2 run_motion unfold_arm')
        try:
            completed = subprocess.run(
                ['ros2', 'run', 'play_motion2', 'run_motion', 'unfold_arm'],
                check=False,
                timeout=30
            )
            if completed.returncode == 0:
                self.get_logger().info('Motion unfold_arm completata')
                return
            self.get_logger().warn(f'unfold_arm ha restituito codice {completed.returncode}; uso la posa corrente come fallback')
        except Exception as ex:
            self.get_logger().warn(f'Impossibile eseguire unfold_arm: {ex}; uso la posa corrente come fallback')
        self.go_to_ready_pose()

    def _as_bool(self, value):
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ('1', 'true', 'yes', 'on')

    def go_to_ready_pose(self):
        if self.ready_pose is None:
            self.get_logger().warn('ready_pose non disponibile')
            return False
        self.send_trajectory(self.ready_pose)
        self.q_sent = self.ready_pose.copy()
        return True

    def attach_object(self):
        task = self.tasks[self.current_task_index]
        req = Attach.Request()
        req.model_name_1 = 'tiago'
        req.link_name_1 = 'gripper_left_finger_link'
        req.model_name_2 = task['model_name']
        req.link_name_2 = task.get('link_name', 'link')
        future = self.attach_client.call_async(req)
        future.add_done_callback(functools.partial(self.attach_response_callback, 'attach'))
        self.get_logger().info('Richiesta attach inviata')

    def detach_object(self):
        task = self.tasks[self.current_task_index]
        req = Attach.Request()
        req.model_name_1 = 'tiago'
        req.link_name_1 = 'gripper_left_finger_link'
        req.model_name_2 = task['model_name']
        req.link_name_2 = task.get('link_name', 'link')
        future = self.detach_client.call_async(req)
        future.add_done_callback(functools.partial(self.attach_response_callback, 'detach'))
        self.get_logger().info('Richiesta detach inviata')

    def attach_response_callback(self, operation, future):
        try:
            response = future.result()
        except Exception as ex:
            self.get_logger().error(f'Service {operation} fallito: {ex}')
            return
        if hasattr(response, 'ok') and not response.ok:
            self.get_logger().error(f'Service {operation} ha risposto ok=false')
            return
        self.get_logger().info(f'Service {operation} completato')


def main(args=None):
    rclpy.init(args=args)
    node = TiagoPickAndPlaceTask2WithTPlace()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
