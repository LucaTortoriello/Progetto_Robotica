#!/usr/bin/env python3

import functools
import subprocess
import time

import rclpy
from rclpy.node import Node
from action_msgs.msg import GoalStatus, GoalStatusArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import PoseStamped
from gazebo_ros_link_attacher.srv import Attach
from roboticstoolbox import ERobot
from spatialmath import SE3
from scipy.spatial.transform import Rotation as R
import numpy as np


class TiagoPickAndPlaceTask2(Node):
    def __init__(self):
        super().__init__('tiago_pick_place_task2')

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
                'model_name': 'cocacola',
                'link_name': 'link',
                'pick_above': (-0.0035, -0.045, 0.1),
                'pick_lower': (-0.0035, -0.045, -0.12),
                'pick_lift': (-0.0035, -0.045, 0.18),
                'place_above': (0.0, 0.0, -0.025),
                'place_lower': (0.0, 0.0, -0.02),
                'place_lift': (0.0, 0.0, 0.1),
                'wait_pick_above': 12,
                'wait_pick_lower': 20,
                'wait_attach': 8,
                'wait_pick_lift': 5,
                'wait_place_above': 5,
                'wait_place_lower': 10,
                'wait_detach': 6,
                'wait_place_lift': 10,
                'lower_steps': 3
            },
            {
                'name': 'pringles',
                'pick': 'B',
                'place': 'D',
                'model_name': 'pringles',
                'link_name': 'link',
                'pick_above': (-0.0035, -0.045, -0.04),
                'pick_lower': (-0.0035, -0.045, -0.2),
                'pick_lift': (-0.0035, -0.045, -0.04),
                'place_above': (0.0, 0.0, -0.025),
                'place_lower': (0.0, 0.0, -0.02),
                'place_lift': (-0.0035, -0.045, -0.1),
                'wait_pick_above': 20,
                'wait_pick_lower': 15,
                'wait_attach': 6,
                'wait_pick_lift': 10,
                'wait_place_above': 20,
                'wait_place_lower': 15,
                'wait_detach': 6,
                'wait_place_lift': 10
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

        self.create_subscription(
            GoalStatusArray,
            '/move_head/_action/status',
            self.task1_status_callback,
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

        urdf_loc = '/home/ubuntu/progetto_12_ws/src/my_robot_description/urdf/tiago_robot.urdf'
        self.robot = ERobot.URDF(urdf_loc)

        self.T_pick = SE3.Ry(np.pi / 2) * SE3.Rz(-np.pi / 2)
        self.T_place = SE3.Rx(np.pi / 2) * SE3.Rz(np.pi)

        self.q0_arm = None
        self.q0_torso = None
        self.q0_ready = False

        self.init_saved = False
        self.ready_pose = None
        self.q_sent = None

        self.current_task_index = 0

        self.state = 'SCAN_MARKERS'
        self.wait_until = 0.0
        self.scan_start_time = None
        self.scan_duration = 30.0
        self.unfold_done = False
        self.task1_done = False

        self.last_wait_marker_log = 0.0
        
        # Per movimenti stepwise
        self.stepwise_mode = False
        self.current_step = 0
        self.total_steps = 0
        self.step_positions = []

        self.attach_client = self.create_client(Attach, '/attach')
        self.detach_client = self.create_client(Attach, '/detach')

        while not self.attach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Attesa servizio /attach...')
        while not self.detach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Attesa servizio /detach...')

        self.timer = self.create_timer(0.5, self.loop)
        self.get_logger().info('Nodo Task2 Pick & Place avviato')

    def arm_callback(self, msg):
        self.q0_arm = np.array(msg.actual.positions)
        self.check_q0_ready()

    def torso_callback(self, msg):
        self.q0_torso = np.array(msg.actual.positions)
        self.check_q0_ready()

    def task1_status_callback(self, msg):
        for status in msg.status_list:
            if status.status == GoalStatus.STATUS_SUCCEEDED:
                self.task1_done = True

    def check_q0_ready(self):
        self.q0_ready = self.q0_arm is not None and self.q0_torso is not None

        if self.q0_ready and not self.init_saved:
            self.ready_pose = np.concatenate((self.q0_torso, self.q0_arm))
            self.q_sent = self.ready_pose.copy()
            self.init_saved = True
            self.get_logger().info('Configurazione iniziale salvata')

    def pose_callback(self, label, msg):
        self.marker_poses[label] = msg
        self.get_logger().debug(f'Pose marker {label} ricevuta')

    def open_gripper(self):
        traj = JointTrajectory()
        traj.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
        point = JointTrajectoryPoint()
        point.positions = [0.44, 0.44]
        point.time_from_start.sec = 2
        traj.points.append(point)
        self.gripper_pub.publish(traj)
        self.get_logger().info('Gripper aperto')

    def close_gripper(self):
        traj = JointTrajectory()
        traj.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
        point = JointTrajectoryPoint()
        point.positions = [0.41, 0.41]
        point.time_from_start.sec = 2
        traj.points.append(point)
        self.gripper_pub.publish(traj)
        self.get_logger().info('Gripper chiuso')

    def run_unfold_arm(self):
        self.get_logger().info('Eseguo unfold_arm')
        result = subprocess.run(
            ['ros2', 'run', 'play_motion2', 'run_motion', 'unfold_arm'],
            capture_output=True,
            text=True
        )

        if result.returncode != 0:
            self.get_logger().error(f'unfold_arm fallita: {result.stderr}')
            return False

        self.get_logger().info('unfold_arm completata')
        return True

    def loop(self):
        if not self.q0_ready:
            return

        if self.state == 'SCAN_MARKERS':
            if self.scan_start_time is None:
                self.scan_start_time = time.time()
                self.get_logger().info('Attendo completamento scansione Task1')
                return

            if not self.task1_done:
                if time.time() - self.scan_start_time > self.scan_duration:
                    self.get_logger().info('Task1 non ancora concluso: continuo ad attendere')
                    self.scan_start_time = time.time()
                return

            self.get_logger().info('Scansione Task1 completata')
            self.state = 'UNFOLD_ARM'
            return

        if self.state == 'UNFOLD_ARM':
            if not self.unfold_done:
                self.run_unfold_arm()
                self.unfold_done = True
                self.wait_until = time.time() + 2.0
                self.state = 'SYNC_AFTER_UNFOLD'
                return

        if self.state == 'SYNC_AFTER_UNFOLD':
            if time.time() < self.wait_until:
                return

            self.ready_pose = np.concatenate((self.q0_torso, self.q0_arm))
            self.q_sent = self.ready_pose.copy()
            self.get_logger().info('Posa unfold_arm salvata come posa comoda')

            self.state = 'WAIT_PICK'
            return

        if self.state == 'WAIT_PICK':
            if time.time() < self.wait_until:
                return
            if not self.have_task_poses():
                self.log_missing_task_poses()
                return

            if not hasattr(self, 'gripper_opened_for_pick'):
                self.open_gripper()
                self.gripper_opened_for_pick = True
                self.wait_until = time.time() + 2.0
                return

            task = self.tasks[self.current_task_index]
            self.get_logger().info(f"Muovo verso PICK {task['name']}")

            pick_pose = self.get_pick_pose()
            dx, dy, dz = task['pick_above']
            pos_above = self._get_offset_position(pick_pose, dx=dx, dy=dy, dz=dz)
            TF_above = self._make_SE3(pick_pose, pos_above) * self.T_pick

            q_target = self.robot.ik_NR(TF_above, q0=self.q_sent, pinv=True)[0]
            self.send_trajectory(q_target)
            self.q_sent = q_target

            self.wait_until = time.time() + task['wait_pick_above']
            self.state = 'LOWER_PICK'
            return

        if self.state == 'LOWER_PICK':
            if time.time() < self.wait_until:
                return

            task = self.tasks[self.current_task_index]
            
            # Controlla se questo task usa movimenti stepwise
            if 'lower_steps' in task and not self.stepwise_mode:
                self.get_logger().info(f"Inizio discesa stepwise verso {task['name']} con {task['lower_steps']} step")
                self.stepwise_mode = True
                self.current_step = 0
                self.total_steps = task['lower_steps']
                
                # Calcola le posizioni intermedie tra pick_above e pick_lower
                pick_pose = self.get_pick_pose()
                dx_above, dy_above, dz_above = task['pick_above']
                dx_lower, dy_lower, dz_lower = task['pick_lower']
                
                self.step_positions = []
                for i in range(self.total_steps + 1):
                    t = i / self.total_steps
                    dx = dx_above + (dx_lower - dx_above) * t
                    dy = dy_above + (dy_lower - dy_above) * t
                    dz = dz_above + (dz_lower - dz_above) * t
                    self.step_positions.append((dx, dy, dz))
                
                self.wait_until = time.time() + 1.0
                self.state = 'LOWER_PICK_STEPWISE'
                return
            
            # Movimento normale (per pringles)
            if not self.stepwise_mode:
                self.get_logger().info(f"Abbasso il gripper verso {task['name']}")

                pick_pose = self.get_pick_pose()
                dx, dy, dz = task['pick_lower']
                pos_lower = self._get_offset_position(pick_pose, dx=dx, dy=dy, dz=dz)
                TF_lower = self._make_SE3(pick_pose, pos_lower) * self.T_pick

                q_lower = self.robot.ik_NR(TF_lower, q0=self.q_sent, pinv=True)[0]
                self.send_trajectory(q_lower)
                self.q_sent = q_lower

                self.wait_until = time.time() + task['wait_pick_lower']
                self.state = 'ATTACH'
                return

        if self.state == 'LOWER_PICK_STEPWISE':
            if time.time() < self.wait_until:
                return

            task = self.tasks[self.current_task_index]
            pick_pose = self.get_pick_pose()
            
            # Prendi la posizione dello step corrente
            dx, dy, dz = self.step_positions[self.current_step]
            pos = self._get_offset_position(pick_pose, dx=dx, dy=dy, dz=dz)
            TF = self._make_SE3(pick_pose, pos) * self.T_pick

            try:
                q_step = self.robot.ik_NR(TF, q0=self.q_sent, pinv=True)[0]
                
                self.get_logger().info(f"Step {self.current_step + 1}/{self.total_steps} verso {task['name']}")
                self.send_trajectory(q_step)
                self.q_sent = q_step
                
                # Calcola il wait time proporzionato
                wait_time = task['wait_pick_lower'] / self.total_steps
                self.wait_until = time.time() + wait_time
                
                self.current_step += 1
                
                if self.current_step > self.total_steps:
                    self.get_logger().info('Discesa stepwise completata')
                    self.stepwise_mode = False
                    self.state = 'ATTACH'
                
            except Exception as e:
                self.get_logger().error(f'Step IK fallito: {e}')
                self.stepwise_mode = False
                self.state = 'ATTACH'
            return

        if self.state == 'ATTACH':
            if time.time() < self.wait_until:
                return

            if not hasattr(self, 'gripper_closed'):
                self.close_gripper()
                self.gripper_closed = True
                self.wait_until = time.time() + 2.0
                return

            self.attach_object()
            self.get_logger().info('Oggetto preso!')

            task = self.tasks[self.current_task_index]
            self.wait_until = time.time() + task['wait_attach']
            self.state = 'LIFT_AFTER_PICK'
            return

        if self.state == 'LIFT_AFTER_PICK':
            if time.time() < self.wait_until:
                return

            task = self.tasks[self.current_task_index]
            self.get_logger().info(f"Risalgo con {task['name']}")

            pick_pose = self.get_pick_pose()
            dx, dy, dz = task['pick_lift']
            pos_above = self._get_offset_position(pick_pose, dx=dx, dy=dy, dz=dz)
            TF_above = self._make_SE3(pick_pose, pos_above) * self.T_pick

            q_target = self.robot.ik_NR(TF_above, q0=self.q_sent, pinv=True)[0]
            self.send_trajectory(q_target)
            self.q_sent = q_target

            self.wait_until = time.time() + task['wait_pick_lift']
            self.state = 'MOVE_TO_PLACE'
            return

        if self.state == 'MOVE_TO_PLACE':
            if time.time() < self.wait_until:
                return

            task = self.tasks[self.current_task_index]
            self.get_logger().info(f"Muovo verso PLACE {task['name']}")

            place_pose = self.get_place_pose()
            dx, dy, dz = task['place_above']
            pos_above = self._get_offset_position(place_pose, dx=dx, dy=dy, dz=dz)
            TF_above = self._make_SE3(place_pose, pos_above) * self.T_place

            q_target = self.robot.ik_NR(TF_above, q0=self.q_sent, pinv=True)[0]

            self.send_trajectory(q_target)
            self.q_sent = q_target

            self.wait_until = time.time() + task['wait_place_above']
            self.state = 'LOWER_PLACE'
            return

        if self.state == 'LOWER_PLACE':
            if time.time() < self.wait_until:
                return

            task = self.tasks[self.current_task_index]
            self.get_logger().info(f"Abbasso {task['name']} quasi al piano di place")

            place_pose = self.get_place_pose()
            dx, dy, dz = task['place_lower']
            pos_lower = self._get_offset_position(place_pose, dx=dx, dy=dy, dz=dz)
            TF_lower = self._make_SE3(place_pose, pos_lower) * self.T_place

            q_lower = self.robot.ik_NR(TF_lower, q0=self.q_sent, pinv=True)[0]
            self.send_trajectory(q_lower)
            self.q_sent = q_lower

            self.wait_until = time.time() + task['wait_place_lower']
            self.state = 'DETACH'
            return

        if self.state == 'DETACH':
            if time.time() < self.wait_until:
                return

            self.detach_object()
            self.open_gripper()
            self.get_logger().info('Oggetto rilasciato!')

            task = self.tasks[self.current_task_index]
            self.wait_until = time.time() + task['wait_detach']
            self.state = 'RETURN_SAFE'
            return

        if self.state == 'RETURN_SAFE':
            if time.time() < self.wait_until:
                return

            task = self.tasks[self.current_task_index]
            place_pose = self.get_place_pose()
            dx, dy, dz = task['place_lift']
            pos_safe = self._get_offset_position(place_pose, dx=dx, dy=dy, dz=dz)
            TF_safe = self._make_SE3(place_pose, pos_safe) * self.T_place

            q_safe = self.robot.ik_NR(TF_safe, q0=self.q_sent, pinv=True)[0]
            self.send_trajectory(q_safe)
            self.q_sent = q_safe

            if hasattr(self, 'gripper_opened_for_pick'):
                del self.gripper_opened_for_pick
            if hasattr(self, 'gripper_closed'):
                del self.gripper_closed

            self.current_task_index += 1
            if self.current_task_index >= len(self.tasks):
                self.get_logger().info('Task2 completato: tutti gli oggetti spostati')
                self.state = 'DONE'
                return

            self.get_logger().info('Passo al task successivo')
            self.wait_until = time.time() + task['wait_place_lift']
            self.state = 'WAIT_PICK'
            return

        if self.state == 'DONE':
            return

    def have_task_poses(self):
        task = self.tasks[self.current_task_index]
        return task['pick'] in self.marker_poses and task['place'] in self.marker_poses

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
        self.get_logger().info(f"Attendo marker per {task['name']}: mancanti {missing}")

    def get_pick_pose(self):
        task = self.tasks[self.current_task_index]
        return self.marker_poses[task['pick']]

    def get_place_pose(self):
        task = self.tasks[self.current_task_index]
        return self.marker_poses[task['place']]

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

    def send_trajectory(self, q):
        q = np.array(q).flatten()

        traj_arm = JointTrajectory()
        traj_torso = JointTrajectory()
        traj_arm.joint_names = self.joint_names[1:]
        traj_torso.joint_names = [self.joint_names[0]]

        point_arm = JointTrajectoryPoint()
        point_torso = JointTrajectoryPoint()
        point_arm.positions = [float(x) for x in q[1:]]
        point_torso.positions = [float(q[0])]

        point_arm.time_from_start.sec = 4
        point_torso.time_from_start.sec = 4

        traj_arm.points.append(point_arm)
        traj_torso.points.append(point_torso)
        self.arm_pub.publish(traj_arm)
        self.torso_pub.publish(traj_torso)

    def attach_object(self):
        task = self.tasks[self.current_task_index]
        req = Attach.Request()
        req.model_name_1 = 'tiago'
        req.link_name_1 = 'gripper_left_finger_link'
        req.model_name_2 = task['model_name']
        req.link_name_2 = task.get('link_name', 'link')
        future = self.attach_client.call_async(req)
        future.add_done_callback(functools.partial(self.service_response_callback, 'attach'))
        self.get_logger().info('Richiesta attach inviata')

    def detach_object(self):
        task = self.tasks[self.current_task_index]
        req = Attach.Request()
        req.model_name_1 = 'tiago'
        req.link_name_1 = 'gripper_left_finger_link'
        req.model_name_2 = task['model_name']
        req.link_name_2 = task.get('link_name', 'link')
        future = self.detach_client.call_async(req)
        future.add_done_callback(functools.partial(self.service_response_callback, 'detach'))
        self.get_logger().info('Richiesta detach inviata')

    def service_response_callback(self, operation, future):
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
    node = TiagoPickAndPlaceTask2()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()