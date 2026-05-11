#!/usr/bin/env python3

# ==================================================
# Task 2 - Pick & Place per TIAGo
# ==================================================
# Questo nodo ROS2 estende Task1 eseguendo il pick & place di due oggetti:
# - Coca-Cola: da ArUco A a ArUco C
# - Pringles: da ArUco B a ArUco D
#
# Usa la stessa logica a stati del file di riferimento pick_place_cola.py,
# ma adattata per Task2 con mappatura ArUco e sequenza automatica.
#
# Dipendenze: RoboticsToolbox, gazebo_ros_link_attacher, TF2
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


class TiagoPickAndPlaceTask2(Node):
    def __init__(self):
        super().__init__('tiago_pick_place_task2')

        # ==================================================
        # Nomi dei giunti del robot TIAGo
        # ==================================================
        # Questi sono i nomi dei giunti controllati dal controller:
        # - torso_lift_joint: giunto del torso
        # - arm_1_joint ... arm_7_joint: giunti del braccio
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

        # ==================================================
        # Parametri configurabili
        # ==================================================
        # ID dei marker ArUco corrispondenti alle posizioni A, B, C, D
        # Possono essere cambiati se i marker hanno ID diversi
        self.declare_parameter('aruco_A', 26)
        self.declare_parameter('aruco_B', 63)
        self.declare_parameter('aruco_C', 4)
        self.declare_parameter('aruco_D', 3)
        # Percorso del file URDF per RoboticsToolbox
        self.declare_parameter('urdf_path', os.path.expanduser('~/progetto_12_ws/src/my_robot_description/urdf/tiago_robot.urdf'))
        # Se true, il nodo prova a eseguire automaticamente la motion "unfold_arm"
        # tramite play_motion2 prima di iniziare il pick & place.
        self.declare_parameter('run_unfold_arm', True)

        # Mappatura etichette -> ID marker
        self.marker_ids = {
            'A': int(self.get_parameter('aruco_A').value),
            'B': int(self.get_parameter('aruco_B').value),
            'C': int(self.get_parameter('aruco_C').value),
            'D': int(self.get_parameter('aruco_D').value)
        }

        # ==================================================
        # Definizione dei task da eseguire
        # ==================================================
        # Ogni task specifica: nome oggetto, marker pick, marker place, nome modello Gazebo
        self.tasks = [
            {
                'name': 'cocacola',
                'pick': 'A',
                'place': 'C',
                'model_name': 'cocacola'
            },
            {
                'name': 'pringles',
                'pick': 'B',
                'place': 'D',
                'model_name': 'pringles'
            }
        ]

        # ==================================================
        # Subscribers
        # ==================================================
        # Lettura dello stato attuale dei controller per salvare la configurazione iniziale
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

        # ==================================================
        # Subscribers per le pose ArUco
        # ==================================================
        # Crea un subscriber per ogni marker ArUco (A, B, C, D)
        # Usa functools.partial per passare l'etichetta del marker al callback
        self.marker_poses = {}
        for label, marker_id in self.marker_ids.items():
            self.create_subscription(
                PoseStamped,
                f'/aruco_base_{marker_id}',
                functools.partial(self.pose_callback, label),
                10
            )

        # ==================================================
        # Publishers
        # ==================================================
        # Pubblicano traiettorie ai controller del robot
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

        # ==================================================
        # Modello del robot
        # ==================================================
        # Carica il modello URDF di TIAGo usando RoboticsToolbox
        # Questo permette di fare cinematica diretta/inversa
        urdf_loc = os.path.expanduser(str(self.get_parameter('urdf_path').value))
        self.robot = ERobot.URDF(urdf_loc)
        # Trasformazione per allineare il gripper al marker ArUco
        # Ruota di 90° attorno Y e -90° attorno Z per orientare correttamente l'end-effector
        self.T_pick = SE3.Ry(np.pi / 2) * SE3.Rz(-np.pi / 2)

        # ==================================================
        # Variabili di stato
        # ==================================================
        # Stato dei controller: quando entrambi sono pronti, salva la configurazione iniziale
        self.q0_ready = False
        self.q0_arm = None
        self.q0_torso = None
        self.init_saved = False
        # Configurazione "comoda" salvata come stato iniziale del robot
        self.ready_pose = None
        self.ready_pose_after_unfold = None
        # Ultima configurazione inviata ai controller (usata come q0 per IK)
        self.q_sent = None
        self.last_plan_ok = False

        # ==================================================
        # Stato della macchina a stati
        # ==================================================
        # Indice del task corrente (0 = Coca-Cola, 1 = Pringles)
        self.current_task_index = 0
        # Stato corrente della macchina a stati
        self.state = 'UNFOLD_ARM'
        # Timestamp per aspettare tra stati
        self.wait_until = 0.0
        self.last_wait_marker_log = 0.0

        # ==================================================
        # Client per attach/detach
        # ==================================================
        # Service per attaccare/staccare oggetti in Gazebo durante il pick & place
        self.attach_client = self.create_client(Attach, '/attach')
        self.detach_client = self.create_client(Attach, '/detach')

        while not self.attach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Attesa servizio /attach...')

        while not self.detach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Attesa servizio /detach...')

        # ==================================================
        # Timer per il loop principale
        # ==================================================
        # Esegue il loop ogni 0.5 secondi per controllare lo stato e avanzare
        self.timer = self.create_timer(0.5, self.loop)
        self.get_logger().info('Nodo Task2 Pick & Place avviato')
        self.get_logger().info(
            'Marker configurati: '
            f"A=/aruco_base_{self.marker_ids['A']}, "
            f"B=/aruco_base_{self.marker_ids['B']}, "
            f"C=/aruco_base_{self.marker_ids['C']}, "
            f"D=/aruco_base_{self.marker_ids['D']}"
        )

    # ==================================================
    # Callbacks
    # ==================================================
    def arm_callback(self, msg):
        # Salva la configurazione attuale del braccio
        self.q0_arm = np.array(msg.actual.positions)
        self.check_q0_ready()

    def torso_callback(self, msg):
        # Salva la configurazione attuale del torso
        self.q0_torso = np.array(msg.actual.positions)
        self.check_q0_ready()

    def check_q0_ready(self):
        # Controlla se entrambi i controller hanno inviato almeno un messaggio
        self.q0_ready = self.q0_arm is not None and self.q0_torso is not None
        if self.q0_ready and not self.init_saved:
            # Salva la configurazione iniziale come "configurazione comoda"
            self.ready_pose = np.concatenate((self.q0_torso, self.q0_arm))
            self.init_saved = True
            self.q_sent = self.ready_pose.copy()
            self.get_logger().info('Configurazione iniziale salvata')

    def pose_callback(self, label, msg):
        # Salva la posa del marker ArUco ricevuto (label = 'A', 'B', 'C', 'D')
        self.marker_poses[label] = msg
        self.get_logger().debug(f'Pose marker {label} ricevuta')

    # ==================================================
    # Gripper
    # ==================================================
    def open_gripper(self):
        # Invia comando per aprire il gripper
        # Posizioni: 0.044 per entrambi i finger (aperto)
        traj = JointTrajectory()
        traj.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']

        point = JointTrajectoryPoint()
        point.positions = [0.044, 0.044]
        point.time_from_start.sec = 2

        traj.points.append(point)
        self.gripper_pub.publish(traj)
        self.get_logger().info('Gripper aperto')

    def close_gripper(self):
        # Invia comando per chiudere il gripper
        # Posizioni: 0.041 per entrambi i finger (chiuso)
        traj = JointTrajectory()
        traj.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']

        point = JointTrajectoryPoint()
        point.positions = [0.041, 0.041]
        point.time_from_start.sec = 2

        traj.points.append(point)
        self.gripper_pub.publish(traj)
        self.get_logger().info('Gripper chiuso')

    # ==================================================
    # Main state machine
    # ==================================================
    # Loop principale che gestisce la sequenza di stati per il pick & place
    # Esegue ogni 0.5 secondi controllando lo stato corrente
    def loop(self):
        if not self.q0_ready:
            return

        # Prima esecuzione: apri il gripper
        if not hasattr(self, 'gripper_opened'):
            self.open_gripper()
            self.gripper_opened = True
            return

        # ==================================================
        # UNFOLD_ARM: porta automaticamente il braccio in una posa comoda
        # ==================================================
        if self.state == 'UNFOLD_ARM':
            if not self.init_saved:
                return
            self.run_unfold_arm_motion()
            self.wait_until = time.time() + 4
            self.state = 'WAIT_READY'
            return

        # ==================================================
        # WAIT_READY: Aspetta che la configurazione comoda sia raggiunta
        # ==================================================
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

        # ==================================================
        # WAIT_MARKER: Aspetta che le pose ArUco siano disponibili
        # ==================================================
        if self.state == 'WAIT_MARKER':
            if not self.have_task_poses():
                self.log_missing_task_poses()
                return
            if not self.plan_pick_approach():  # Pianifica movimento sopra il pick
                self.state = 'ERROR'
                return
            self.state = 'MOVE_ABOVE_PICK'
            self.wait_until = time.time() + 4  # Aspetta movimento
            return

        # ==================================================
        # MOVE_ABOVE_PICK: Muovi il braccio sopra la posizione di pick
        # ==================================================
        if self.state == 'MOVE_ABOVE_PICK':
            if time.time() < self.wait_until:
                return
            self.get_logger().info('Posiziono il gripper sopra il pick')
            if not self.lower_to_pick():  # Abbassa verso l'oggetto
                self.state = 'ERROR'
                return
            self.state = 'LOWER_PICK'
            self.wait_until = time.time() + 3
            return

        # ==================================================
        # LOWER_PICK: Abbassa il gripper verso l'oggetto
        # ==================================================
        if self.state == 'LOWER_PICK':
            if time.time() < self.wait_until:
                return
            self.get_logger().info('Abbasso verso l\'oggetto')
            self.close_gripper()  # Chiudi il gripper per afferrare
            self.state = 'CLOSE_GRIPPER'
            self.wait_until = time.time() + 2.5
            return

        # ==================================================
        # CLOSE_GRIPPER: Chiudi il gripper e attacca l'oggetto
        # ==================================================
        if self.state == 'CLOSE_GRIPPER':
            if time.time() < self.wait_until:
                return
            self.attach_object()  # Attacca l'oggetto al gripper in Gazebo
            self.get_logger().info('Oggetto attaccato al gripper')
            if not self.plan_place_approach():  # Pianifica movimento verso il place
                self.state = 'ERROR'
                return
            self.state = 'MOVE_TO_PLACE'
            self.wait_until = time.time() + 4
            return

        # ==================================================
        # MOVE_TO_PLACE: Muovi l'oggetto verso la posizione di destinazione
        # ==================================================
        if self.state == 'MOVE_TO_PLACE':
            if time.time() < self.wait_until:
                return
            self.get_logger().info('Sposto il pezzo verso la posizione di place')
            if not self.lower_to_place():  # Abbassa nella posizione finale
                self.state = 'ERROR'
                return
            self.state = 'LOWER_PLACE'
            self.wait_until = time.time() + 3
            return

        # ==================================================
        # LOWER_PLACE: Abbassa l'oggetto nella posizione finale
        # ==================================================
        if self.state == 'LOWER_PLACE':
            if time.time() < self.wait_until:
                return
            self.get_logger().info('Abbasso l\'oggetto nella posizione finale')
            self.open_gripper()  # Apri il gripper per rilasciare
            self.state = 'OPEN_GRIPPER'
            self.wait_until = time.time() + 2.5
            return

        # ==================================================
        # OPEN_GRIPPER: Apri il gripper e stacca l'oggetto
        # ==================================================
        if self.state == 'OPEN_GRIPPER':
            if time.time() < self.wait_until:
                return
            self.detach_object()  # Stacca l'oggetto dal gripper in Gazebo
            self.get_logger().info('Oggetto staccato')
            self.open_gripper()
            self.go_to_ready_pose()
            self.state = 'RETURN_SAFE'
            self.wait_until = time.time() + 3
            return

        # ==================================================
        # RETURN_SAFE: Torna in posizione sicura e passa al task successivo
        # ==================================================
        if self.state == 'RETURN_SAFE':
            if time.time() < self.wait_until:
                return
            self.current_task_index += 1  # Passa al task successivo
            if self.current_task_index >= len(self.tasks):
                self.get_logger().info('Task2 completato: tutti gli oggetti sono stati spostati')
                self.state = 'DONE'
                return
            self.get_logger().info('Passo successivo: nuovo oggetto da spostare')
            self.state = 'WAIT_MARKER'  # Torna ad aspettare i marker per il nuovo task
            return

        if self.state == 'ERROR':
            self.get_logger().error(
                'Task2 interrotto: pianificazione o IK fallita. '
                'Verificare marker, offset, URDF e controller prima di riprovare.'
            )
            self.state = 'DONE'
            return

    # ==================================================
    # Planning helpers
    # ==================================================
    # Metodi per controllare la disponibilità delle pose e pianificare i movimenti

    def have_task_poses(self):
        # Controlla se le pose necessarie per il task corrente sono disponibili
        task = self.tasks[self.current_task_index]
        pick_label = task['pick']
        place_label = task['place']
        return pick_label in self.marker_poses and place_label in self.marker_poses

    def log_missing_task_poses(self):
        now = time.time()
        if now - self.last_wait_marker_log < 5.0:
            return
        self.last_wait_marker_log = now

        task = self.tasks[self.current_task_index]
        required_labels = [task['pick'], task['place']]
        missing = [
            f"{label} (/aruco_base_{self.marker_ids[label]})"
            for label in required_labels
            if label not in self.marker_poses
        ]
        received = sorted(self.marker_poses.keys())
        self.get_logger().info(
            f"Attendo marker per {task['name']}: mancanti {missing}; ricevuti {received}"
        )

    def get_current_task_poses(self):
        # Restituisce le pose di pick e place per il task corrente
        task = self.tasks[self.current_task_index]
        return self.marker_poses[task['pick']], self.marker_poses[task['place']]

    def plan_pick_approach(self):
        # Pianifica il movimento sopra la posizione di pick
        # Applica offset per avvicinarsi dall'alto con orientamento corretto
        pick_pose, _ = self.get_current_task_poses()
        pos_above = self._get_offset_position(pick_pose, dx=-0.0035, dy=-0.045, dz=-0.10)
        target = self._make_SE3(pick_pose, pos_above) * self.T_pick
        q_target = self.solve_ik(target)
        if q_target is not None:
            self.send_trajectory(q_target)
            self.q_sent = q_target
            return True
        return False

    def lower_to_pick(self):
        # Abbassa il gripper nella posizione di pick
        pick_pose, _ = self.get_current_task_poses()
        pos_lower = self._get_offset_position(pick_pose, dx=-0.0035, dy=-0.045, dz=-0.14)
        target = self._make_SE3(pick_pose, pos_lower) * self.T_pick
        q_target = self.solve_ik(target)
        if q_target is not None:
            self.send_trajectory(q_target)
            self.q_sent = q_target
            return True
        return False

    def plan_place_approach(self):
        # Pianifica il movimento sopra la posizione di place
        _, place_pose = self.get_current_task_poses()
        pos_above = self._get_offset_position(place_pose, dx=-0.0035, dy=-0.045, dz=-0.10)
        target = self._make_SE3(place_pose, pos_above) * self.T_pick
        q_target = self.solve_ik(target)
        if q_target is not None:
            self.send_trajectory(q_target)
            self.q_sent = q_target
            return True
        return False

    def lower_to_place(self):
        # Abbassa l'oggetto nella posizione finale di place
        _, place_pose = self.get_current_task_poses()
        pos_lower = self._get_offset_position(place_pose, dx=-0.0035, dy=-0.045, dz=-0.14)
        target = self._make_SE3(place_pose, pos_lower) * self.T_pick
        q_target = self.solve_ik(target)
        if q_target is not None:
            self.send_trajectory(q_target)
            self.q_sent = q_target
            return True
        return False

    def solve_ik(self, pose):
        # Risolvi l'inverse kinematics per raggiungere la posa desiderata
        # Usa la configurazione precedente come punto di partenza
        try:
            q0 = self.q_sent if self.q_sent is not None else self.ready_pose
            solution = self.robot.ik_NR(pose, q0=q0, pinv=True)
            if solution is None or len(solution) == 0:
                raise ValueError('IK fallita')
            if hasattr(solution, 'success') and not solution.success:
                raise ValueError(f'IK non convergente: {solution}')
            if len(solution) > 1 and isinstance(solution[1], (bool, np.bool_)) and not solution[1]:
                raise ValueError(f'IK non convergente: {solution}')
            q = np.array(solution[0]).flatten()
            if len(q) != len(self.joint_names):
                raise ValueError(f'IK ha restituito {len(q)} giunti, attesi {len(self.joint_names)}')
            return q
        except Exception as ex:
            self.get_logger().warn(f'IK fallita: {ex}')
            return None

    # ==================================================
    # Utility methods
    # ==================================================
    # Metodi di supporto per calcoli geometrici e invio traiettorie

    def _get_offset_position(self, pose, dx=0.0, dy=0.0, dz=0.0):
        # Applica un offset alla posizione della posa
        # Usato per calcolare posizioni sopra/sotto gli oggetti
        return np.array([
            pose.pose.position.x + dx,
            pose.pose.position.y + dy,
            pose.pose.position.z + dz
        ])

    def _make_SE3(self, pose, position):
        # Crea una trasformazione SE3 dalla posa ROS (quaternione) e posizione
        # Converte il quaternione in matrice di rotazione
        quat = [
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w
        ]
        R_mat = R.from_quat(quat).as_matrix()
        return SE3.Rt(R_mat, position)

    def send_trajectory(self, q):
        # Invia una traiettoria ai controller del braccio e torso
        # q è un array con [torso, arm_1, arm_2, ..., arm_7]
        q = np.array(q).flatten()

        traj_arm = JointTrajectory()
        traj_torso = JointTrajectory()

        traj_arm.joint_names = self.joint_names[1:]  # Solo giunti del braccio
        traj_torso.joint_names = [self.joint_names[0]]  # Solo giunto del torso

        point_arm = JointTrajectoryPoint()
        point_torso = JointTrajectoryPoint()

        point_arm.positions = [float(x) for x in q[1:]]
        point_torso.positions = [float(q[0])]

        point_arm.time_from_start.sec = 3  # Tempo per raggiungere la posizione
        point_torso.time_from_start.sec = 3

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
            self.get_logger().warn(
                f'unfold_arm ha restituito codice {completed.returncode}; uso la posa corrente come fallback'
            )
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
        # Attacca l'oggetto corrente al gripper in Gazebo
        # Questo permette di simulare il pick & place fisico
        req = Attach.Request()
        req.model_name_1 = 'tiago'  # Modello del robot
        req.link_name_1 = 'gripper_left_finger_link'  # Link del gripper
        req.model_name_2 = self.tasks[self.current_task_index]['model_name']  # Oggetto da attaccare
        req.link_name_2 = 'link'  # Link dell'oggetto
        future = self.attach_client.call_async(req)
        future.add_done_callback(functools.partial(self.attach_response_callback, 'attach'))
        self.get_logger().info('Richiesta attach inviata')

    def detach_object(self):
        # Stacca l'oggetto corrente dal gripper in Gazebo
        req = Attach.Request()
        req.model_name_1 = 'tiago'
        req.link_name_1 = 'gripper_left_finger_link'
        req.model_name_2 = self.tasks[self.current_task_index]['model_name']
        req.link_name_2 = 'link'
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
    # Funzione principale per avviare il nodo ROS2
    rclpy.init(args=args)
    node = TiagoPickAndPlaceTask2()

    try:
        rclpy.spin(node)  # Esegue il loop del nodo fino a interruzione
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
