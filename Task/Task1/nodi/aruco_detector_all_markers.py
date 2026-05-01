import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped

import cv2 #OpenCV per ArUco
import numpy as np 
from cv_bridge import CvBridge #per convertire ROS Image in OpenCV Image (comunicazione tra ROS e OpenCV)

import tf2_ros #solo per trasformazioni
from scipy.spatial.transform import Rotation as R


class ArucoDetector(Node):

    def __init__(self):
        super().__init__('aruco_detector_all_markers')

        # Subscriber
        self.create_subscription(
            Image,
            '/head_front_camera/rgb/image_raw', #tipo di messaggio contenente l'immagine image_raw
            self.image_callback, #obbligatoria con subscriber, è la funzione che viene chiamata quando arriva un messaggio e per leggere immagine
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/head_front_camera/rgb/camera_info', #si sottoscrive camera_info e callback
            self.camera_info_callback,
            10 #queue size
        )

        # Timer republish
        self.timer = self.create_timer(0.1, self.republish) #ogni 100ms ripubblica ultima informazione, se non è stata aggiornata da camera_info_callback o image_callback

    #ripubblica posa anche se tiago non la vede più

        self.bridge = CvBridge()
        self.camera_matrix = None # matrice intrinseca della camera, inizialmente None, poi viene aggiornata quando arriva il messaggio camera_info
        self.dist_coeffs = None  # distorsione della camera, inizialmente None, poi viene aggiornata quando arriva il messaggio camera_info

        # Vengono memorizzate le pose di TUTTI i marker
        # Dizionari che associano ID marker: (ultima posa camera, ultima posa base)
        self.last_poses_camera = {}  # {marker_id: pose_camera}
        self.last_poses_base = {}    # {marker_id: pose_base}
        self.last_detected_ids = []  # Lista degli ID rilevati nell'ultimo frame
        self.last_logged_ids = []
        self.last_detection_log_stamp = None

        # Publisher dinamici (UNO per ID)
        self.camera_publishers = {} #dizionario che associa a ogni ID del marker un publisher per la posa in terna camera
        self.base_publishers = {}

        # TF
        self.tf_buffer = tf2_ros.Buffer() #buffer per memorizzare le trasformazioni, permette di fare lookup delle trasformazioni tra frame
        #memorizzazione istante per istante informazione catena cinematica del robot
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self) #listener per ascoltare le trasformazioni, si collega al buffer e al nodo, permette di aggiornare il buffer con le trasformazioni che arrivano

        self.base_frame = 'base_footprint' #frame di riferimento per la posa in terna base, è il frame del robot, in cui vogliamo esprimere la posa del marker
        self.camera_frame = 'head_front_camera_color_optical_frame' #frame di riferimento per la posa in terna camera, è il frame della camera, in cui esprimiamo la posa del marker quando lo rileviamo
 
        # ArUco
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.params = cv2.aruco.DetectorParameters_create()
        self.marker_size = 0.06

    def camera_info_callback(self, msg): 
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        self.destroy_subscription(self.camera_info_sub)

    def image_callback(self, msg):

        if self.camera_matrix is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers( #rileva i marker e restituisce le coordinate degli angoli. ids array nx1 con n numero di marker, 
        # corners array n x 1 x 4 x 2 con le coordinate degli angoli dei marker rilevati con 4:vertici e 2: coordinate x e y
            gray,
            self.aruco_dict,
            parameters=self.params
        )

        if ids is None:
            cv2.imshow("Aruco", frame)
            cv2.waitKey(1)
            return

        detected_ids = [int(marker_id[0]) for marker_id in ids]

        now_stamp = self.get_clock().now()
        should_log_detection = (
            detected_ids != self.last_logged_ids or
            self.last_detection_log_stamp is None or
            (now_stamp - self.last_detection_log_stamp).nanoseconds > 1_000_000_000
        )

        if should_log_detection:
            self.get_logger().info(f"ArUco rilevati: {detected_ids}")
            self.last_logged_ids = detected_ids.copy()
            self.last_detection_log_stamp = now_stamp

        # Stima la posa di TUTTI i marker in una sola chiamata
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners,
            self.marker_size,
            self.camera_matrix,
            self.dist_coeffs
        )

        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        # Disegna gli assi per il primo marker
        cv2.aruco.drawAxis(frame, self.camera_matrix, self.dist_coeffs, rvec[0], tvec[0], 0.03)

        # Resetta la lista degli ID rilevati in questo frame
        self.last_detected_ids = []

        # ==========================
        # LOOP SU TUTTI I MARKER RILEVATI
        # ==========================
        # Itera su TUTTI i marker rilevati
        for i in range(len(ids)):
            marker_id = int(ids[i][0])
            self.last_detected_ids.append(marker_id)

            # ==========================
            # CREATE PUBLISHER DINAMICI
            # ==========================
            if marker_id not in self.camera_publishers:
                topic_cam = f"/aruco_camera_{marker_id}"
                self.camera_publishers[marker_id] = self.create_publisher(PoseStamped, topic_cam, 10)
                #si scrive tipo di messaggio (pose stamped), nome del topic (aruco_camera_ID) e queue size (10)

            if marker_id not in self.base_publishers:
                topic_base = f"/aruco_base_{marker_id}"
                self.base_publishers[marker_id] = self.create_publisher(PoseStamped, topic_base, 10)

            # ==========================
            # POSE CAMERA
            # ==========================
            pose_cam = PoseStamped()
            #crea un messaggio di tipo PoseStamped, che contiene la posa del marker rispetto alla camera
            pose_cam.header.stamp = msg.header.stamp
            pose_cam.header.frame_id = self.camera_frame

            # Accede all'i-esimo marker nel vettore di risultati
            # rvec e tvec sono array n x 3 (per n marker rilevati)
            pose_cam.pose.position.x = float(tvec[i][0][0])
            # Estrae coordinate Y e Z del marker dalla camera
            # Descrive traslazione lungo asse x y e z
            pose_cam.pose.position.y = float(tvec[i][0][1])
            pose_cam.pose.position.z = float(tvec[i][0][2])

            # Converte il vettore di rotazione (rvec) in quaternione e assegna alla pose
            quat = R.from_rotvec(rvec[i][0]).as_quat()
            pose_cam.pose.orientation.x = float(quat[0])
            pose_cam.pose.orientation.y = float(quat[1])
            pose_cam.pose.orientation.z = float(quat[2])
            pose_cam.pose.orientation.w = float(quat[3])

            # Salva le pose di TUTTI i marker in dizionari
            self.last_poses_camera[marker_id] = pose_cam

            # Pubblica la posa del marker nel frame della camera
            self.camera_publishers[marker_id].publish(pose_cam)

            self.get_logger().info(
                f"Marker {marker_id} | camera: "
                f"x={pose_cam.pose.position.x:.3f}, "
                f"y={pose_cam.pose.position.y:.3f}, "
                f"z={pose_cam.pose.position.z:.3f}"
            )

            # ==========================
            # CALCOLA POSE NEL FRAME BASE
            # ==========================
            # Verifica se la trasformazione camera->base è disponibile
            if self.tf_buffer.can_transform(
                self.base_frame,
                self.camera_frame,
                Time(),
                timeout=Duration(seconds=0.2)
            ):
# Prende trasformazione con tf buffer mediante lookup trasform, dal frame sorgente (camera frame) al frame base (base footprint)
                # Recupera la trasformazione da camera a base
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.camera_frame,
                    Time()
                ) 
                # Estrae la posizione del marker nel frame della camera
                # Trasforma in numpy array
                # Accede all'i-esimo marker
                pos_cam = np.array([tvec[i][0][0], tvec[i][0][1], tvec[i][0][2]])

                # Recupera la rotazione della trasformazione camera->base
                # Ricostruzione della catena
                q = transform.transform.rotation #accesso al quaternione
                # Rotazione (r) a partire dal quaternione. Prendiamo gli elementi
                r = R.from_quat([q.x, q.y, q.z, q.w]) # Ricaviamo matrice di rotazione dalla camera alla base 

                # Trasforma la posizione dal frame camera al frame base
                pos_base = r.as_matrix() @ pos_cam
                # Rispetto alla base cerchiamo la posizione. Per avere la posizione in terna base,
                #consideriamo la matrice di rotazione tra camera e base (r) e la moltiplichiamo per la posizione del marker 
                #rispetto alla camera (pos_cam).

                #Prendiamo traslazione OO' per ricostruire matrice di trasformazione omogenea A1^0
                pos_base += np.array([
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z
                ])

                # Trasforma la rotazione del marker dal frame camera al frame base
                # Rotazione del marker in terna camera
                r_marker = R.from_rotvec(rvec[i][0])
                # Rotazione in terna base del marker, considerando la rotazione tra camera e base (r) e la rotazione 
                # del marker in terna camera (r_marker)
                r_base = r * r_marker
                # Il quaternione in terna base del marker è dato dalla rotazione tra camera e base (r) 
                # moltiplicata per la rotazione del marker in terna camera (r_marker).
                quat_base = r_base.as_quat()

                # Posebase è messaggio di tipo PoseStamped che contiene la posa 
                # del marker rispetto al frame base, con posizione e orientazione
                # Nell'header ha id
                pose_base = PoseStamped()
                pose_base.header.stamp = msg.header.stamp
                pose_base.header.frame_id = self.base_frame


                #Posizione
                pose_base.pose.position.x = float(pos_base[0])
                pose_base.pose.position.y = float(pos_base[1])
                pose_base.pose.position.z = float(pos_base[2])

                #Orientazione
                pose_base.pose.orientation.x = float(quat_base[0])
                pose_base.pose.orientation.y = float(quat_base[1])
                pose_base.pose.orientation.z = float(quat_base[2])
                pose_base.pose.orientation.w = float(quat_base[3])

                # Mantiene le informazioni sulle posizioni di TUTTI i marker per fare pick and place
                # Memorizziamo tutte le pose base di ogni marker rilevato
                self.last_poses_base[marker_id] = pose_base
                self.base_publishers[marker_id].publish(pose_base)

                self.get_logger().info(
                    f"Marker {marker_id} | base: "
                    f"x={pose_base.pose.position.x:.3f}, "
                    f"y={pose_base.pose.position.y:.3f}, "
                    f"z={pose_base.pose.position.z:.3f}"
                )
            else:
                self.get_logger().warn(
                    f"Marker {marker_id} rilevato nel frame camera, ma la trasformazione verso base non e' disponibile."
                )

        # Visualizza il frame con i marker rilevati
        cv2.imshow("Aruco", frame)
        cv2.waitKey(1)

        #Mostra ARUCO marker nell'ambiente 

    # ==========================
    # REPUBLISH
    # ==========================
    def republish(self):
        """Ripubblica la pose di TUTTI i marker rilevati nell'ultimo frame"""
        
        # Itera su tutti gli ID rilevati nell'ultimo frame
        for marker_id in self.last_detected_ids:
            # Ripubblica la pose nel frame della camera
            if marker_id in self.last_poses_camera:
                self.camera_publishers[marker_id].publish(self.last_poses_camera[marker_id])

            # Ripubblica la pose nel frame base se disponibile
            if marker_id in self.last_poses_base:
                self.base_publishers[marker_id].publish(self.last_poses_base[marker_id])


def main(args=None):
    """Inizializza il nodo ROS2 e avvia la rilevazione ArUco di TUTTI i marker"""
    rclpy.init(args=args)
    node = ArucoDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Cleanup: destroy node e shutdown
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
