import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped

import cv2 #OpenCV per ArUco
import numpy as np 
from cv_bridge import CvBridge #per convertire ROS Image in OpenCV Image

import tf2_ros #solo per trasformazioni
from scipy.spatial.transform import Rotation as R #solo per rotazioni (evitiamo di fare tutto a mano)


class ArucoDetector(Node):

    def __init__(self):
        super().__init__('aruco_detector_simple')

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
            10 #queue size: permette di accumulare n numero di informazioni e per svuota la memoria. 10 numero aureo
        )

        # Timer republish
        self.timer = self.create_timer(0.1, self.republish) #ogni 100ms ripubblica ultima informazione, se non è stata aggiornata da camera_info_callback o image_callback

    #ripubblica posa anche se tiago non la vede più

        self.bridge = CvBridge()
        self.camera_matrix = None # matrice intrinseca della camera, inizialmente None, poi viene aggiornata quando arriva il messaggio camera_info
        self.dist_coeffs = None  # distorsione della camera, inizialmente None, poi viene aggiornata quando arriva il messaggio camera_info

        self.last_pose_camera = None #posa in terna camera
        self.last_pose_base = None #posa in terna base
        self.last_id = None #ID dell'ultimo marker visto, inizialmente None, poi viene aggiornato quando arriva un messaggio image con marker rilevato

        # Publisher dinamici (UNO per ID)
        self.camera_publishers = {} #dizionario che associa a ogni ID del marker un publisher per la posa in terna camera
        self.base_publishers = {}

        # TF
        self.tf_buffer = tf2_ros.Buffer() #buffer per memorizzare le trasformazioni, permette di fare lookup delle trasformazioni tra frame
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

        # 👉 SOLO PRIMO MARKER
        marker_id = int(ids[0][0])

        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners[0],
            self.marker_size,
            self.camera_matrix,
            self.dist_coeffs
        )

        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        cv2.aruco.drawAxis(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.03)

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

        pose_cam.pose.position.x = float(tvec[0][0][0])
        # Estrai coordinate Y e Z del marker dalla camera (tvec è array 1x1x3)
        pose_cam.pose.position.y = float(tvec[0][0][1])
        pose_cam.pose.position.z = float(tvec[0][0][2])

        # Converti il vettore di rotazione (rvec) in quaternione e assegna alla pose
        quat = R.from_rotvec(rvec[0][0]).as_quat()
        pose_cam.pose.orientation.x = float(quat[0])
        pose_cam.pose.orientation.y = float(quat[1])
        pose_cam.pose.orientation.z = float(quat[2])
        pose_cam.pose.orientation.w = float(quat[3])

        # Salva l'ultima pose e ID per republish
        self.last_pose_camera = pose_cam
        self.last_id = marker_id

        # Pubblica la pose del marker nel frame della camera
        self.camera_publishers[marker_id].publish(pose_cam)

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

            # Recupera la trasformazione da camera a base
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_frame,
                Time()
            )

            # Estrai la posizione del marker nel frame della camera
            pos_cam = np.array([tvec[0][0][0], tvec[0][0][1], tvec[0][0][2]])

            # Recupera la rotazione della trasformazione camera->base
            q = transform.transform.rotation
            r = R.from_quat([q.x, q.y, q.z, q.w])

            # Trasforma la posizione dal frame camera al frame base
            pos_base = r.as_matrix() @ pos_cam
            pos_base += np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ])

            # Trasforma la rotazione del marker dal frame camera al frame base
            r_marker = R.from_rotvec(rvec[0][0])
            r_base = r * r_marker
            quat_base = r_base.as_quat()

            # Crea e popola il messaggio di pose nel frame base
            pose_base = PoseStamped()
            pose_base.header.stamp = msg.header.stamp
            pose_base.header.frame_id = self.base_frame

            pose_base.pose.position.x = float(pos_base[0])
            pose_base.pose.position.y = float(pos_base[1])
            pose_base.pose.position.z = float(pos_base[2])

            pose_base.pose.orientation.x = float(quat_base[0])
            pose_base.pose.orientation.y = float(quat_base[1])
            pose_base.pose.orientation.z = float(quat_base[2])
            pose_base.pose.orientation.w = float(quat_base[3])

            # Salva e pubblica la pose del marker nel frame base
            self.last_pose_base = pose_base
            self.base_publishers[marker_id].publish(pose_base)

        # Visualizza il frame con i marker rilevati
        cv2.imshow("Aruco", frame)
        cv2.waitKey(1)

    # ==========================
    # REPUBLISH ULTIMO MARKER
    # ==========================
    def republish(self):
        """Ripubblica la pose dell'ultimo marker rilevato"""
        
        if self.last_id is None:
            return

        # Ripubblica la pose nel frame della camera
        if self.last_pose_camera is not None:
            self.camera_publishers[self.last_id].publish(self.last_pose_camera)

        # Ripubblica la pose nel frame base se disponibile
        if self.last_pose_base is not None:
            self.base_publishers[self.last_id].publish(self.last_pose_base)


def main(args=None):
    """Inizializza il nodo ROS2 e avvia la rilevazione ArUco"""
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
































