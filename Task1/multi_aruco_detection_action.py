#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
import tf2_ros

from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int32MultiArray

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MultiArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector_multi_action')

        # Parameters to adapt script behavior without editing code.
        self.declare_parameter('expected_marker_ids', [3, 4, 26, 63])
        self.declare_parameter('marker_size', 0.06)
        self.declare_parameter('scan_positions', [-0.6, -0.3, 0.0, 0.3, 0.6])
        self.declare_parameter('scan_tilt', -0.57)
        self.declare_parameter('scan_period_sec', 1.2)
        self.declare_parameter('republish_period_sec', 0.1)
        self.declare_parameter('pose_log_period_sec', 2.0)

        self.expected_marker_ids = set(
            int(x) for x in self.get_parameter('expected_marker_ids').value
        )
        self.marker_size = float(self.get_parameter('marker_size').value)
        self.scan_positions = [
            float(x) for x in self.get_parameter('scan_positions').value
        ]
        self.scan_tilt = float(self.get_parameter('scan_tilt').value)
        self.scan_period_sec = float(self.get_parameter('scan_period_sec').value)
        self.republish_period_sec = float(
            self.get_parameter('republish_period_sec').value
        )
        self.pose_log_period_sec = float(
            self.get_parameter('pose_log_period_sec').value
        )

        # Il frame base è quello del robot, che in questo caso è 'base_footprint'
        self.base_frame = 'base_footprint'
        # Il frame della camera è quello del topic camera_info, che in questo caso è 'head_front_camera_color_optical_frame'
        self.camera_frame = 'head_front_camera_color_optical_frame' 

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.detected_ids = set()
        self.all_found_reported = False

        self.last_pose_camera = {}
        self.last_pose_base = {}
        self.last_pose_log_time = {}

        self.camera_publishers = {}
        self.base_publishers = {}

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.marker_tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250) #come richiesto nel punto 3.4
        self.params = cv2.aruco.DetectorParameters_create()

        self.head_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/head_controller/follow_joint_trajectory', #action client su follow_joint_trajectory del controller della testa
        )
        self.joint_names = ['head_1_joint', 'head_2_joint'] #giunti come da traccia
        self.scan_index = 0
        self.head_goal_in_progress = False
        self.warned_head_server_unavailable = False

        self.visible_ids_pub = self.create_publisher(
            Int32MultiArray, #ID visibili in questo momento 
            '/aruco_visible_ids',
            10,
        )
        self.detected_ids_pub = self.create_publisher(
            Int32MultiArray, #ID visti da inizio esecuzione
            '/aruco_detected_ids',
            10,
        )

        self.create_subscription(
            Image,
            '/head_front_camera/rgb/image_raw',
            self.image_callback,
            10,
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/head_front_camera/rgb/camera_info',
            self.camera_info_callback,
            10,
        )

        self.republish_timer = self.create_timer(
            self.republish_period_sec,
            self.republish,
        )
        self.scan_timer = self.create_timer(
            self.scan_period_sec,
            self.scan_if_needed,
        )

        self.get_logger().info(
            f'Multi ArUco detector avviato. Marker attesi: {sorted(self.expected_marker_ids)}'
        )

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        self.destroy_subscription(self.camera_info_sub)
        self.get_logger().info('Camera info ricevuta, detection attiva.')

    def image_callback(self, msg):
        if self.camera_matrix is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.params,
        )

        if ids is None or len(ids) == 0:
            self._publish_ids(self.visible_ids_pub, [])
            self._show_frame(frame)
            return

        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        visible_ids = []

        for marker_id_raw, marker_corners in zip(ids.flatten(), corners):
            marker_id = int(marker_id_raw)
            visible_ids.append(marker_id)

            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                marker_corners,
                self.marker_size,
                self.camera_matrix,
                self.dist_coeffs,
            )

            cv2.aruco.drawAxis(
                frame,
                self.camera_matrix,
                self.dist_coeffs,
                rvec,
                tvec,
                0.03,
            )

            self._ensure_publishers(marker_id)

            pose_cam = self._build_pose_camera(msg, tvec, rvec)
            self.last_pose_camera[marker_id] = pose_cam
            self.camera_publishers[marker_id].publish(pose_cam)

            pose_base = self._build_pose_base(msg, tvec, rvec)
            if pose_base is not None:
                self.last_pose_base[marker_id] = pose_base
                self.base_publishers[marker_id].publish(pose_base)
                self._broadcast_marker_tf(marker_id, pose_base)
                self._log_pose_base(marker_id, pose_base)

            self.detected_ids.add(marker_id)

        self._publish_ids(self.visible_ids_pub, sorted(set(visible_ids)))
        self._publish_ids(self.detected_ids_pub, sorted(self.detected_ids))

        missing = sorted(self.expected_marker_ids - self.detected_ids)
        text = f'Visti: {sorted(self.detected_ids)} | Mancanti: {missing}'
        cv2.putText(
            frame,
            text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        if not self.all_found_reported and self._all_markers_found():
            self.all_found_reported = True
            self.get_logger().info(
                f'Tutti i marker attesi trovati: {sorted(self.expected_marker_ids)}'
            )

        self._show_frame(frame)

    def _build_pose_camera(self, msg, tvec, rvec):
        pose_cam = PoseStamped()
        pose_cam.header.stamp = msg.header.stamp
        pose_cam.header.frame_id = self.camera_frame

        pose_cam.pose.position.x = float(tvec[0][0][0])
        pose_cam.pose.position.y = float(tvec[0][0][1])
        pose_cam.pose.position.z = float(tvec[0][0][2])

        quat = R.from_rotvec(rvec[0][0]).as_quat()
        pose_cam.pose.orientation.x = float(quat[0])
        pose_cam.pose.orientation.y = float(quat[1])
        pose_cam.pose.orientation.z = float(quat[2])
        pose_cam.pose.orientation.w = float(quat[3])

        return pose_cam

    def _build_pose_base(self, msg, tvec, rvec):
        if not self.tf_buffer.can_transform(
            self.base_frame,
            self.camera_frame,
            Time(),
            timeout=Duration(seconds=0.2),
        ):
            return None

        transform = self.tf_buffer.lookup_transform(
            self.base_frame,
            self.camera_frame,
            Time(),
        )

        pos_cam = np.array([tvec[0][0][0], tvec[0][0][1], tvec[0][0][2]])

        q = transform.transform.rotation
        r_camera_to_base = R.from_quat([q.x, q.y, q.z, q.w])
        pos_base = r_camera_to_base.as_matrix() @ pos_cam
        pos_base += np.array(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ]
        )

        r_marker_cam = R.from_rotvec(rvec[0][0])
        r_marker_base = r_camera_to_base * r_marker_cam
        quat_base = r_marker_base.as_quat()

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

        return pose_base

    def _ensure_publishers(self, marker_id):
        if marker_id not in self.camera_publishers:
            topic_cam = f'/aruco_camera_{marker_id}' #topic per pose in camera frame, come da traccia
            self.camera_publishers[marker_id] = self.create_publisher(
                PoseStamped,
                topic_cam,
                10,
            )

        if marker_id not in self.base_publishers:
            topic_base = f'/aruco_base_{marker_id}' #topic per pose in base frame, come da traccia
            self.base_publishers[marker_id] = self.create_publisher(
                PoseStamped,
                topic_base,
                10,
            )

    def _all_markers_found(self):
        if not self.expected_marker_ids:
            return False
        return self.expected_marker_ids.issubset(self.detected_ids)

    def _broadcast_marker_tf(self, marker_id, pose_base):
        tf_msg = TransformStamped()
        tf_msg.header = pose_base.header
        tf_msg.child_frame_id = f'aruco_{marker_id}_base'

        tf_msg.transform.translation.x = pose_base.pose.position.x
        tf_msg.transform.translation.y = pose_base.pose.position.y
        tf_msg.transform.translation.z = pose_base.pose.position.z
        tf_msg.transform.rotation = pose_base.pose.orientation

        self.marker_tf_broadcaster.sendTransform(tf_msg)

    def _log_pose_base(self, marker_id, pose_base):
        now = self.get_clock().now()
        last_time = self.last_pose_log_time.get(marker_id)

        if last_time is None:
            should_log = True
        else:
            elapsed_ns = (now - last_time).nanoseconds
            should_log = elapsed_ns >= int(self.pose_log_period_sec * 1e9)

        if not should_log:
            return

        self.last_pose_log_time[marker_id] = now
        p = pose_base.pose.position
        self.get_logger().info(
            f'Marker {marker_id} in base -> x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}'
        )

    @staticmethod
    def _publish_ids(publisher, values):
        msg = Int32MultiArray()
        msg.data = [int(v) for v in values]
        publisher.publish(msg)

    def scan_if_needed(self):
        if self.camera_matrix is None:
            return

        if not self.expected_marker_ids:
            return

        if self._all_markers_found():
            return

        if not self.scan_positions:
            return

        if self.head_goal_in_progress:
            return

        if not self.head_client.wait_for_server(timeout_sec=0.1):
            if not self.warned_head_server_unavailable:
                self.warned_head_server_unavailable = True
                self.get_logger().warn(
                    'Action server /head_controller/follow_joint_trajectory non disponibile.'
                )
            return

        self.warned_head_server_unavailable = False
        pan = self.scan_positions[self.scan_index]
        self.scan_index = (self.scan_index + 1) % len(self.scan_positions)
        missing = sorted(self.expected_marker_ids - self.detected_ids)
        self.get_logger().info(
            f'Marker mancanti: {missing}. Scansione testa -> pan={pan:.2f}, tilt={self.scan_tilt:.2f}'
        )
        self._send_head_goal(pan, self.scan_tilt)

    def _send_head_goal(self, pan, tilt):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [float(pan), float(tilt)]
        point.time_from_start.sec = 1
        traj.points.append(point)

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
            _ = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Errore risultato action testa: {exc}')

    def republish(self):
        for marker_id, pose in self.last_pose_camera.items():
            pub = self.camera_publishers.get(marker_id)
            if pub is not None:
                pub.publish(pose)

        for marker_id, pose in self.last_pose_base.items():
            pub = self.base_publishers.get(marker_id)
            if pub is not None:
                pub.publish(pose)

        self._publish_ids(self.detected_ids_pub, sorted(self.detected_ids))

    @staticmethod
    def _show_frame(frame):
        cv2.imshow('Aruco Multi', frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = MultiArucoDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
