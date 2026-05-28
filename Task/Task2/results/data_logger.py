#!/usr/bin/env python3
"""
Data Logger per Task2 - Pick & Place
Registra i dati della simulazione per analisi posteriore
"""

import json
import time
from datetime import datetime
from pathlib import Path
import numpy as np

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import PoseStamped


class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger_task2')
        
        # Configurazione
        self.log_dir = Path('/tmp/task2_logs')
        self.log_dir.mkdir(exist_ok=True)
        self.timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # File di output
        self.data_file = self.log_dir / f'task2_data_{self.timestamp}.json'
        self.csv_file = self.log_dir / f'task2_joints_{self.timestamp}.csv'
        
        # Dati raccolti
        self.data = {
            'start_time': time.time(),
            'start_datetime': datetime.now().isoformat(),
            'arm_trajectory': [],
            'torso_trajectory': [],
            'marker_poses': {},
            'gripper_commands': [],
            'attach_detach_events': [],
            'task_events': []
        }
        
        # Subscribers
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
        
        # Marker ArUco
        for marker_id in [1, 2, 3, 4]:
            self.create_subscription(
                PoseStamped,
                f'/aruco_base_{marker_id}',
                lambda msg, mid=marker_id: self.marker_callback(msg, mid),
                10
            )
        
        # Timer per salvare periodicamente
        self.create_timer(5.0, self.save_data)
        
        self.get_logger().info(f'Data Logger avviato. Output: {self.log_dir}')
        self.log_event('DATA_LOGGER_START', 'Logger avviato')
    
    def arm_callback(self, msg):
        """Registra lo stato del braccio"""
        entry = {
            'timestamp': time.time(),
            'positions': list(msg.actual.positions),
            'velocities': list(msg.actual.velocities),
            'efforts': list(msg.actual.efforts),
            'desired_positions': list(msg.desired.positions) if msg.desired.positions else None
        }
        self.data['arm_trajectory'].append(entry)
    
    def torso_callback(self, msg):
        """Registra lo stato del torso"""
        entry = {
            'timestamp': time.time(),
            'positions': list(msg.actual.positions),
            'velocities': list(msg.actual.velocities),
            'efforts': list(msg.actual.efforts)
        }
        self.data['torso_trajectory'].append(entry)
    
    def marker_callback(self, msg, marker_id):
        """Registra pose dei marker ArUco"""
        if f'marker_{marker_id}' not in self.data['marker_poses']:
            self.data['marker_poses'][f'marker_{marker_id}'] = []
        
        entry = {
            'timestamp': time.time(),
            'position': {
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'z': msg.pose.position.z
            },
            'orientation': {
                'x': msg.pose.orientation.x,
                'y': msg.pose.orientation.y,
                'z': msg.pose.orientation.z,
                'w': msg.pose.orientation.w
            }
        }
        self.data['marker_poses'][f'marker_{marker_id}'].append(entry)
    
    def log_event(self, event_type, description):
        """Registra un evento importante"""
        entry = {
            'timestamp': time.time(),
            'type': event_type,
            'description': description
        }
        self.data['task_events'].append(entry)
        self.get_logger().info(f'[{event_type}] {description}')
    
    def save_data(self):
        """Salva i dati in JSON e CSV"""
        try:
            # Salva JSON
            data_copy = self.data.copy()
            data_copy['end_time'] = time.time()
            data_copy['end_datetime'] = datetime.now().isoformat()
            data_copy['duration_seconds'] = data_copy['end_time'] - data_copy['start_time']
            
            with open(self.data_file, 'w') as f:
                json.dump(data_copy, f, indent=2)
            
            # Salva CSV dei giunti
            if self.data['arm_trajectory']:
                self._save_joints_csv()
            
            self.get_logger().debug(f'Dati salvati in {self.data_file}')
        
        except Exception as e:
            self.get_logger().error(f'Errore nel salvataggio: {e}')
    
    def _save_joints_csv(self):
        """Salva la traiettoria dei giunti in CSV"""
        try:
            with open(self.csv_file, 'w') as f:
                # Header
                f.write('timestamp,torso_lift,arm_1,arm_2,arm_3,arm_4,arm_5,arm_6,arm_7\n')
                
                # Merge arm e torso data per timestamp
                arm_dict = {entry['timestamp']: entry for entry in self.data['arm_trajectory']}
                torso_dict = {entry['timestamp']: entry for entry in self.data['torso_trajectory']}
                
                all_times = sorted(set(arm_dict.keys()) | set(torso_dict.keys()))
                
                for ts in all_times:
                    arm_entry = arm_dict.get(ts)
                    torso_entry = torso_dict.get(ts)
                    
                    if arm_entry and torso_entry:
                        positions = [torso_entry['positions'][0]] + arm_entry['positions']
                        f.write(f"{ts}," + ",".join(map(str, positions)) + "\n")
        
        except Exception as e:
            self.get_logger().error(f'Errore nel salvataggio CSV: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.log_event('DATA_LOGGER_STOP', 'Logger interrotto')
        node.save_data()
        print(f"\nDati salvati in:")
        print(f"  JSON: {node.data_file}")
        print(f"  CSV:  {node.csv_file}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
