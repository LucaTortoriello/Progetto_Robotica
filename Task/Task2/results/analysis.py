#!/usr/bin/env python3
"""
Analysis Script per Task2 - Pick & Place
Genera grafici e tabelle dai dati registrati
"""

import json
import csv
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from datetime import datetime


class Task2Analysis:
    def __init__(self, json_file):
        self.json_file = Path(json_file)
        self.output_dir = self.json_file.parent / 'analysis_output'
        self.output_dir.mkdir(exist_ok=True)
        
        # Carica dati
        with open(self.json_file, 'r') as f:
            self.data = json.load(f)
        
        self.start_time = self.data['start_time']
        self.joint_names = ['Torso', 'Arm1', 'Arm2', 'Arm3', 'Arm4', 'Arm5', 'Arm6', 'Arm7']
        
        print(f"Dati caricati da {self.json_file}")
        print(f"Durata simulazione: {self.data.get('duration_seconds', 0):.2f}s")
    
    def plot_joint_trajectories(self):
        """Grafico delle traiettorie dei giunti nel tempo"""
        if not self.data['arm_trajectory']:
            print("Nessun dato arm_trajectory disponibile")
            return
        
        fig, axes = plt.subplots(4, 2, figsize=(15, 12))
        fig.suptitle('Traiettorie dei Giunti - Task2 Pick & Place', fontsize=16, fontweight='bold')
        
        # Merge arm e torso
        arm_data = self.data['arm_trajectory']
        torso_data = self.data['torso_trajectory']
        
        # Crea dizionari per accesso veloce
        arm_dict = {entry['timestamp']: entry for entry in arm_data}
        torso_dict = {entry['timestamp']: entry for entry in torso_data}
        all_times = sorted(set(arm_dict.keys()) | set(torso_dict.keys()))
        
        # Dati combinati
        all_joints = []
        time_array = np.array(all_times) - self.start_time
        
        for ts in all_times:
            arm_entry = arm_dict.get(ts)
            torso_entry = torso_dict.get(ts)
            if arm_entry and torso_entry:
                joint_config = [torso_entry['positions'][0]] + arm_entry['positions']
                all_joints.append(joint_config)
        
        all_joints = np.array(all_joints)
        
        # Plot 8 giunti
        for i in range(8):
            ax = axes[i // 2, i % 2]
            ax.plot(time_array, all_joints[:, i], 'b-', linewidth=2, label='Posizione reale')
            ax.set_xlabel('Tempo (s)')
            ax.set_ylabel('Posizione (rad)')
            ax.set_title(f'{self.joint_names[i]}')
            ax.grid(True, alpha=0.3)
            ax.legend()
        
        plt.tight_layout()
        fig.savefig(self.output_dir / 'joint_trajectories.png', dpi=150, bbox_inches='tight')
        print("✓ Salvato: joint_trajectories.png")
        plt.close()
    
    def plot_marker_positions(self):
        """Grafico delle posizioni dei marker ArUco nel tempo"""
        if not self.data['marker_poses']:
            print("Nessun dato marker disponibile")
            return
        
        fig = plt.figure(figsize=(15, 10))
        gs = gridspec.GridSpec(2, 2, figure=fig)
        
        fig.suptitle('Posizioni Marker ArUco - Task2 Pick & Place', fontsize=16, fontweight='bold')
        
        markers_info = {
            'marker_1': ('B - Pringles Pick', 'red'),
            'marker_2': ('A - Coca-Cola Pick', 'blue'),
            'marker_3': ('C - Coca-Cola Place', 'green'),
            'marker_4': ('D - Pringles Place', 'orange')
        }
        
        for idx, (marker_key, (label, color)) in enumerate(markers_info.items()):
            if marker_key not in self.data['marker_poses']:
                continue
            
            poses = self.data['marker_poses'][marker_key]
            if not poses:
                continue
            
            times = np.array([p['timestamp'] - self.start_time for p in poses])
            xs = np.array([p['position']['x'] for p in poses])
            ys = np.array([p['position']['y'] for p in poses])
            zs = np.array([p['position']['z'] for p in poses])
            
            # Subplot 1: X, Y, Z nel tempo
            ax = fig.add_subplot(gs[idx // 2, idx % 2])
            ax.plot(times, xs, 'r-', label='X', linewidth=2)
            ax.plot(times, ys, 'g-', label='Y', linewidth=2)
            ax.plot(times, zs, 'b-', label='Z', linewidth=2)
            ax.set_xlabel('Tempo (s)')
            ax.set_ylabel('Posizione (m)')
            ax.set_title(f'{label} (Marker {marker_key[-1]})')
            ax.grid(True, alpha=0.3)
            ax.legend()
        
        plt.tight_layout()
        fig.savefig(self.output_dir / 'marker_positions.png', dpi=150, bbox_inches='tight')
        print("✓ Salvato: marker_positions.png")
        plt.close()
    
    def plot_joint_velocities(self):
        """Grafico delle velocità dei giunti"""
        if not self.data['arm_trajectory']:
            return
        
        arm_data = self.data['arm_trajectory']
        torso_data = self.data['torso_trajectory']
        
        arm_dict = {entry['timestamp']: entry for entry in arm_data}
        torso_dict = {entry['timestamp']: entry for entry in torso_data}
        all_times = sorted(set(arm_dict.keys()) | set(torso_dict.keys()))
        
        all_velocities = []
        time_array = np.array(all_times) - self.start_time
        
        for ts in all_times:
            arm_entry = arm_dict.get(ts)
            torso_entry = torso_dict.get(ts)
            if arm_entry and torso_entry:
                vel_config = [torso_entry['velocities'][0]] + arm_entry['velocities']
                all_velocities.append(vel_config)
        
        all_velocities = np.array(all_velocities)
        
        fig, axes = plt.subplots(4, 2, figsize=(15, 12))
        fig.suptitle('Velocità dei Giunti - Task2 Pick & Place', fontsize=16, fontweight='bold')
        
        for i in range(8):
            ax = axes[i // 2, i % 2]
            ax.plot(time_array, all_velocities[:, i], 'g-', linewidth=2)
            ax.set_xlabel('Tempo (s)')
            ax.set_ylabel('Velocità (rad/s)')
            ax.set_title(f'{self.joint_names[i]}')
            ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        fig.savefig(self.output_dir / 'joint_velocities.png', dpi=150, bbox_inches='tight')
        print("✓ Salvato: joint_velocities.png")
        plt.close()
    
    def generate_statistics_table(self):
        """Genera tabella con statistiche dei giunti"""
        if not self.data['arm_trajectory']:
            return
        
        arm_data = self.data['arm_trajectory']
        torso_data = self.data['torso_trajectory']
        
        arm_dict = {entry['timestamp']: entry for entry in arm_data}
        torso_dict = {entry['timestamp']: entry for entry in torso_data}
        all_times = sorted(set(arm_dict.keys()) | set(torso_dict.keys()))
        
        all_joints = []
        for ts in all_times:
            arm_entry = arm_dict.get(ts)
            torso_entry = torso_dict.get(ts)
            if arm_entry and torso_entry:
                joint_config = [torso_entry['positions'][0]] + arm_entry['positions']
                all_joints.append(joint_config)
        
        all_joints = np.array(all_joints)
        
        # Calcola statistiche
        stats = []
        for i in range(8):
            joint_traj = all_joints[:, i]
            stats.append({
                'Giunto': self.joint_names[i],
                'Min (rad)': f"{np.min(joint_traj):.4f}",
                'Max (rad)': f"{np.max(joint_traj):.4f}",
                'Media (rad)': f"{np.mean(joint_traj):.4f}",
                'Std Dev (rad)': f"{np.std(joint_traj):.4f}",
                'Range (rad)': f"{np.max(joint_traj) - np.min(joint_traj):.4f}"
            })
        
        # Salva in CSV
        csv_file = self.output_dir / 'statistics_joints.csv'
        if stats:
            with open(csv_file, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=stats[0].keys())
                writer.writeheader()
                writer.writerows(stats)
        
        print(f"✓ Salvato: statistics_joints.csv")
        
        # Stampa tabella
        print("\n" + "="*80)
        print("STATISTICHE GIUNTI")
        print("="*80)
        print(f"{'Giunto':<12} {'Min (rad)':<15} {'Max (rad)':<15} {'Media':<15} {'Std Dev':<15} {'Range':<15}")
        print("-"*80)
        for stat in stats:
            print(f"{stat['Giunto']:<12} {stat['Min (rad)']:<15} {stat['Max (rad)']:<15} {stat['Media (rad)']:<15} {stat['Std Dev (rad)']:<15} {stat['Range (rad)']:<15}")
        print("="*80)
    
    def generate_task_events_table(self):
        """Genera tabella degli eventi del task"""
        events = self.data.get('task_events', [])
        
        if not events:
            print("Nessun evento registrato")
            return
        
        csv_file = self.output_dir / 'task_events.csv'
        
        with open(csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Tempo (s)', 'Tipo', 'Descrizione'])
            for event in events:
                time_s = event['timestamp'] - self.start_time
                writer.writerow([f"{time_s:.2f}", event['type'], event['description']])
        
        print(f"✓ Salvato: task_events.csv")
        
        # Stampa tabella
        print("\n" + "="*80)
        print("EVENTI DEL TASK")
        print("="*80)
        print(f"{'Tempo (s)':<15} {'Tipo':<25} {'Descrizione':<40}")
        print("-"*80)
        for event in events:
            time_s = event['timestamp'] - self.start_time
            print(f"{time_s:<15.2f} {event['type']:<25} {event['description']:<40}")
        print("="*80)
    
    def generate_summary_report(self):
        """Genera un report di sintesi"""
        report_file = self.output_dir / 'summary_report.txt'
        
        with open(report_file, 'w') as f:
            f.write("="*80 + "\n")
            f.write("TASK2 - PICK & PLACE - REPORT DI SIMULAZIONE\n")
            f.write("="*80 + "\n\n")
            
            f.write(f"Data/Ora avvio: {self.data['start_datetime']}\n")
            f.write(f"Data/Ora fine: {self.data['end_datetime']}\n")
            f.write(f"Durata simulazione: {self.data.get('duration_seconds', 0):.2f}s\n\n")
            
            f.write("DATI RACCOLTI:\n")
            f.write(f"  - Campioni arm_trajectory: {len(self.data['arm_trajectory'])}\n")
            f.write(f"  - Campioni torso_trajectory: {len(self.data['torso_trajectory'])}\n")
            f.write(f"  - Marker ArUco tracciati: {len(self.data['marker_poses'])}\n")
            f.write(f"  - Eventi registrati: {len(self.data.get('task_events', []))}\n\n")
            
            f.write("FILE GENERATI:\n")
            f.write("  - joint_trajectories.png - Grafico traiettorie giunti\n")
            f.write("  - joint_velocities.png - Grafico velocità giunti\n")
            f.write("  - marker_positions.png - Grafico posizioni marker\n")
            f.write("  - statistics_joints.csv - Statistiche giunti\n")
            f.write("  - task_events.csv - Log eventi\n")
            f.write("="*80 + "\n")
        
        print(f"✓ Salvato: summary_report.txt")
    
    def run_all_analysis(self):
        """Esegui tutta l'analisi"""
        print("\n" + "="*80)
        print("ANALISI DATI TASK2 - Pick & Place")
        print("="*80 + "\n")
        
        self.plot_joint_trajectories()
        self.plot_joint_velocities()
        self.plot_marker_positions()
        self.generate_statistics_table()
        self.generate_task_events_table()
        self.generate_summary_report()
        
        print("\n" + "="*80)
        print(f"Analisi completata. Output in: {self.output_dir}")
        print("="*80 + "\n")


def main():
    import sys
    
    if len(sys.argv) < 2:
        print("Uso: python3 analysis.py <path_to_json_file>")
        print("\nEsempio:")
        print("  python3 analysis.py /tmp/task2_logs/task2_data_20260528_153000.json")
        sys.exit(1)
    
    json_file = sys.argv[1]
    
    if not Path(json_file).exists():
        print(f"Errore: File non trovato: {json_file}")
        sys.exit(1)
    
    analyzer = Task2Analysis(json_file)
    analyzer.run_all_analysis()


if __name__ == '__main__':
    main()
