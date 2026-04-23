"""
Script per convertire i parametri DH (Denavit-Hartenberg), comunemente usati in robotica per descrivere la cinematica dei manipolatori, in trasformazioni utilizzabili in URDF (xyz e rpy), che servono a descrivere i robot in simulazioni come ROS (Robot Operating System)
"""

import numpy as np
from math import atan2, pi
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R


def dh_to_transformation_matrix(d, theta, a, alpha):
    """
    Converte DH in una matrice di trasformazione omogenea.
    Parameters:
    d (float): Joint distance (translation along previous z)
    theta (float): Joint angle (rotation around previous z) in radians
    a (float): Link length (translation along current x)
    alpha (float): Link twist (rotation around current x) in radians
    
    Returns:
    numpy.ndarray: 4x4 homogeneous transformation matrix
    """
    
    # Creazione della matrice DH standard: trasformazione da un frame giunto al successivo

    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    cos_alpha = np.cos(alpha)
    sin_alpha = np.sin(alpha)
    
    T = np.array([
        [cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha, a * cos_theta],
        [sin_theta, cos_theta * cos_alpha, -cos_theta * sin_alpha, a * sin_theta],
        [0, sin_alpha, cos_alpha, d],
        [0, 0, 0, 1]
    ])
    
    return T

def rotation_matrix_to_rpy(R_mat):
    """Converte la matrice di rotazione 3x3 in angoli roll-pith-yaw (utili per URDF standard)
    Calcola il problema inverso di RPY: calcola gli angoli dalla matrice di rotazione usando le funzioni inverse trigonometriche
    
    Parameters:
    R (numpy.ndarray): 3x3 rotation matrix
    
    Returns:
    tuple: (roll, pitch, yaw) in radians"""
    
    rot = R.from_matrix(R_mat)
    roll, pitch, yaw = rot.as_euler('xyz', degrees=False)
    return (roll, pitch, yaw)

def transformation_matrix_to_urdf(T):
    """
    Converte la matrice di trasformazione omogena in xyz e rpy, da usare in URDF
    
    Parameters:
    T (numpy.ndarray): 4x4 homogeneous transformation matrix
    
    Returns:
    tuple: ((x, y, z), (roll, pitch, yaw))
    """
    # Extract translation (xyz)
    xyz = (T[0, 3], T[1, 3], T[2, 3])
    
    # Extract rotation matrix
    R = T[0:3, 0:3]
    
    # Convert rotation matrix to rpy
    rpy = rotation_matrix_to_rpy(R)
    
    return (xyz, rpy)



def format_urdf_transform(xyz, rpy):
    """
    Format the URDF transform values as strings for URDF file.
    
    Parameters:
    xyz (tuple): (x, y, z) translation values
    rpy (tuple): (roll, pitch, yaw) rotation values in radians
    
    Returns:
    tuple: (xyz_str, rpy_str) formatted strings
    """
    xyz_str = f"{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f}"
    rpy_str = f"{rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}"
    
    return (xyz_str, rpy_str)


def plot_manipulator(dh_params):
    """
    Crea un plot 3D statico del braccio robotico dato un insieme di parametri DH. Serve a verificare visivamente la geometria e la struttura cinematica.
    
    Parameters:
    dh_params (list): Lista di tuple (d, theta, a, alpha) con alpha in gradi
    """
    positions = [np.zeros(3)]  # Origine del robot (base)
    T_current = np.eye(4) # Inizializza la matrice di trasformazione cumulativa T_current.

    
    for dh in dh_params:
        d, theta, a, alpha = dh
        T_i = dh_to_transformation_matrix(d, theta, a, alpha) # calcola la matrice di trasformazione omogenea per il giunto corrente
        T_current = np.dot(T_current, T_i) # Moltiplica la trasformazione corrente con quella precedente: ottenere la posizione del giunto nel sistema di riferimento della base
        position = T_current[:3, 3] # estrae la posizione del giunto dal vettore di traslazione
        positions.append(position)

    # Estrai coordinate x, y, z per ogni giunto
    xs = [p[0] for p in positions]
    ys = [p[1] for p in positions]
    zs = [p[2] for p in positions]

    # Plot 3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(xs, ys, zs, '-o', linewidth=3, markersize=5)
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.set_title('Struttura del manipolatore (DH)')
    plt.show()

def animate_robot_3d(dh_params_base, joint_angle_ranges, n_frames=50):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    total_length = sum([a for _, _, a, _ in dh_params_base]) + sum([d for d, _, _, _ in dh_params_base]) # Calcola una stima della lunghezza massima del robot 
    ax.set_xlim([-total_length, total_length])
    ax.set_ylim([-total_length, total_length])
    ax.set_zlim([0, total_length])
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")

    # Colori per i segmenti
    colors = ['red', 'green', 'blue', 'orange', 'purple', 'cyan', 'magenta']

    # Interpolazione angoli per ogni giunto
    # Interpolazione lineare degli angoli tra theta_min e theta_max su n_frames
    theta_frames = []
    for (theta_min, theta_max) in joint_angle_ranges:
        theta_frames.append(np.linspace(theta_min, theta_max, n_frames))

    for frame_idx in range(n_frames):
        dh_current = []
        
        # Crea i parametri DH aggiornati per il frame corrente, sostituendo theta con il valore interpolato.
        for i, (d, theta, a, alpha) in enumerate(dh_params_base):
            theta_current = theta_frames[i][frame_idx]
            dh_current.append((d, theta_current, a, alpha))
        
        # Inizializza la trasformazione totale e la lista delle posizioni partendo dall’origine
        T_current = np.eye(4)
        positions = [T_current[:3, 3]]
        
        # Calcola la trasformazione cumulativa per ogni giunto: aggiorna la posizione finale del giunto.
        for d, theta, a, alpha in dh_current:
            T_next = dh_to_transformation_matrix(d, theta, a, alpha)
            T_current = T_current @ T_next
            positions.append(T_current[:3, 3])
            
        print(f"Frame {frame_idx+1} - T end effector:\n{T_current}\n")
        
        ax.cla()
        ax.set_xlim([-total_length, total_length])
        ax.set_ylim([-total_length, total_length])
        ax.set_zlim([0, total_length])
        ax.set_title(f"Frame {frame_idx+1}/{n_frames}")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")

        # Disegna segmenti con colori diversi e label dei giunti: link = linee tra posizioni consecutive
        for i in range(len(positions) - 1):
            x_seg = [positions[i][0], positions[i + 1][0]]
            y_seg = [positions[i][1], positions[i + 1][1]]
            z_seg = [positions[i][2], positions[i + 1][2]]
            ax.plot(x_seg, y_seg, z_seg, '-o', color=colors[i % len(colors)], linewidth=3, markersize=6)

            # Label del giunto
            label_pos = positions[i + 1]
            ax.text(label_pos[0], label_pos[1], label_pos[2], f"J{i+1}", fontsize=9, color=colors[i % len(colors)])

        plt.pause(0.1) # Pausa tra i frame, per dare il tempo al grafico di aggiornarsi (0.1 secondi ≈ 10 fps)

    plt.show()



def main():
    # Example DH parameters from the user's input
    # Format: (d, theta, a, alpha) 
    dh_params = [
        (0.08, 0, 0, 90 * (np.pi /180)),
        (0, 0, 0, -90 * (np.pi /180)),
        (0.17, 0, 0, -90 * (np.pi /180)),
        (0, 0, 0, 90 * (np.pi /180)),
        (0.25, 0, 0, 90 *(np.pi /180)),
        (0, 0, 0, -90 * (np.pi /180)),
        (0.04, 0, 0, 0)
    ]
    
    print("Converting DH parameters to URDF transformations...\n")
    print("DH Parameters (d, theta, a, alpha):")
    for i, dh in enumerate(dh_params):
        print(f"Joint {i+1}: {dh}")
    
    print("\nResulting URDF Transformations:")
    
    # Initialize the current transformation matrix
    T_current = np.eye(4)
    
    for i, dh in enumerate(dh_params):
        d, theta, a, alpha = dh
        
        # Calculate the transformation from this frame to the next
        T_i = dh_to_transformation_matrix(d, theta, a, alpha)
        
        # Print the individual transformation
        xyz, rpy = transformation_matrix_to_urdf(T_i)
        xyz_str, rpy_str = format_urdf_transform(xyz, rpy)
        
        print(f"\nJoint {i+1}:")
        print(f"  DH Parameters: d={d}, theta={theta}, a={a}, alpha={alpha}°")
        print(f"  URDF Joint Origin: xyz=\"{xyz_str}\" rpy=\"{rpy_str}\"")
        
        # Update the current transformation to get the cumulative effect
        T_current = np.dot(T_current, T_i)
    print(f"\nCumulative Transformation from Base to End-Effector:\n{T_current}") 
        
    plot_manipulator(dh_params)
    joint_angle_ranges = [(0, np.pi/2) for _ in dh_params]
    joint_angle_ranges = np.array([
    [-150 * (np.pi / 180), 114 * (np.pi / 180)],
    [-67 * (np.pi / 180), 109 * (np.pi / 180)],
    [-150 * (np.pi / 180), 41 * (np.pi / 180)],
    [-92 * (np.pi / 180), 110 * (np.pi / 180)],
    [-150 * (np.pi / 180), 150 * (np.pi / 180)],
    [-92 * (np.pi / 180), 113 * (np.pi / 180)],
    [-150 * (np.pi / 180), 150 * (np.pi / 180)]
]) # limiti di giunto

    animate_robot_3d(dh_params, joint_angle_ranges, n_frames=100)


if __name__ == "__main__":
    main()
