"""
7DoF Robot Kinematics and Trajectory Planning

This script implements:
1. Forward kinematics for a 7DoF robot using DH parameters
2. Geometric Jacobian calculation
3. Planar trajectory planning in Cartesian space
4. 3D visualization of the robot following the trajectory
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import time

def dh_transformation_matrix(d, theta, a, alpha):
    """
    Calculate the homogeneous transformation matrix from DH parameters.
    
    Parameters:
    d (float): Joint distance (translation along previous z)
    theta (float): Joint angle (rotation around previous z) in radians
    a (float): Link length (translation along current x)
    alpha (float): Link twist (rotation around current x) in radians
    
    Returns:
    numpy.ndarray: 4x4 homogeneous transformation matrix
    """
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

def forward_kinematics(dh_params, joint_angles):
    """
    Calculate forward kinematics for a series of joints defined by DH parameters.
    
    Parameters:
    dh_params (list): List of tuples (d, a, alpha) - constant parameters
    joint_angles (list): List of joint angles (theta) for each joint
    
    Returns:
    list: List of transformation matrices for each joint relative to base
    numpy.ndarray: 4x4 transformation matrix for the end-effector
    list: List of (x, y, z) positions for each joint
    """
    transforms = []
    positions = []
    
    # Base position
    T_current = np.eye(4)
    positions.append(T_current[0:3, 3].copy())
    
    for i, (d, _, a, alpha) in enumerate(dh_params):
        # Calculate the transformation matrix with the current joint angle
        T_i = dh_transformation_matrix(d, joint_angles[i], a, alpha)
        
        # Update the current transformation matrix
        T_current = np.dot(T_current, T_i)
        
        # Store the transformation and position
        transforms.append(T_current.copy())
        positions.append(T_current[0:3, 3].copy())
    
    # The last transformation matrix is for the end-effector
    end_effector_transform = transforms[-1]
    
    return transforms, end_effector_transform, positions

def calculate_geometric_jacobian(dh_params, joint_angles):
    """
    Calculate the geometric Jacobian for a robot at given joint angles.
    
    Parameters:
    dh_params (list): List of tuples (d, a, alpha) - constant parameters
    joint_angles (list): List of joint angles (theta) for each joint
    
    Returns:
    numpy.ndarray: 6xn Jacobian matrix where n is the number of joints
    """

    n = len(joint_angles)  # Numero di giunti

    T_matrices = []  # Trasformazioni cumulative
    T = np.eye(4)    # Trasformazione iniziale (identità)

    # Costruzione delle trasformazioni T0i
    for i in range(n):
        d, theta, a, alpha = dh_params[i]
        theta = joint_angles[i]
        Ti = dh_transformation_matrix(d, theta, a, alpha)
        T = np.dot(T, Ti)  # Aggiorna trasformazione cumulativa
        T_matrices.append(T)

    # Estrai posizioni e assi z
    z_axes = [np.array([0, 0, 1])]  # z0
    positions = [np.array([0, 0, 0])]  # p0

    for T in T_matrices:
        z_axes.append(T[:3, 2])  # asse z
        positions.append(T[:3, 3])  # posizione

    # Posizione end-effector
    p_e = positions[-1]

    # Costruzione Jacobiano
    J = np.zeros((3, n))

    for i in range(n):
        J[:, i] = np.cross(z_axes[i], p_e - positions[i])

    return J  # Jacobiano 3x7

def inverse_kinematics_jacobian(dh_params, joint_angles, target_position, target_velocity, joint_limits, K=1.0, max_iterations=1000, tol=1e-2):
    """
    Solve inverse kinematics using the Jacobian pseudo-inverse method with a proportional gain.
    Calculates the joint velocity using the Jacobian pseudo-inverse for the desired end-effector velocity and error correction.

    Parameters:
    dh_params (list): List of tuples (d, a, alpha) for DH parameters
    joint_angles (list): Initial joint angles
    target_position (numpy.ndarray): Target position [x, y, z]
    target_velocity (numpy.ndarray): Desired velocity [vx, vy, vz] of the end-effector
    joint_limits (numpy.ndarray): Joint angle limits [n, 2] where n is number of joints
    K (float): Proportional gain for controlling the position error
    max_iterations (int): Maximum number of iterations
    tol (float): Tolerance for position error

    Returns:
    numpy.ndarray: Final joint angles
    bool: Whether the solution converged
    """
    q = np.array(joint_angles)  # Inizializzazione degli angoli dei giunti
    
    for iteration in range(max_iterations):  # Iterazioni fino a max_iterations
        # Cinematica diretta per ottenere la posizione attuale dell'end-effector
        _, end_effector, _ = forward_kinematics(dh_params, q)
        current_position = end_effector[:3, 3]  # Estrazione della posizione (x, y, z) dell'end-effector
        
        # Calcolo dell'errore di posizione
        error = target_position - current_position
        error_magnitude = np.linalg.norm(error)  # Norma dell'errore
        
        # Verifica se l'errore è entro la tolleranza
        if error_magnitude < tol:
            return q, True  # Se l'errore è abbastanza piccolo, restituisce la soluzione

        # Calcolo del Jacobiano geometrico
        J = calculate_geometric_jacobian(dh_params, q)
        
        # Calcolo della pseudo-inversa del Jacobiano
        J_pinv = np.linalg.pinv(J)
        
        # Velocità desiderata dell'end-effector (vd)
        xd_dot = target_velocity 
        
        # Calcolo del termine di correzione con il guadagno K
        error_scaled = K * error
        
        # Somma della velocità desiderata e del termine di correzione
        control_signal = xd_dot + error_scaled
        
        # Calcolo degli incrementi dei giunti (dq) usando il Jacobiano pseudo-inverso
        dq = np.dot(J_pinv, control_signal)
        
        # Fattore di passo per evitare cambiamenti troppo rapidi
        step_factor = 0.5
       # dq *= step_factor
        q=q+dq*step_factor
        
        # Aggiornamento degli angoli dei giunti
        #q += dq
        
        # Applicazione dei limiti dei giunti
        for i in range(len(q)):
            q[i] = np.clip(q[i], joint_limits[i, 0], joint_limits[i, 1])
    
    # Se raggiungiamo il numero massimo di iterazioni, restituiamo la miglior soluzione trovata
    return q, False


def cartesianPlanner(pi, pf, ti, tf, t):
    """
    Plan a cubic polynomial trajectory between two points in Cartesian space.
    
    Parameters:
    pi (numpy.ndarray): Initial position [x, y, z]
    pf (numpy.ndarray): Final position [x, y, z]
    ti (float): Initial time
    tf (float): Final time
    t (float): Current time
    
    Returns:
    tuple: (pd, pdot) where
        pd: numpy.ndarray - Desired position at time t
        pdot: numpy.ndarray - Desired velocity at time t
    """
    if t < ti:
        pd = pi
        pdot = np.array([0, 0, 0])
    elif t < tf:
        A = np.array([
            [ti**3, ti**2, ti, 1],
            [tf**3, tf**2, tf, 1],
            [3*ti**2, 2*ti, 1, 0],
            [3*tf**2, 2*tf, 1, 0]
        ])
        b = np.array([[0], [np.linalg.norm(pf-pi)], [0], [0]])
        c = np.linalg.solve(A, b)
        s = c[0]*t**3 + c[1]*t**2 + c[2]*t + c[3]
        sdot = 3*c[0]*t**2 + 2*c[1]*t + c[2]
        pd = pi + (s/np.linalg.norm(pf-pi))*(pf-pi)
        pdot = (sdot/np.linalg.norm(pf-pi))*(pf-pi)
    else:
        pd = pf
        pdot = np.array([0, 0, 0])
    
    return pd, pdot

def plot_robot(ax, positions, start_point=None, end_point=None, show_trajectory=False, trajectory=None):
    """
    Plot the robot in 3D space.

    Parameters:
    ax (matplotlib.axes.Axes): 3D axes object
    positions (list): List of (x, y, z) positions for each joint
    start_point (numpy.ndarray): Starting point of the trajectory
    end_point (numpy.ndarray): Ending point of the trajectory
    show_trajectory (bool): Whether to show the trajectory
    trajectory (numpy.ndarray): Array of points along the trajectory
    """
    # Convert positions list to numpy array
    positions = np.array(positions)

    # Plot robot arm as a line connecting all joints
    robot_line, = ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'o-', linewidth=2, markersize=6, label='Robot Arm')

    # Plot coordinate at each joint
    for i, pos in enumerate(positions):
        ax.text(pos[0], pos[1], pos[2], f' {i}', fontsize=8)

    # Plot trajectory if specified
    if show_trajectory and trajectory is not None:
        traj_line, = ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], 'r--', linewidth=1.5, label='Trajectory')

    # Plot start and end points if specified
    if start_point is not None:
        start_point_plot = ax.scatter(start_point[0], start_point[1], start_point[2], color='green', s=80, marker='o', label='Start Position')
    if end_point is not None:
        end_point_plot = ax.scatter(end_point[0], end_point[1], end_point[2], color='blue', s=80, marker='X', label='End Position')

    # Add a legend
    ax.legend(loc='upper right')

def main():
    # DH parameters for the 7DoF robot (d, theta, a, alpha)
    dh_params = [
        (0.08, 0, 0, 90 * (np.pi /180)),
        (0, 0, 0, -90 * (np.pi /180)),
        (0.17, 0, 0, -90 * (np.pi /180)),
        (0, 0, 0, 90 * (np.pi /180)),
        (0.25, 0, 0, 90 *(np.pi /180)),
        (0, 0, 0, -90 * (np.pi /180)),
        (0.04, 0, 0, 0)
    ]
    
    # Joint angle limits
    joint_angle_ranges = np.array([
        [-150 * (np.pi / 180), 114 * (np.pi / 180)],
        [-67 * (np.pi / 180), 109 * (np.pi / 180)],
        [-150 * (np.pi / 180), 41 * (np.pi / 180)],
        [-92 * (np.pi / 180), 110 * (np.pi / 180)],
        [-150 * (np.pi / 180), 150 * (np.pi / 180)],
        [92 * (np.pi / 180), 113 * (np.pi / 180)],
        [-150 * (np.pi / 180), 150 * (np.pi / 180)]
    ])
    
    # Initial joint angles (in the middle of the range)
    initial_joint_angles = np.mean(joint_angle_ranges, axis=1)
    
    # initial_joint_angles = joint_angle_ranges[:, 0]  # Limiti inferiori
    final_joint_angles = joint_angle_ranges[:, 1]    # Limiti superiori
    # Calcolo cinematica diretta per la configurazione iniziale
    _, initial_ee_transform, _ = forward_kinematics(dh_params, initial_joint_angles)
    initial_ee_position = initial_ee_transform[:3, 3]

    # Calcolo cinematica diretta per la configurazione finale
    _, final_ee_transform, _ = forward_kinematics(dh_params, final_joint_angles)
    final_ee_position = final_ee_transform[:3, 3]

    start_point = initial_ee_position
    end_point = final_ee_position
    
    print("Start Point:", start_point)
    print("End Point:", end_point)

    ti = 0
    tf = 5  # Total time for trajectory
    num_points = 100
    times = np.linspace(ti, tf, num_points)
    
    # Plan the trajectory using the cartesian planner
    trajectory = []
    for t in times:
        pd, pdot = cartesianPlanner(initial_ee_position, end_point, ti, tf, t)
        trajectory.append(pd)
    
    trajectory = np.array(trajectory)
    
    # Solve inverse kinematics for each point in the trajectory
    joint_angles_trajectory = []
    current_joint_angles = initial_joint_angles.copy()

    for target_position in trajectory:
        new_joint_angles, converged = inverse_kinematics_jacobian(
            dh_params, current_joint_angles, target_position, pdot, joint_angle_ranges
        )
        if not converged:
            print(f"Warning: IK did not converge for target position {target_position}")
        joint_angles_trajectory.append(new_joint_angles)
        current_joint_angles = new_joint_angles
    
    # Set up the 3D figure for animation
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Function to update the animation
    def update(frame):
        ax.clear()
        
        # Calculate robot configuration for this frame
        joint_angles = joint_angles_trajectory[frame]
        _, _, positions = forward_kinematics(dh_params, joint_angles)
        
        # Plot the robot and trajectory
        plot_robot(ax, positions, start_point=start_point, end_point=end_point, show_trajectory=True, trajectory=trajectory)
        
        # Set axis properties
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_zlabel('Z [m]')
        ax.text2D(0.05, 0.95, f'7DoF Robot Following a Planar Trajectory\nFrame {frame+1}/{len(joint_angles_trajectory)}', transform=ax.transAxes, fontsize=10)

        # Keep a consistent view
        ax.set_xlim([-0.3, 0.3])
        ax.set_ylim([-0.3, 0.3])
        ax.set_zlim([0, 0.6])
        
        # Add a note about the current target position
        target_pos = trajectory[frame]
        ax.text2D(0.05, 0.9, f'Target: [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}]', transform=ax.transAxes)
    
    # Create the animation
    ani = FuncAnimation(fig, update, frames=len(joint_angles_trajectory), 
                         interval=100, repeat=True)
    
    # Display the animation
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
