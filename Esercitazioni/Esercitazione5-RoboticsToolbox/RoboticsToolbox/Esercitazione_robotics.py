import numpy as np
import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteDH, ERobot


# modo 1 per caricare robot --> urdf
urdf_loc = '/home/rita/FdR_ISI2026/esercitazioni_2026/src/my_robot_description/urdf/my_robot.urdf' # caricare robot da urdf --> INSERIRE IL PROPRIO PATH!
robot = ERobot.URDF(urdf_loc)
print(robot)

# modo 2 --> parametri DH
links = [
    RevoluteDH(d=0.08, a=0, alpha=np.pi/2),
    RevoluteDH(d=0, a=0, alpha=-np.pi/2),
    RevoluteDH(d=0.17, a=0, alpha=-np.pi/2),
    RevoluteDH(d=0, a=0, alpha=np.pi/2),
    RevoluteDH(d=0.25, a=0, alpha=np.pi/2),
    RevoluteDH(d=0, a=0, alpha=-np.pi/2),
    RevoluteDH(d=0.04, a=0, alpha=0)
]
robot = DHRobot(links)
print(robot)
joint_angle_ranges = np.array([
        [-150 * (np.pi / 180), 114 * (np.pi / 180)],
        [-67 * (np.pi / 180), 109 * (np.pi / 180)],
        [-150 * (np.pi / 180), 41 * (np.pi / 180)],
        [-92 * (np.pi / 180), 110 * (np.pi / 180)],
        [-150 * (np.pi / 180), 150 * (np.pi / 180)],
        [92 * (np.pi / 180), 113 * (np.pi / 180)],
        [-150 * (np.pi / 180), 150 * (np.pi / 180)]
    ])

# Configurazione iniziale e finale dell'end-effector
q0 = np.mean(joint_angle_ranges, axis=1)
qf = joint_angle_ranges[:, 1]    # Limiti superiori
 # Pose
T0 = robot.fkine(q0)
Tf = robot.fkine(qf)

# 1. Traiettoria cartesiana tra le due pose (SE3)
N = 100
Ts = rtb.ctraj(T0, Tf, N)  # genera N pose interpolate

# 2. Inverse kinematics per ciascun frame
q_traj = []
q_curr = q0
for T in Ts:
    sol = robot.ik_NR(T, q0=q_curr, pinv=True)
    if sol is not None and len(sol) > 0:
        q_curr = sol[0]
    q_traj.append(q_curr)

q_traj = np.array(q_traj)

# 3. Visualizzazione
robot.plot(q_traj, block=True)


