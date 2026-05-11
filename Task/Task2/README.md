# Task 2 - Pick & Place

Questo README descrive il nodo e il launch file di Task2, che estende Task1.

## Cosa fa
- usa il nodo ArUco di Task1 per leggere i topic `/aruco_base_<id>`
- legge le pose dei marker A, B, C, D
- avvia anche l'action server del Task1 per la scansione della testa
- esegue la sequenza:
  - Coca-Cola: A -> C
  - Pringles: B -> D
- esegue la stessa logica a stati del nodo di riferimento `pick_place_cola.py`
- visualizza la posa ArUco in RViz tramite il nodo di Task1
- utilizza attach/detach per simulare il pick & place in Gazebo
- prova a portare automaticamente il braccio in configurazione comoda con `play_motion2 run_motion unfold_arm`
- interrompe la sequenza se una pianificazione IK fallisce

## File creati
- `nodi/pick_and_place_task2.py`
- `launch/launch_task2.py`

## Come usare
1. Copiare il nodo `pick_and_place_task2.py` in un package ROS2 esistente, ad esempio `task2_pkg/task2_pkg/`.

3. Copiare il launch file in un package di launch valido, ad esempio `my_bringup/launch/launch_task2.py`.
4. Compilare il workspace:

```bash
source /opt/ros/humble/setup.bash
cd ~/progetto_12_ws
colcon build --symlink-install
source install/setup.bash
```

5. Avviare Gazebo:

```bash
ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True
```

6. Avviare Task2:

```bash
ros2 launch my_bringup launch_task2.py
```

Il launch avvia:
- `det_task1` del Task1, cioe' l'action server `/move_head`
- `aruco_detector_all_markers` del Task1
- `pick_and_place_task2` del Task2

Per avviare la scansione della testa:

```bash
ros2 action send_goal /move_head my_robot_interfaces/action/MoveHead "{min: -0.6, max: 0.6, step: 0.2}" --feedback
```

## Parametri utili
- `aruco_A`, `aruco_B`, `aruco_C`, `aruco_D`: ID dei marker ArUco corrispondenti ad A, B, C, D
- `urdf_path`: percorso del file URDF caricato da RoboticsToolbox
- `run_unfold_arm`: se `true`, il nodo prova a eseguire automaticamente `ros2 run play_motion2 run_motion unfold_arm`

Esempio con parametri espliciti:

```bash
ros2 launch my_bringup launch_task2.py \
  aruco_A:=26 aruco_B:=63 aruco_C:=4 aruco_D:=3 \
  urdf_path:=/home/<utente>/progetto_12_ws/src/my_robot_description/urdf/tiago_robot.urdf \
  run_unfold_arm:=true
```

## Note
- il nodo usa `unfold_arm` per ottenere una configurazione comoda; se la motion non e' disponibile, usa la configurazione corrente come fallback
- il movimento tra i passaggi e' calcolato con l'IK di RoboticsToolbox tramite `ik_NR`
- il gripper si apre e chiude automaticamente, quindi non serve un comando manuale per ogni passo
- dopo ogni rilascio il robot torna alla configurazione comoda prima di passare all'oggetto successivo
- se l'IK fallisce, il nodo entra nello stato `ERROR` e ferma il task

## Errori da risolvere

prima di mandare il goal, mel avviare il launch appare questo errore

ubuntu@ubuntu12:~/progetto_12_ws$ ros2 launch my_bringup launch_task2.py
[INFO] [launch]: All log files can be found below /home/ubuntu/.ros/log/2026-05-11-18-20-31-560619-ubuntu12-7124
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [det_task1-1]: process started with pid [7128]
[INFO] [aruco_detector_all_markers-2]: process started with pid [7130]
[INFO] [pick_and_place_task2-3]: process started with pid [7132]
[det_task1-1] [INFO] [1778516432.611681962] [action_server_head_scan]: Action server custom pronto.
[pick_and_place_task2-3] Traceback (most recent call last):
[pick_and_place_task2-3]   File "/home/ubuntu/progetto_12_ws/install/task2_pkg/lib/task2_pkg/pick_and_place_task2", line 33, in <module>
[pick_and_place_task2-3]     sys.exit(load_entry_point('task2-pkg', 'console_scripts', 'pick_and_place_task2')())
[pick_and_place_task2-3]   File "/home/ubuntu/progetto_12_ws/build/task2_pkg/task2_pkg/pick_and_place_task2.py", line 541, in main
[pick_and_place_task2-3]     node = TiagoPickAndPlaceTask2()
[pick_and_place_task2-3]   File "/home/ubuntu/progetto_12_ws/build/task2_pkg/task2_pkg/pick_and_place_task2.py", line 152, in __init__
[pick_and_place_task2-3]     self.robot = ERobot.URDF(urdf_loc)
[pick_and_place_task2-3]   File "/home/ubuntu/.local/lib/python3.10/site-packages/roboticstoolbox/robot/Robot.py", line 306, in URDF
[pick_and_place_task2-3]     links, name, urdf_string, urdf_filepath = Robot.URDF_read(file_path)
[pick_and_place_task2-3]   File "/home/ubuntu/.local/lib/python3.10/site-packages/roboticstoolbox/robot/Robot.py", line 279, in URDF_read
[pick_and_place_task2-3]     urdf_string = open(file_path).read()
[pick_and_place_task2-3] FileNotFoundError: [Errno 2] No such file or directory: '/home/ubuntu/.local/lib/python3.10/site-packages/rtbdata/xacro/~/progetto_12_ws/src/my_robot_description/urdf/tiago_robot.urdf'
[ERROR] [pick_and_place_task2-3]: process has died [pid 7132, exit code 1, cmd '/home/ubuntu/progetto_12_ws/install/task2_pkg/lib/task2_pkg/pick_and_place_task2 --ros-args -r __node:=tiago_pick_place_task2 --params-file /tmp/launch_params_0a8v77z3 --params-file /tmp/launch_params_1_sexxil --params-file /tmp/launch_params_zxf09pbg --params-file /tmp/launch_params_twuj0p0q --params-file /tmp/launch_params_5qgcb6s2 --params-file /tmp/launch_params_hfanfimj --params-file /tmp/launch_params_bfq24wna'].

urdf_loc = '/home/rita/FdR_ISI2026/esercitazioni_2026/src/my_robot_description/urdf/tiago_robot.urdf'

path nostro 

/home/ubuntu/progetto_12_ws/src/my_robot_description/urdf/tiago_robot.urdf
