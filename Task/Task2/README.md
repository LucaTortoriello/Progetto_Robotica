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

