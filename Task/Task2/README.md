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
2. Aggiungere l'entry point in `setup.py`:

```python
'pick_and_place_task2 = task2_pkg.pick_and_place_task2:main',
```

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
  aruco_A:=1 aruco_B:=2 aruco_C:=3 aruco_D:=4 \
  urdf_path:=/home/<utente>/tiago_public_ws/src/my_robot_description/urdf/tiago_robot.urdf \
  run_unfold_arm:=true
```

## Note
- il nodo usa `unfold_arm` per ottenere una configurazione comoda; se la motion non e' disponibile, usa la configurazione corrente come fallback
- il movimento tra i passaggi e' calcolato con l'IK di RoboticsToolbox tramite `ik_NR`
- il gripper si apre e chiude automaticamente, quindi non serve un comando manuale per ogni passo
- dopo ogni rilascio il robot torna alla configurazione comoda prima di passare all'oggetto successivo
- se l'IK fallisce, il nodo entra nello stato `ERROR` e ferma il task

## Verifiche da fare in laboratorio lunedi'

1. Verificare gli ID reali dei marker A, B, C, D:

```bash
ros2 topic list | grep aruco_base
ros2 topic echo /aruco_base_<ID> --once
```

Nel world della repo gli oggetti sembrano usare i marker 26 e 63, mentre i marker liberi sono 4 e 3, ma questa associazione va confermata guardando Gazebo/RViz.

Quando gli ID sono confermati, rilanciare Task2 passando i valori corretti, per esempio:

```bash
ros2 launch my_bringup launch_task2.py aruco_A:=26 aruco_B:=63 aruco_C:=4 aruco_D:=3
```

Se C e D risultano invertiti in RViz/Gazebo:

```bash
ros2 launch my_bringup launch_task2.py aruco_A:=26 aruco_B:=63 aruco_C:=3 aruco_D:=4
```

2. Confermare rapidamente i controller usati dal nodo:

```bash
ros2 topic list | grep joint_trajectory
ros2 action list -t | grep follow_joint_trajectory
```

Il nodo pubblica su:
- `/arm_controller/joint_trajectory`
- `/torso_controller/joint_trajectory`
- `/gripper_controller/joint_trajectory`

Questa scelta deriva dal codice di esercitazione `pick_place_cola.py`, dove braccio, torso e gripper sono comandati tramite questi topic. Il controllo in laboratorio serve solo a confermare che la macchina sia avviata con gli stessi controller.

3. Verificare che i service di attach/detach siano disponibili:

```bash
ros2 service list | grep attach
ros2 service list | grep detach
```

4. Confermare che la motion automatica sia disponibile:

```bash
ros2 run play_motion2 run_motion unfold_arm
```

Questa motion e' gia' riportata nei README/appunti delle esercitazioni; il controllo serve solo a verificare che il setup della macchina di laboratorio sia stato caricato correttamente.

5. Confermare che l'URDF caricato da RoboticsToolbox sia quello usato nelle esercitazioni:

```bash
ros2 launch my_bringup launch_task2.py urdf_path:=/home/<utente>/tiago_public_ws/src/my_robot_description/urdf/tiago_robot.urdf
```

Il file URDF di riferimento in repo contiene gli 8 giunti usati dal nodo: `torso_lift_joint` e `arm_1_joint` ... `arm_7_joint`. La stessa struttura e' gia' usata da `pick_place_cola.py`.
