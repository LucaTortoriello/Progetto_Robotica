# Task 2 - Pick & Place

Questo README descrive il nodo e il launch file di Task2, che estende Task1.

## Cosa fa
- usa il nodo ArUco di Task1 per leggere i topic `/aruco_base_<id>`
- legge le pose dei marker A, B, C, D
- esegue la sequenza:
  - Coca-Cola: A -> C
  - Pringles: B -> D
- esegue la stessa logica a stati del nodo di riferimento `pick_place_cola.py`
- visualizza la posa ArUco in RViz tramite il nodo di Task1
- utilizza attach/detach per simulare il pick & place in Gazebo

## File creati
- `nodi/pick_and_place_task2.py`
- `launch/launch_task2.py`

## Come usare
1. Copiare il nodo `pick_and_place_task2.py` in un package ROS2 esistente (ad esempio `task2_pkg/src/task2_pkg/`)
2. Aggiungere l'entry point in `setup.py`:

```python
'pick_and_place_task2 = task2_pkg.pick_and_place_task2:main',
```

3. Copiare il launch file in un package di launch valido (ad esempio `my_bringup/launch/launch_task2.py`)
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

## Parametri utili
- `aruco_A`, `aruco_B`, `aruco_C`, `aruco_D`: ID dei marker ArUco corrispondenti ad A, B, C, D
- `urdf_path`: percorso del file URDF caricato da RoboticsToolbox

## Note
- il nodo cerca di usare la stessa configurazione iniziale come "configurazione comoda"
- il movimento tra i passaggi è calcolato con l'IK di RoboticsToolbox tramite `ik_NR`
- il gripper si apre e chiude automaticamente, quindi non serve un comando manuale per ogni passo
