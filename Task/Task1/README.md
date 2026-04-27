# Task 1 - Comandi per eseguire il progetto

Questo README contiene i comandi necessari per avviare il Task 1 e verificare il lavoro svolto.

Il Task 1 avvia:
- il nodo action server `det_task1`, che muove la testa di TIAGo per fare la scansione;
- il nodo `aruco_detector_all_markers`, che rileva tutti i marker ArUco visibili dalla camera;
- i topic `/aruco_camera_ID` e `/aruco_base_ID`, con la posa dei marker rispetto alla camera e alla base del robot.

Workspace ROS 2 usato: `~/progetto_12_ws`

Package coinvolti:
- `task1_pkg`
- `my_bringup`
- `my_robot_interfaces`

Comando principale:

```bash
ros2 launch my_bringup launch_task1.py
```

## 1. Preparazione del workspace

Aprire un terminale e lanciare:

```bash
source /opt/ros/humble/setup.bash
cd ~/progetto_12_ws
colcon build --symlink-install
source install/setup.bash
```


## 2. Avvio di Gazebo con TIAGo

Terminale 1:

```bash
source /opt/ros/humble/setup.bash
source ~/progetto_12_ws/install/setup.bash
ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True
```


## 3. Avvio dei nodi del Task 1

Terminale 2:

```bash
source /opt/ros/humble/setup.bash
source ~/progetto_12_ws/install/setup.bash
ros2 launch my_bringup launch_task1.py
```



## 4. Verifica dei nodi attivi

Terminale 3:

```bash
source /opt/ros/humble/setup.bash
source ~/progetto_12_ws/install/setup.bash
ros2 node list
```

Nella lista devono comparire:
- `/action_server_head_scan`
- `/aruco_detector`

## 5. Verifica dell'action server

Terminale 3:

```bash
ros2 action list -t
```

Deve comparire:

```text
/move_head [my_robot_interfaces/action/MoveHead]
```

Per avviare la scansione della testa:

```bash
ros2 action send_goal /move_head my_robot_interfaces/action/MoveHead "{min: -0.6, max: 0.6, step: 0.2}" --feedback
```


## 6. Verifica della detection ArUco

Dopo aver avviato la scansione, controllare i topic ArUco.

La detection puo' essere verificata in due modi:
- da terminale, tramite i log del nodo e i topic pubblicati;
- dalla finestra grafica `Aruco`, che si apre dopo l'avvio del detector e mostra i marker rilevati mentre la testa del robot si muove dopo il goal.

Terminale 3:

```bash
ros2 topic list | grep aruco
```

Esempio di topic attesi:

```text
/aruco_base_1
/aruco_base_2
/aruco_base_3
/aruco_base_4
/aruco_camera_1
/aruco_camera_2
/aruco_camera_3
/aruco_camera_4
```



## 7. Se i file devono essere copiati nel workspace

Se la consegna viene aperta partendo dal repository e non dal workspace ROS 2 gia' configurato, copiare i file nei package corretti prima della build.

Dal repository del progetto:

```bash
mkdir -p ~/progetto_12_ws/src/my_bringup/launch
cp Task/Task1/Launch/launch_task1.py ~/progetto_12_ws/src/my_bringup/launch/
cp Task/Task1/nodi/det_task1.py ~/progetto_12_ws/src/task1_pkg/task1_pkg/
cp Task/Task1/nodi/aruco_detector_all_markers.py ~/progetto_12_ws/src/task1_pkg/task1_pkg/
```

Poi verificare che in `~/progetto_12_ws/src/task1_pkg/setup.py` siano presenti questi entry point:

```python
'det_task1 = task1_pkg.det_task1:main',
'aruco_detector_all_markers = task1_pkg.aruco_detector_all_markers:main',
```

Infine ricompilare:

```bash
source /opt/ros/humble/setup.bash
cd ~/progetto_12_ws
colcon build --symlink-install
source install/setup.bash
```


