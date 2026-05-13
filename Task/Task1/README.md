# Task 1 - Comandi per eseguire il progetto
Il Task 1 avvia:
- il nodo action server `det_task1`, che muove la testa di TIAGo per fare la scansione;
- il nodo `aruco_detector_all_markers`, che rileva tutti i marker ArUco visibili dalla camera;
- il nodo action client `move_head_client_task1`, che invia automaticamente il goal di scansione con i parametri richiesti;
- i topic `/aruco_camera_ID` e `/aruco_base_ID`, con la posa dei marker rispetto alla camera e alla base del robot.

Workspace ROS 2 usato: `~/progetto_12_ws`

Comando principale:

```bash
ros2 launch my_bringup launch_task1.py
```



## 1. Avvio di Gazebo con TIAGo

Terminale 1:

```bash
source /opt/ros/humble/setup.bash
source ~/progetto_12_ws/install/setup.bash
ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True
```


## 2. Avvio dei nodi del Task 1

Terminale 2:

```bash
source /opt/ros/humble/setup.bash
source ~/progetto_12_ws/install/setup.bash
ros2 launch my_bringup launch_task1.py
```


