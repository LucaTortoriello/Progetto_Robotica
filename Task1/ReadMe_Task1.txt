README OPERATIVO - TASK 1 (SCAN TESTA + ARUCO)
Workspace ROS 2: ~/progetto_12_ws
Package launch: my_bringup
Comando launch principale: ros2 launch my_bringup launch_task1.py

OBIETTIVO
=========
Avviare TIAGo in Gazebo, eseguire la scansione della testa con l'action server
`det_task1` e verificare la detection degli ArUco tramite i topic pubblicati.

STRUTTURA DEL TASK
==================
- `task1_pkg`: contiene i nodi Python
- `my_robot_interfaces`: contiene l'action custom `MoveHead`
- `my_bringup`: contiene il launch `launch_task1.py`

FASE 0 - PREREQUISITI
=====================
Verificare che siano presenti:
- ROS 2 Humble
- `task1_pkg`
- `my_robot_interfaces`
- `my_bringup`
- ambiente di lavoro in `~/progetto_12_ws`

FASE 1 - PREPARAZIONE WORKSPACE
===============================
Terminale 0
-----------
```bash
source /opt/ros/humble/setup.bash
cd ~/progetto_12_ws
colcon build --symlink-install
source install/setup.bash
```

Controllo utile:
```bash
ros2 pkg prefix task1_pkg
ros2 pkg prefix my_bringup
```

FASE 2 - AVVIO GAZEBO / TIAGo
=============================
Terminale 1
-----------
```bash
source /opt/ros/humble/setup.bash
source ~/progetto_12_ws/install/setup.bash
ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True
```

Cosa aspettarsi:
- si apre Gazebo
- TIAGo compare nella scena
- il terminale resta occupato con la simulazione

FASE 3 - AVVIO DEL TASK 1
=========================
Terminale 2
-----------
```bash
source /opt/ros/humble/setup.bash
source ~/progetto_12_ws/install/setup.bash
ros2 launch my_bringup launch_task1.py
```

Cosa aspettarsi:
- parte `det_task1`
- parte `aruco_detector_all_markers`
- compaiono messaggi di avvio dei nodi
- i log dei due nodi compaiono nello stesso terminale del launch, non in un terminale separato

FASE 4 - VERIFICA CHE I NODI SIANO ATTIVI
=========================================
Terminale 3
-----------
```bash
source /opt/ros/humble/setup.bash
source ~/progetto_12_ws/install/setup.bash
ros2 node list
```

Devi vedere almeno:
- il nodo action server della testa
- il nodo detector ArUco

FASE 5 - VERIFICA DELL'ACTION
=============================
Terminale 3 o 4
---------------
```bash
source /opt/ros/humble/setup.bash
source ~/progetto_12_ws/install/setup.bash
ros2 action list -t
```

Devi vedere:
- `/move_head`

Invio del goal per far muovere la testa:
```bash
ros2 action send_goal /move_head my_robot_interfaces/action/MoveHead "{min: -0.6, max: 0.6, step: 0.2}" --feedback
```

Cosa aspettarsi:
- il goal viene accettato
- la testa di TIAGo inizia a muoversi
- compare il feedback sul terminale del client

FASE 6 - VERIFICA DETECTION ARUCO
=================================
Dopo aver fatto muovere la testa, controllare i topic ArUco.

Nota utile:
- il nodo `aruco_detector_all_markers` stampa anche nel terminale gli ID rilevati
- stampa anche le pose nel frame camera e nel frame base per ogni marker
- quindi puoi verificare la detection senza usare sempre `ros2 topic list`
- se non vedi subito i log, torna nella scheda del terminale del launch e cerca le righe che iniziano con `Marker` o `ArUco rilevati`
- in alternativa puoi aumentare la finestra del terminale per evitare che i messaggi vadano fuori schermo

Terminale 3 o 4
---------------
```bash
source /opt/ros/humble/setup.bash
source ~/progetto_12_ws/install/setup.bash
ros2 topic list | grep aruco
```

Controllo rapido da terminale:
```bash
ros2 topic echo /aruco_base_1
```

oppure osserva direttamente il terminale del nodo ArUco: se il marker viene visto,
comparirà una riga del tipo `ArUco rilevati: [1, 2, 3, 4]`.

Esempio di risultato atteso:
- `/aruco_base_1`
- `/aruco_base_2`
- `/aruco_base_3`
- `/aruco_base_4`
- `/aruco_camera_1`
- `/aruco_camera_2`
- `/aruco_camera_3`
- `/aruco_camera_4`

Verifica di un singolo marker:
```bash
ros2 topic echo /aruco_base_1
```

oppure:
```bash
ros2 topic echo /aruco_camera_1
```

Se la detection funziona, arriveranno messaggi `PoseStamped`.

FASE 7 - VERIFICA VISIVA
========================
Cose da controllare:
- la testa di TIAGo oscilla da sinistra a destra
- in `rqt_graph` si vedono i nodi e le connessioni ROS
- i topic ArUco compaiono solo dopo la scansione e la visione dei marker

FASE 8 - ORDINE CONSIGLIATO DEI TERMINALI
=========================================
1. Terminale 1 -> Gazebo / TIAGo
2. Terminale 2 -> `ros2 launch my_bringup launch_task1.py`
3. Terminale 3 -> `ros2 node list`, `ros2 action list -t`, `ros2 topic list | grep aruco`
4. Terminale 4 -> `ros2 action send_goal ...` e `ros2 topic echo ...`

CHECK FINALE
============
Se tutto va bene, devi avere:
- TIAGo in Gazebo attivo
- `det_task1` in esecuzione
- `aruco_detector_all_markers` in esecuzione
- action `/move_head` disponibile
- topic `/aruco_base_*` e `/aruco_camera_*` pubblicati

NOTE UTILI
==========
- Il launch del task non avvia Gazebo: Gazebo va lanciato in un terminale separato.
- La scansione della testa parte solo dopo l'invio del goal su `/move_head`.
- Se non vedi i topic ArUco, verifica che la testa stia effettivamente muovendosi.
