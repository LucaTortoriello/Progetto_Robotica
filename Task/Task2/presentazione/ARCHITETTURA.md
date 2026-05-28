# Task2 - Pick & Place: Architettura e Documentazione

## 🏗️ Architettura complessiva

Task2 **estende e integra** Task1. Quando lanci Task2, automaticamente avvii:

### Nodi da Task1:
- **`aruco_detector_all_markers`** - Rileva i 4 marker ArUco (A, B, C, D)
- **`det_task1`** - Action server per il movimento della testa (`/move_head`)

### Nodi da Task2:
- **`pick_and_place_task2`** - Nodo principale che orchestr a il pick & place

### Flusso dati complessivo:

```
Gazebo (simulazione fisica)
    ↓
ArUco Markers (A, B, C, D in Gazebo)
    ↓
[aruco_detector_all_markers] (Task1)
    ↓
/aruco_base_1, /aruco_base_2, /aruco_base_3, /aruco_base_4
    ↓
[pick_and_place_task2] (Task2)
    ↓
/arm_controller/joint_trajectory → Braccio
/torso_controller/joint_trajectory → Torso
/gripper_controller/joint_trajectory → Gripper
    ↓
Gazebo (esecuzione movimenti + attach/detach)
```

---

## 📋 Contenuto della cartella `presentazione/`

Tutti i file di analisi e logging:

### Script di raccolta dati:
- **`data_logger.py`** - Nodo ROS2 che durante l'esecuzione di Task2 registra:
  - Posizioni e velocità di torso + braccio (8 giunti)
  - Pose di tutti i 4 marker ArUco
  - Timeline degli eventi del task

- **`analysis.py`** - Elabora i dati raccolti e genera:
  - Grafici PNG delle traiettorie
  - Tabelle CSV con statistiche
  - Report di sintesi

- **`utils.py`** - Utility per automatizzare il workflow

### Documentazione:
- **`README.md`** - Guida step-by-step su come usare gli script
- **`INTEGRATION_GUIDE.md`** - Opzioni di integrazione del logger
- **`PRESENTATION_CHECKLIST.md`** - Checklist completa per la presentazione

---

## 🚀 Workflow di acquisizione dati

```bash
# PREREQUISITO: Gazebo deve essere in esecuzione
ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True

# TERMINALE 2: Avvia il logger
cd ~/progetto_12_ws/src/my_robot_description/Task/Task2/presentazione
python3 data_logger.py

# TERMINALE 3: Lancia Task2 (che integra Task1)
# Questo automaticamente:
#  - Avvia aruco_detector_all_markers (Task1)
#  - Avvia det_task1 action server (Task1)
#  - Avvia pick_and_place_task2 (Task2)
cd ~/progetto_12_ws
source install/setup.bash
ros2 launch my_bringup launch_task2.py

# [La simulazione esegue il pick & place]
# [Il logger registra tutti i dati]

# TERMINALE 2: Al termine, premi Ctrl+C sul logger
# I dati vengono salvati in /tmp/task2_logs/task2_data_TIMESTAMP.json

# Genera l'analisi:
python3 analysis.py /tmp/task2_logs/task2_data_TIMESTAMP.json

# Risultati in /tmp/task2_logs/analysis_output/
```

---

## 📊 Dati registrati dal Logger

Durante l'esecuzione di Task2, il logger raccoglie:

### 1. **Traiettorie dei giunti** (ogni 0.5s dal controller)
```
Torso: 1 giunto (lift)
Braccio: 7 giunti (arm_1..arm_7)
```
- Posizioni reali e desiderate
- Velocità
- Sforzi (torques)

### 2. **Pose dei marker ArUco** (da `aruco_detector_all_markers` - Task1)
```
Marker A (ID 2): Coca-Cola pick location
Marker B (ID 1): Pringles pick location
Marker C (ID 3): Coca-Cola place location
Marker D (ID 4): Pringles place location
```
- Posizione X, Y, Z
- Orientamento quaternione

### 3. **Timeline degli eventi**
```
WAIT_PICK (apro gripper, approach)
LOWER_PICK (scendo verso oggetto)
ATTACH (Gazebo attach)
LIFT_AFTER_PICK (sollevatevi)
MOVE_TO_PLACE (vado al marker di place)
LOWER_PLACE (scendo verso pavimento)
DETACH (Gazebo detach)
RETURN_SAFE (torno alla posa iniziale)
[REPEAT per secondo oggetto]
```

---

## 📈 Output dell'Analisi

Dopo aver eseguito `analysis.py`, ottieni:

### Grafici PNG:
1. **`joint_trajectories.png`** - 8 subplot dei giunti nel tempo
2. **`joint_velocities.png`** - Velocità di ogni giunto
3. **`marker_positions.png`** - Traiettorie dei 4 marker

### Tabelle CSV:
1. **`statistics_joints.csv`** - Min, Max, Media, StdDev per giunto
2. **`task_events.csv`** - Timeline con timestamp di ogni fase

### Report:
- **`summary_report.txt`** - Resoconto generale

---

## 🔗 Dipendenze: Task1 ↔ Task2

### Task1 fornisce a Task2:
- ✅ **ArUco Detection**: Topic `/aruco_base_*` con pose dei marker
- ✅ **Head Control**: Action `/move_head` per controllare la telecamera
- ✅ **Infrastruttura ROS2**: Setup di Gazebo e controller

### Task2 usa Task1:
- ✅ **Marker poses** per orientamento del robot
- ✅ **Action server** per scannerizzare con la testa prima del pick

### Launch file di Task2 (`launch_task2.py`):
```python
# Avvia PRIMA i nodi di Task1
ld.add_action(aruco_detector_node)  # Task1
ld.add_action(move_head_server)      # Task1

# POI avvia Task2
ld.add_action(pick_place_node)       # Task2
```

---

## ✅ Verifiche prima del lancio

Assicurati che:

1. ✓ Gazebo è in esecuzione
2. ✓ I controller ROS2 sono attivi (`/arm_controller`, `/torso_controller`, `/gripper_controller`)
3. ✓ I 4 marker ArUco sono presenti in Gazebo (visibili in RVIZ)
4. ✓ Task1 è compilato e disponibile nel workspace
5. ✓ Task2 è compilato e disponibile nel workspace

Diagnostica:
```bash
# Verifica topic disponibili
ros2 topic list | grep -E "aruco|arm_controller|gripper"

# Verifica nodi in esecuzione
ros2 node list | grep -E "task|aruco|move_head"

# Verifica action server
ros2 action list
```

---

## 📂 Struttura della cartella Task2

```
Task2/
├── README.md                    # Descrizione Task2
├── RVIZ_CHECKLIST.md           # Checklist RVIZ
├── nodi/
│   └── pick_and_place_task2.py  # Nodo principale Task2
├── launch/
│   └── launch_task2.py          # Lancia Task1 + Task2
└── presentazione/
    ├── README.md                # Questo file
    ├── data_logger.py           # ROS2 node che registra dati
    ├── analysis.py              # Script per generare grafici
    ├── utils.py                 # Utility
    ├── INTEGRATION_GUIDE.md     # Come integrare il logger
    └── PRESENTATION_CHECKLIST.md # Checklist presentazione
```

---

## 🎯 Prossimi passi

1. **Esegui una simulazione completa** con il logger attivo
2. **Genera l'analisi** con `analysis.py`
3. **Verifica i grafici** per validare il comportamento
4. **Prepara la presentazione** con i risultati

Per dettagli su come usare gli script, vedi [README.md](README.md) nella cartella presentazione.

---

**Nota**: Tutta la documentazione è basata sul flusso integrato Task1+Task2. Non è possibile eseguire solo Task2 senza Task1, poiché Task2 dipende dai dati dei marker ArUco forniti da Task1.
