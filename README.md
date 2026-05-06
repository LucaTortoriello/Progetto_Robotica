# Progetto di Robotica - TIAGo Pick & Place

**Autori**: Erika Oliveri, Luca Tortoriello

---

## Overview del Progetto

Sviluppo di un framework di simulazione che consente al robot **TIAGo** di eseguire attività di **pick and place**, sfruttando:
- Algoritmi avanzati basati sull'intelligenza artificiale
- Modellazione cinematica e del movimento
- Controllo robotico
- Percezione visiva con ArUco marker

---

## Obiettivi Principali

### Task 1: Modellazione e Localizzazione
- Modellazione del robot TIAGo in Gazebo
- Rilevamento di oggetti tramite marker ArUco
- Trasformazioni tra frame di riferimento
- Localizzazione di oggetti nello spazio 3D

### Task 2: Pianificazione del Movimento
- Pianificazione del movimento del robot
- Inversione cinematica
- Controllo del movimento
- Esecuzione di compiti di manipolazione

---

## Stack Tecnologico

### Framework e Simulatori
- **ROS 2** - Robot Operating System (versione Humble)
- **Gazebo** - Simulatore fisico 3D
- **Rviz** - Tool di visualizzazione

### Librerie e Strumenti
- **OpenCV** - Elaborazione immagini e rilevamento ArUco
- **TF2 ROS** - Gestione trasformazioni tra frame
- **Python 3** - Linguaggio di programmazione principale

### Architettura ROS 2
- Nodi (Nodes)
- Topic
- Services
- Actions
- Launch Files

---

## Structure del Progetto

```
Progetto_Robotica/
├── README.md                 # Questo file
├── ComandiUtili.txt         # Comandi ROS 2 organizzati per ambiente
├── ARUCO/                   # Package per rilevamento ArUco
│   └── single_aruco_detection.py
├── src/                     # Sorgenti dei package ROS 2
│   ├── tiago_simulation/   # Task 1: Simulazione TIAGo
│   └── tiago_manipulation/ # Task 2: Manipolazione
└── docs/                    # Documentazione aggiuntiva
```

---

## Requisiti Preliminari

### Installazione di ROS 2
Seguire le indicazioni ufficiali per installare ROS 2 Humble:
https://docs.ros.org/en/humble/Installation.html

### Configurazione dell'Ambiente
```bash
# Setup iniziale
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build del workspace
colcon build

# Source dell'ambiente
source install/setup.bash
```

### Package TIAGo
Installare i package necessari per la simulazione del robot TIAGo:
```bash
sudo apt install ros-humble-tiago-*
```

---

## Comandi Utili

Per una lista completa di comandi ROS 2 organizzati per ambiente, consultare:
 **[ComandiUtili.txt](ComandiUtili.txt)**

### Quick Start

**Avviare la simulazione del TIAGo**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True
```

**Avviare Rviz per la visualizzazione**
```bash
ros2 launch tiago_viz tiago_viz_gazebo.launch.py
```

**Rilevare marker ArUco**
```bash
cd ~/ros2_ws/src/tiago_simulation
python3 single_aruco_detection.py
```

### Screencast

Il video dimostrativo è disponibile nella cartella [video](video).

---

## Task 1: Modellazione e Localizzazione

### Obiettivi Specifici
1. Avviare la simulazione di TIAGo in Gazebo
2. Movimentare la testa del robot per scansionare l'ambiente
3. Rilevare marker ArUco tramite camera RGB-D
4. Stimare la posa dei marker rispetto alla camera
5. Effettuare trasformazioni di coordinate
6. Visualizzare i risultati in Rviz

### Step Implementativi

#### 1. Lancio della Simulazione
```bash
ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True
```

#### 2. Controllo della Testa del Robot
- **Action**: `/head_controller/follow_joint_trajectory`
- **Giunti**: 
  - `head_1_joint` (rotazione sinistra/destra)
  - `head_2_joint` (rotazione su/giù)

#### 3. Acquisizione Immagini
- **Topic immagini**: `/head_front_camera/rgb/image_raw`
- **Topic info camera**: `/head_front_camera/rgb/camera_info`
- **Tipo messaggio**: `sensor_msgs/msg/Image` e `sensor_msgs/msg/CameraInfo`

#### 4. Rilevamento marker ArUco
Utilizzare OpenCV per:
- Rilevare i marker ArUco nell'immagine
- Stimare la posa (posizione e orientamento)
- Estrarre le coordinate 3D

#### 5. Trasformazioni di Coordinate
Utilizzare TF2 per trasformare le coordinate dei marker dal frame della camera al frame della base del robot:
- Frame camera: `head_front_camera_rgb_optical_frame`
- Frame base: `base_footprint`

#### 6. Visualizzazione
- Pubblicare i risultati come marker Rviz
- Visualizzare i frame di riferimento
- Mostrare le coordinate nel terminale

---

## Task 2: Pianificazione del Movimento

### Obiettivi Specifici
1. Implementare algoritmi di pianificazione del movimento
2. Calcolare l'inversione cinematica
3. Controllare il movimento del robot
4. Eseguire le azioni di pick and place

### Librerie Consigliate
- **MoveIt 2** - Pianificazione e controllo del movimento
- **Pinocchio** - Libreria per robotica (cinematica forward/inverse)
- **PyBullet** - Simulazione fisica

---

## Modalità di Verifica

### Consegna
- Consegna dei package ROS 2 creati
- Codice sorgente commentato e documentato
- File launch per riprodurre i risultati

### Presentazione Finale
- Report a fine corso
- Presentazione dei risultati
- Domande individuali da parte dei docenti

---

## Risorse Utili

### Documentazione Ufficiale
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Documentation](http://gazebosim.org/)
- [OpenCV Documentation](https://docs.opencv.org/)
- [ArUco Detection Guide](https://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html)

### Tutorial ROS 2
- [TF2 Transformations](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)
- [Image Transport](https://docs.ros.org/en/humble/p/image_common/)
- [Actions in ROS 2](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Writing-an-Action-Server-Client.html)

---

## Note Importanti

-  Prima di eseguire qualsiasi comando ROS 2, sempre fare il `source` del setup.bash
-  Verificare che Gazebo sia correttamente installato e configurato
-  I marker ArUco devono essere visibili nella camera del robot
-  Assicurarsi di avere abbastanza risorse di sistema per eseguire la simulazione

---

**Ultimo aggiornamento**: Marzo 2026  
**Status**: In Development
