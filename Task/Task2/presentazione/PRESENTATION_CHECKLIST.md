# Checklist Presentazione - Task2 Pick & Place

Questo documento aiuta a preparare la presentazione **usando SOLO ciò che è stato spiegato a lezione**.

## ⭐ Linee guida: Rimani semplice

### ✅ INCLUDI (roba insegnata a lezione):
- FK/IK con Robotics Toolbox (Esercitazione5)
- Marker ArUco (Esercitazione1)
- Topic ROS2 (Esercitazione2)
- Machine a stati semplice
- Grafici 2D (come in Esercitazione5)
- Tabelle numeriche

### ❌ NON INCLUDERE (troppo complicato):
- Visualizzazione 3D del robot
- Animazioni
- Jacobiane dettagliate
- Cose non insegnate
- Formule matematiche complicate

## ☐ 1. Schema/Diagramma di funzionamento

**Nota**: Deve includere sia Task1 che Task2, poiché Task2 dipende da Task1.

Cosa includere:
- [ ] **Nodi Task1**:
  - `aruco_detector_all_markers` - Rileva marker A, B, C, D
  - `det_task1` - Action server per movimento testa
- [ ] **Nodi Task2**:
  - `pick_and_place_task2` - Orchestrazione pick & place
- [ ] **Topic**:
  - Da Task1: `/aruco_base_1`, `/aruco_base_2`, `/aruco_base_3`, `/aruco_base_4`
  - Da Task2: `/arm_controller/joint_trajectory`, `/torso_controller/joint_trajectory`, `/gripper_controller/joint_trajectory`
- [ ] **Service/Action**:
  - Gazebo: `/attach`, `/detach` (usati da Task2)
  - `/move_head` action (da Task1, usato opzionalmente da Task2)
- [ ] **Flusso dati**: Frecce che mostrano come Gazebo → ArUco (Task1) → Pick&Place (Task2)
- [ ] **Sequenza di stati**: WAIT_PICK → LOWER_PICK → ATTACH → ... → RETURN_SAFE

**Output:**
- [ ] Diagramma in formato immagine (PNG o PDF)
- [ ] Posizionare in: `Task/Task2/diagrams/`

**Suggerimento:** Usa draw.io, Lucidchart, o Graphviz per creare il diagramma.

---

## ☐ 2. Descrizione metodi e tool ROS2

**Nota**: Basato su ciò che è stato insegnato a lezione. Niente cose complicate, solo quello che hai imparato.

### Task1 (da Esercitazione2-Topic e Esercitazione1):
- [ ] **ArUco Detection**:
  - Come i marker ArUco vengono rilevati in Gazebo
  - Topic `/aruco_base_*` fornisce pose (x, y, z) e orientamento quaternione
  - 4 marker per il task: A (pick coca), B (pick pringles), C (place coca), D (place pringles)

### Task2 (da Esercitazione5-RoboticsToolbox e PickPlace):
- [ ] **Forward Kinematics (FK)**: `robot.fkine(q)`
  - Calcola la pose dell'end-effector data una configurazione dei giunti
  - Usato per verificare dove il robot arriva
  
- [ ] **Inverse Kinematics (IK)**: `robot.ik_NR()` (Newton-Raphson)
  - Calcola la configurazione dei giunti da una pose desiderata
  - Seed iniziale (`q0`) per aiutare la convergenza
  - `pinv=True` usa pseudo-inversa jacobiana
  - Usato per ogni target (approach pick, lower, move, approach place)
  
- [ ] **Traslazioni SE3**: `SE3.Ry()`, `SE3.Rz()` per le rotazioni
  - Orienta l'end-effector correttamente (90° per pick, 180° per place)
  - Combinate con marker pose per generare target trajectories
  
- [ ] **Offsets spaziali**:
  - Approach sopra marker: `dz = -0.1 m` (10 cm sopra)
  - Lower per pick: `dz = -0.2 m` (sotto l'oggetto)
  - `dx`, `dy` per allineamento fine
  
- [ ] **Joint Trajectory Controller**: 
  - Pubblica su `/arm_controller/joint_trajectory`
  - Controller ROS2 esegue il movimento interpolando la configurazione
  - Tempo di movimento: `duration_sec` (es. 3 secondi per abbassare)
  
- [ ] **Macchina a Stati**: 8 stati ripetuti per 2 oggetti
  - WAIT_PICK → LOWER_PICK → ATTACH → LIFT_AFTER_PICK → MOVE_TO_PLACE → LOWER_PLACE → DETACH → RETURN_SAFE

**Output:**
- [ ] Spiega i concetti di FK/IK (come nella lezione Esercitazione5)
- [ ] Diagramma dei marker (A, B, C, D)
- [ ] Timeline degli 8 stati

---

## ☐ 3. Video che mostra i compiti eseguiti

Cosa registrare:
- [ ] Video di una simulazione completa (pickup cocacola + pickup pringles)
- [ ] Camera fissa su RViz o Gazebo
- [ ] Audio/sottotitoli che spiegano ogni fase
- [ ] Durata: 1-3 minuti

**Fasi principali:**
1. Robot in posa iniziale
2. Approach sopra marker A (cocacola pick)
3. Abbassamento e chiusura gripper
4. Sollevamento e spostamento a marker C (cocacola place)
5. Abbassamento e apertura gripper
6. Ritorno e repeat per pringles

**Formato:**
- [ ] MP4 o WebM
- [ ] Risoluzione: 1920x1080 p
- [ ] Salvare in: `Task/Task2/videos/task2_demo.mp4`

**Tool per registrazione:**
- SimpleScreenRecorder (Linux)
- OBS (cross-platform)
- ffmpeg (riga di comando)

---

## ☐ 4. Risultati della simulazione (GRAFICI E TABELLE)

✓ **Questo è quello che abbiamo preparato - SEMPLICE e BASATO SU CLASSE**

Cosa generare e includere:
- [ ] Esegui il data_logger durante una simulazione completa di Task2
- [ ] Genera l'analisi con `analysis.py`
- [ ] **NON serve visualizzazione 3D** - basta i grafici 2D

### Grafici (immagini PNG - come Esercitazione5):
- [ ] **joint_trajectories.png** - Posizioni degli 8 giunti nel tempo (come il grafico di Esercitazione5)
- [ ] **joint_velocities.png** - Velocità di ciascun giunto (mostra il controllo del movimento)
- [ ] **marker_positions.png** - Posizioni X,Y,Z dei 4 marker ArUco (stazionarie se tracking corretto)

### Tabelle (semplici statistiche):
- [ ] **Statistiche giunti** (Min, Max, Media, Std Dev)
  - Mostra il range di movimento di ogni giunto
  - Verifica se i limiti sono rispettati

- [ ] **Timeline eventi** (tempo di cada fase)
  - Quando avviene ogni stato della macchina a stati
  - Tempo totale di pick & place

**Output:**
- [ ] PNG ad alta risoluzione (150 DPI)
- [ ] CSV leggibili e importabili in Excel/Calc
- [ ] Niente 3D animation o cose complicate

---

## ☐ 5. Altro contenuto rilevante

Cosa aggiungere a discrezione:

### Analisi critica (✓ Altamente consigliato):
- [ ] **Successi**: Cosa è andato bene
  - IK ha convergito rapidamente
  - Sequenza di stati eseguita correttamente
  - Marker tracking stabile

- [ ] **Limitazioni/Problemi**: Cosa potrebbe migliorare
  - Eventuali fallimenti di IK
  - Tempo totale di esecuzione
  - Precisione di pick & place
  - Collisioni o comportamenti inattesi

### Confronto con Task1:
- [ ] Come Task2 estende Task1
- [ ] Nuove funzionalità aggiunte
- [ ] Complessità aumentata

### Prospettive future:
- [ ] Miglioramenti possibili (migliore IK, visual servoing, feedback force)
- [ ] Estensione ad altri oggetti/scenario

---

## 📋 Struttura consigliata della cartella Task2

```
Task2/
├── README.md                          # Descrizione principale
├── RVIZ_CHECKLIST.md                  # (già presente)
├── nodi/
│   └── pick_and_place_task2.py         # Nodo principale
├── launch/
│   └── launch_task2.py                 # Launch file
├── presentazione/                      # ← CARTELLA PRESENTAZIONE
│   ├── data_logger.py                  # Logger ROS2
│   ├── analysis.py                     # Script di analisi
│   ├── utils.py                        # Utility
│   ├── README.md                       # Come usare
│   ├── INTEGRATION_GUIDE.md            # Come integrare
│   ├── PRESENTATION_CHECKLIST.md       # Questo file
│   └── analysis_output/                # Output dei grafici
│       ├── joint_trajectories.png
│       ├── joint_velocities.png
│       ├── marker_positions.png
│       ├── statistics_joints.csv
│       ├── task_events.csv
│       └── summary_report.txt
├── diagrams/                           # ← NUOVA CARTELLA
│   └── block_diagram.png               # Schema nodi/topic
└── videos/                             # ← NUOVA CARTELLA
    └── task2_demo.mp4                  # Video demo
```

---

## 🎯 Procedura finale (Quick Start)

```bash
# 1. Prepara i dati
cd ~/progetto_12_ws
source install/setup.bash

# Terminale 1: Gazebo
ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True

# Terminale 2: Logger
python3 src/my_robot_description/Task/Task2/presentazione/data_logger.py

# Terminale 3: Task2
ros2 launch my_bringup launch_task2.py

# 2. Quando finisce, analizza (Terminale 2)
python3 src/my_robot_description/Task/Task2/presentazione/utils.py analyze

# 3. Raccogli i file
# - Copia i PNG da /tmp/task2_logs/analysis_output/ → Task2/presentazione/
# - Esporta CSV → PowerPoint
# - Registra video (ripeti la simulazione con screenrecorder)
# - Crea diagramma

# 4. Prepara slides con:
# ✓ Diagramma + spiegazione metodi
# ✓ Video demo
# ✓ Grafici + tabelle risultati
# ✓ Analisi critica
```

---

## ✅ Checklist finale prima della presentazione

- [ ] Tutte le immagini sono in alta risoluzione (≥150 DPI)
- [ ] I grafici hanno etichette leggibili e legenda
- [ ] Le tabelle sono formattate in modo professionale
- [ ] Il video è sincronizzato e ha sottotitoli/narrazione
- [ ] Il diagramma è chiaro e rappresenta correttamente il sistema
- [ ] La presentazione non è più di 15-20 minuti
- [ ] Hai provato a spiegare i contenuti a qualcuno (dry-run)
- [ ] I file sono organizzati in cartelle logiche

---

**Buona fortuna con la presentazione! 🚀**

Per domande su come usare gli script di analysis, vedi: [README.md](README.md)
