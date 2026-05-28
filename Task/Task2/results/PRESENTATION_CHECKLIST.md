# Checklist Presentazione - Task2 Pick & Place

Questo documento aiuta a preparare la presentazione seguendo i suggerimenti forniti.

## ☐ 1. Schema/Diagramma di funzionamento

Cosa includere:
- [ ] Nodi ROS2 (pick_and_place_task2, aruco_detector, move_head)
- [ ] Topic utilizzati (`/arm_controller/joint_trajectory`, `/aruco_base_*`, `/gripper_controller/joint_trajectory`)
- [ ] Service/Action utilizzati (`/attach`, `/detach`, `/move_head`)
- [ ] Flusso dati tra nodi (frecce che mostrano comunicazione)
- [ ] Sequenza di stati macchina (WAIT_PICK → LOWER_PICK → ... → RETURN_SAFE)

**Output:**
- [ ] Diagramma in formato immagine (PNG o PDF)
- [ ] Posizionare in: `Task/Task2/diagrams/` (crea la cartella)

**Suggerimento:** Usa draw.io, Lucidchart, o Graphviz per creare il diagramma.

---

## ☐ 2. Descrizione metodi e tool ROS2

Cosa documentare:
- [ ] **Inverse Kinematics (IK)**: usato `ERobot.ik_NR()` da Robotics Toolbox
  - Come viene calcolata la configurazione da una pose desiderata
  - Seed iniziale usato
  - Pinv e NR (Newton-Raphson) per convergenza
  
- [ ] **ArUco Detection**: marker-based positioning
  - Topic `/aruco_base_*` fornisce pose di ogni marker
  - Offsets applicati (dx, dy, dz) per approach vs lowering
  
- [ ] **Attach/Detach**: simulazione del grip in Gazebo
  - Service `/attach` e `/detach` usati con physics
  - Link name e model name specifici
  
- [ ] **Joint Trajectory Controller**: esecuzione dei movimenti
  - Pubblicazione su `/arm_controller/joint_trajectory`
  - `/torso_controller/joint_trajectory` per il torso
  
- [ ] **Macchina a Stati**: orchestrazione della sequenza
  - 8 stati per oggetto (WAIT_PICK, LOWER_PICK, ATTACH, LIFT_AFTER_PICK, MOVE_TO_PLACE, LOWER_PLACE, DETACH, RETURN_SAFE)
  - Timer (0.5s) che esegue il loop principale

**Output:**
- [ ] Sezione nel README.md principale
- [ ] O file separato: `Task/Task2/METODOLOGIA.md`

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

✓ **Questo è quello che abbiamo appena preparato!**

Cosa generare e includere:
- [ ] Esegui il data_logger durante una simulazione completa
- [ ] Genera l'analisi con `analysis.py`
- [ ] Includi nella presentazione:

### Grafici (come immagini PNG):
- [ ] **joint_trajectories.png** - Mostra come si muovono i giunti nel tempo
- [ ] **joint_velocities.png** - Mostra le velocità durante il movimento
- [ ] **marker_positions.png** - Mostra il tracking dei marker ArUco

### Tabelle (esporta da CSV):
- [ ] **Statistiche giunti** (Min, Max, Media, Std Dev)
  - Copia i dati da `statistics_joints.csv` in una slide
  - Formatta con tabella professionale

- [ ] **Timeline eventi** (quando avviene ogni evento)
  - Copia da `task_events.csv`
  - Mostra i tempi di ogni fase (pick, place, attach, detach)

**Output:**
- [ ] Cartella `Task/Task2/results/analysis_output/` con tutti i file
- [ ] Esporta i grafici come PNG ad alta risoluzione
- [ ] Converti le tabelle CSV → PowerPoint/Impress

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
├── results/                            # ← NUOVA CARTELLA
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
python3 src/my_robot_description/Task/Task2/results/data_logger.py

# Terminale 3: Task2
ros2 launch my_bringup launch_task2.py

# 2. Quando finisce, analizza (Terminale 2)
python3 src/my_robot_description/Task/Task2/results/utils.py analyze

# 3. Raccogli i file
# - Copia i PNG da /tmp/task2_logs/analysis_output/ → Task2/results/
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
