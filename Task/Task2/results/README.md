# Analisi Risultati Task2 - Pick & Place

Questa cartella contiene gli script per raccogliere e analizzare i dati della simulazione.

## File disponibili

- **`data_logger.py`** - Nodo ROS2 che registra i dati durante l'esecuzione
- **`analysis.py`** - Script Python che genera grafici e tabelle dai dati raccolti

## Come usare

### 1. Avvia il Data Logger durante la simulazione

Apri un nuovo terminale e avvia il logger **prima** di lanciare il Task2:

```bash
cd ~/progetto_12_ws
source install/setup.bash
python3 src/my_robot_description/Task/Task2/results/data_logger.py
```

**Oppure** aggiungilo direttamente al launch file di Task2.

### 2. Esegui la simulazione

Normalmente, nel terminale principale:

```bash
ros2 launch my_bringup launch_task2.py
```

Il logger registrerà:
- Posizioni e velocità di tutti i giunti
- Pose dei marker ArUco (A, B, C, D)
- Configurazioni desiderate vs reali
- Eventi importanti

### 3. Genera i grafici e le tabelle

Quando la simulazione termina, i dati vengono salvati in `/tmp/task2_logs/`:

```bash
ls /tmp/task2_logs/
```

Vedrai file come:
- `task2_data_20260528_153000.json`
- `task2_joints_20260528_153000.csv`

Ora genera l'analisi:

```bash
python3 src/my_robot_description/Task/Task2/results/analysis.py /tmp/task2_logs/task2_data_20260528_153000.json
```

**Sostituisci il timestamp con il file effettivo.**

### 4. Visualizza i risultati

I risultati vengono salvati in:

```
/tmp/task2_logs/analysis_output/
```

Contiene:
- **`joint_trajectories.png`** - Traiettorie di tutti i 8 giunti nel tempo
- **`joint_velocities.png`** - Velocità di tutti i giunti
- **`marker_positions.png`** - Posizioni X,Y,Z dei marker ArUco nel tempo
- **`statistics_joints.csv`** - Tabella con min, max, media, std dev di ogni giunto
- **`task_events.csv`** - Timeline degli eventi (pick, place, attach, detach)
- **`summary_report.txt`** - Report di sintesi

## Esempio completo

```bash
# Terminale 1 - Gazebo
ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True

# Terminale 2 - Data Logger
cd ~/progetto_12_ws
source install/setup.bash
python3 src/my_robot_description/Task/Task2/results/data_logger.py

# Terminale 3 - Task2
ros2 launch my_bringup launch_task2.py

# [Attendi che la simulazione termini]

# Terminale 2 - Analisi (dopo che il logger è stato interrotto)
python3 src/my_robot_description/Task/Task2/results/analysis.py /tmp/task2_logs/task2_data_*.json

# Apri i file PNG con un image viewer
```

## Output atteso

### Grafici generati:

1. **Traiettorie dei giunti**: mostra la posizione di ogni giunto nel tempo
   - Utile per verificare che i movimenti seguono il profilo desiderato
   - Identifica eventuali anomalie o oscillazioni

2. **Velocità dei giunti**: velocità angolare di ogni giunto
   - Mostra la dinamica del movimento
   - Identifica picchi di velocità inattesi

3. **Posizioni marker**: traiettoria X,Y,Z di ogni marker ArUco
   - Verifica che il robot si posiziona correttamente sopra i marker
   - Mostra la stabilità del tracking

### Tabelle generate:

1. **statistics_joints.csv**: statistiche per giunto
   - Min, Max, Media, Std Dev, Range
   - Utile per verificare i limiti di movimento

2. **task_events.csv**: timeline degli eventi
   - Quando ogni fase del pick & place inizia/termina
   - Identifica eventuali ritardi o fallimenti

## Note importanti

- I dati vengono salvati ogni **5 secondi** (modificabile in `data_logger.py`)
- Se Gazebo non è in real-time, i timestamp potrebbero non essere accurati
- Interrompi il logger con `Ctrl+C` per finalizzare il salvataggio
- I grafici PNG hanno risoluzione 150 DPI per stampa di qualità

## Personalizzazione

### Modificare la frequenza di salvataggio

Apri `data_logger.py` e cambia questa riga:

```python
self.create_timer(5.0, self.save_data)  # 5 secondi
```

In:

```python
self.create_timer(2.0, self.save_data)  # 2 secondi (più frequente)
```

### Aggiungere più subscriber

Se vuoi tracciare altri topic, aggiungi subscription in `data_logger.py`:

```python
self.create_subscription(
    YourMessageType,
    '/your/topic',
    self.your_callback,
    10
)
```

Poi crea il callback e registra i dati in `self.data`.

## Troubleshooting

### Q: I dati sono vuoti
**R:** Verifica che:
- Il logger è stato avviato **prima** di Task2
- I topic ROS2 sono corretti (`/arm_controller/controller_state`, etc.)
- Gazebo è in esecuzione

### Q: Errore "File not found"
**R:** Controlla il timestamp nel file JSON e copialo esattamente nel comando analysis.

### Q: I grafici non sono leggibili
**R:** Aumenta la dimensione della figura in `analysis.py`:
```python
fig, axes = plt.subplots(4, 2, figsize=(20, 14))  # Invece di (15, 12)
```

---

Buona analisi! 📊📈
