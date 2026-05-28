# Integrazione Data Logger nel Launch File

Se vuoi che il logger si avvii automaticamente con il Task2, aggiungi questo al file `launch_task2.py`:

## Metodo 1: Aggiungere il nodo al launch file

```python
# In launch_task2.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    # ... altri nodi ...
    
    # Aggiungi il data logger
    data_logger_node = Node(
        package='task2_pkg',  # Il tuo package
        executable='data_logger',  # Se hai creato un setup.py
        output='screen'
    )
    ld.add_action(data_logger_node)
    
    # Aggiungi il nodo di pick and place
    pick_place_node = Node(
        package='task2_pkg',
        executable='pick_and_place_task2',
        output='screen'
    )
    ld.add_action(pick_place_node)
    
    return ld
```

## Metodo 2: Lancio manuale (più semplice)

**Terminale 1 - Gazebo:**
```bash
ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True
```

**Terminale 2 - Logger:**
```bash
cd ~/progetto_12_ws/src/my_robot_description/Task/Task2/results
python3 data_logger.py
```

**Terminale 3 - Task2:**
```bash
ros2 launch my_bringup launch_task2.py
```

**Terminale 2 (quando finisce) - Analisi:**
```bash
# Interrompi il logger con Ctrl+C, poi:
cd ~/progetto_12_ws/src/my_robot_description/Task/Task2/results
python3 utils.py find
python3 analysis.py /tmp/task2_logs/task2_data_TIMESTAMP.json
```

## Metodo 3: Script bash per automatizzare

Crea un file `run_with_analysis.sh`:

```bash
#!/bin/bash

# Avvia il logger
echo "Avvio logger..."
python3 /home/ubuntu/progetto_12_ws/src/my_robot_description/Task/Task2/results/data_logger.py &
LOGGER_PID=$!

# Attendi che il logger sia pronto
sleep 2

# Avvia la simulazione
echo "Avvio Task2..."
cd /home/ubuntu/progetto_12_ws
source install/setup.bash
ros2 launch my_bringup launch_task2.py

# Al termine, kill il logger
kill $LOGGER_PID

# Analizza
echo "Generando analisi..."
python3 /home/ubuntu/progetto_12_ws/src/my_robot_description/Task/Task2/results/utils.py analyze

echo "✓ Fatto!"
```

Rendi eseguibile:
```bash
chmod +x run_with_analysis.sh
./run_with_analysis.sh
```

## Cosa ci aspettiamo di vedere

### Grafici

1. **joint_trajectories.png**
   - 8 sottoplot, uno per ogni giunto
   - Curves lisce indicano movimenti fluidi
   - Salti o discontinuità indicano problemi di planning

2. **joint_velocities.png**
   - Velocità di ogni giunto nel tempo
   - Dovrebbe essere quasi zero durante le pause
   - Picchi durante i movimenti

3. **marker_positions.png**
   - 4 sottoplot (A, B, C, D)
   - Posizione costante se il marker è statico (✓)
   - Piccole variazioni se il tracking è rumoroso

### Tabelle

1. **statistics_joints.csv**
   - Torso: range piccolo (poca variazione)
   - Arm1-7: range più ampio (molti movimenti)

2. **task_events.csv**
   - Sequence: WAIT_PICK → LOWER_PICK → ATTACH → LIFT_AFTER_PICK → MOVE_TO_PLACE → LOWER_PLACE → DETACH → RETURN_SAFE
   - Ripetuto per ogni oggetto (cocacola, pringles)

## Interpretazione risultati

### ✓ Esecuzione corretta
- Grafici lisci senza discontinuità
- Marker posizioni costanti nel tempo
- Sequenza di stati logica negli eventi

### ⚠ Possibili problemi
- **Spikes nei giunti**: IK non ottimale
- **Marker che si muovono**: tracking instabile
- **Eventi mancanti**: fallimento di una fase
- **Oscilazioni**: controller gains troppo alti

## Next Steps

1. Esegui il logger durante una simulazione completa
2. Genera i grafici con `analysis.py`
3. Verifica che i risultati corrispondono alle aspettative
4. In caso di problemi, usa i grafici per debuggare
