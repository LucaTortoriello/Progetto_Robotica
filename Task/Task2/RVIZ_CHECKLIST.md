# Checklist RViz - Task 2

Questa checklist serve per verificare in laboratorio che terne, pose e fase di pick/place siano corretti.

## 1) Avvio ambiente

1. Apri un terminale nella workspace.
2. Sorgenta ROS2 e la tua workspace.
3. Lancia Task 2 con il file di launch del progetto.
4. Apri RViz2 in un secondo terminale.

Riferimento launch: [Task/Task2/launch/launch_task2.py](launch/launch_task2.py)

## 2) Configurazione iniziale RViz

1. In Global Options imposta Fixed Frame su base_footprint.
2. Se noti inconsistenze frame, prova base_link e mantieni quello scelto.
3. Aggiungi i display:
- TF
- RobotModel
- Pose (una istanza per ogni topic ArUco)
- Marker o MarkerArray (se presenti)

## 3) Topic da caricare in Pose

1. /aruco_base_1
2. /aruco_base_2
3. /aruco_base_3
4. /aruco_base_4

## 4) Controlli prima del pick

1. Nessun warning persistente su TF.
2. Pose ArUco stabili dopo la fine della scansione testa.
3. Marker della Coca-Cola coerente con la posizione in Gazebo.
4. Gripper visibile e aggiornato in RobotModel/TF.

## 5) Controlli durante il pick (fase critica)

1. Arrivo sopra oggetto:
- Il gripper deve essere sopra la lattina, non lateralmente.

2. Discesa:
- Movimento quasi verticale.
- Nessuna rotazione improvvisa all ultimo tratto.

3. Chiusura gripper:
- Deve avvenire solo quando il gripper e centrato.

4. Post-chiusura:
- L oggetto deve seguire il gripper senza cadute o oscillazioni anomale.

## 6) Segnali problema e interpretazione

1. Rotazione improvvisa vicino alla lattina:
- Possibile cambio ramo IK.

2. Gripper a lato della lattina:
- Offset di pick non corretto.

3. Urto e caduta oggetto:
- Discesa troppo profonda o troppo rapida.

4. Pose marker che ballano:
- Rumore o perdita tracking ArUco.

## 7) Note da prendere durante il test

1. Timestamp del problema.
2. Marker coinvolto e topic relativo.
3. Tipo errore: centratura, rotazione, profondita, timing chiusura.
4. Se il problema e ripetibile.
5. Screenshot RViz con TF e Pose nel momento dell errore.

## 8) Cosa modificare dopo la prova

1. Se manca centratura:
- Ritoccare offset in pick approach e lower pick nel nodo Task 2.

2. Se urta in discesa:
- Ridurre profondita di lower pick.
- Aumentare durata traiettoria vicino all oggetto.

3. Se ruota all ultimo:
- Stabilizzare la solve_ik con seed piu vicini e filtro sui salti.

4. Se marker instabile:
- Applicare filtro temporale alle pose ArUco nel detector.

## 9) Salvataggio configurazione RViz

1. Quando la vista e corretta, salva la configurazione RViz.
2. Riusa quel file nelle prove successive per evitare setup manuale.
