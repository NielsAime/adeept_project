# Adeept Robot — Navigation ArUco & Pick-and-Place

Système robot autonome utilisant une caméra externe, des marqueurs ArUco et un bras articulé pour naviguer, éviter les obstacles et saisir des objets.

## Architecture

Le projet est divisé en deux espaces :

| Espace | Plateforme | Rôle |
|---|---|---|
| `pc_worspace_client/` | PC (maître) | Vision, navigation, envoi de commandes TCP |
| `robot_workspace/` | Raspberry Pi (esclave) | Réception commandes, contrôle hardware, évitement |

### Flux de données

```
[Caméra USB]
     │
     ▼
[PC] aruco_detector → SceneState (position, angle, distance)
     │
     ▼
[PC] path_planning → NavigationCommand (left_speed, right_speed, phase)
     │  TCP JSON
     ▼
[Pi] tcp_server → on_command()
     │
     ├── move  → MotorController + SteeringServo
     ├── stop  → MotorController
     └── start_pick → aruco_arm (scan W + IK + pick-and-place)
```

### Phases de navigation (PC)

- **ROTATE** : rotation sur place pour aligner le robot vers la cible
- **MOVE** : déplacement en ligne vers la cible, direction via servo avant
- **GRASP** : déclenche le workflow `aruco_arm` sur le Pi
- **DONE** : mission terminée, arrêt

### Workflow `aruco_arm` (Pi)

1. Balayage en W (5 waypoints pan/épaule/coude) à la recherche du marqueur ArUco ID=3
2. Estimation de pose → conversion coordonnées caméra → repère robot (via IK)
3. Séquence pick : approche IK → fermeture pince → lever
4. Séquence place : déplacement vers position de dépôt → ouverture pince → retour home

### Gestion des obstacles (Pi)

- Thread ultrason à 10 Hz surveille en permanence
- Si obstacle détecté : verrou hardware → arrêt → manœuvre d'évitement (`ObstacleAvoider`)
- Cooldown de 2 s après évitement avant réactivation de la détection

## Installation

```bash
# Sur le PC
python -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate
pip install -r requirement.txt

# Sur le Raspberry Pi
pip install opencv-contrib-python RPi.GPIO adafruit-circuitpython-pca9685
```

## Lancement

```bash
# 1. Sur le Pi — démarrer le serveur robot
cd robot_workspace
python3 main_robot.py

# 2. Sur le PC — démarrer la navigation (après avoir lancé le Pi)
cd pc_worspace_client
python main_pc.py
```

### Calibration caméra (à faire une fois)

```bash
cd pc_worspace_client
python vision/compute_calibration.py
# Génère camera_calibration_data.npz à la racine du projet
```

## Configuration

Dans `pc_worspace_client/main_pc.py` :
```python
ROBOT_IP     = '192.168.137.175'  # IP du Raspberry Pi
CAMERA_INDEX = 0                  # Index de la webcam USB
```

## Commandes clavier (PC)

| Touche | Action |
|---|---|
| `ESC` | Arrêt d'urgence et déconnexion |
| `r` | Reset du navigateur (nouvelle mission) |

## Commandes TCP (JSON)

| Commande | Paramètres | Description |
|---|---|---|
| `move` | `left`, `right` (-100..100) | Déplacement différentiel |
| `stop` | — | Arrêt immédiat |
| `start_pick` | — | Déclenche le workflow aruco_arm |
