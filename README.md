# TeachingHolonomousRobot

Robot mobile holonome pour un projet étudiant : chaque groupe construit un robot similaire, avec le même matériel, et le programme en Python. **MobileRobot-1** est le robot de référence sur lequel on met tout au point.

## Matériel

- **Base** : châssis en MDF, 4 roues mecanum. Le robot peut avancer, se déplacer de côté et tourner sur lui-même, y compris les trois à la fois.
- **Moteurs** : 4 Dynamixel MX-12W, un par roue, chaînés sur un même bus série TTL demi-duplex (protocole 1.0).
- **Calcul** : Raspberry Pi 4 Model B (4 Go).
- **Interface moteurs** : carte maison posée sur le GPIO. Elle relie le port série de la Pi (GPIO 14/15) au bus Dynamixel et distribue le 11 V.
- **Lidar** : YDLidar X4 (code modèle 6), branché en USB par son adaptateur (puce CP2102, `/dev/ttyUSB0`, 128 000 bauds).
- **Caméra** : Intel RealSense D435i (couleur, profondeur, centrale inertielle), sur un port USB 3. Elle définit l'avant du robot ; le lidar est à l'arrière. Sur MobileRobot-1, elle est montée tête en bas pour faciliter le câblage : les images sont à retourner de 180°. Elle ne mesure pas la profondeur à moins d'environ 30 cm.
- **Énergie** : pack de 3 éléments Li-ion 18650 de 2500 mAh en série (12,6 V chargé, environ 11 V en moyenne) pour les moteurs, et un convertisseur 5 V pour la Pi. Ne pas descendre sous 3,3 V par élément (9,9 V pour le pack).

## Bus moteurs de MobileRobot-1

| Réglage | Valeur |
|---|---|
| Port | `/dev/serial0`, c'est-à-dire l'UART PL011 sur les GPIO 14/15 |
| Vitesse | registre 33 des moteurs, soit 58 824 bauds ; on ouvre le port à 57 600 |
| Identifiants | 1 arrière droite, 2 arrière gauche, 4 avant gauche, 8 avant droite |
| Mode | roue (rotation continue) |

Pour retrouver les moteurs, sans rien faire bouger : `python3 tools/dxl_scan.py`.

## Préparer une Raspberry Pi

Les Pi sont préparées avant d'être confiées aux étudiants.

1. **Carte SD** : Raspberry Pi OS Trixie 64 bits avec bureau, configurée au premier démarrage par cloud-init : nom du robot, wifi, SSH, et deux comptes. `etudiant` n'a pas les droits d'administration et ouvre le bureau automatiquement ; `admin` a `sudo`. *(Script à venir.)*
2. **Port série des moteurs** : `sudo setup/configure_serial.sh`, puis redémarrer. Le vrai UART passe sur les GPIO 14/15, le Bluetooth sur le mini-UART, et la console série est retirée.
3. **Environnement Python** : `sudo setup/install_robot_env.sh`. Il installe dans `/opt/robot/venv` Python 3.12, pypot, pyrealsense2, pyserial, matplotlib, ipython, JupyterLab et ipywidgets, ainsi que la bibliothèque `holorobot` (le dépôt est copié dans `/opt/robot/src` ; relancer le script la met à jour). Il place cet environnement en tête du `PATH` de tous les comptes. Les étudiants ne peuvent pas le modifier, mais chacun peut y ajouter des paquets pour lui seul avec `pip install --user`.
4. **JupyterLab** : `sudo setup/install_jupyter.sh`, qui demande un mot de passe. JupyterLab tourne alors en permanence sous le compte `etudiant` : depuis un navigateur du même réseau, `http://<nom-du-robot>.local:8888`. Les carnets de `notebooks/` sont copiés dans `~etudiant/notebooks`, sans jamais écraser un carnet existant.

## Bibliothèque `holorobot`

### Lidar : `holorobot.lidar`

```python
from holorobot.lidar import Lidar

with Lidar() as lidar:          # lance le moteur et attend le premier tour
    scan = lidar.get_scan()     # attend le tour suivant : les données sont toujours fraîches
    print(len(scan), "points")
    angle, distance = scan.nearest()
```

- Un scan correspond à un tour du lidar : environ 7 par seconde, avec environ 500 points chacun, horodatés.
- Les angles sont en degrés, comptés dans le sens des aiguilles d'une montre vu de dessus, depuis le 0° du lidar. Les distances sont en mètres. Les directions sans mesure (rien vu, ou obstacle à moins de 12 cm) sont absentes du scan : elles ne valent jamais 0.
- `scan.points()` donne la liste des (angle, distance), `scan.xy()` les coordonnées cartésiennes (x vers le 0° du lidar, y vers sa gauche), et `for scan in lidar.scans():` permet de boucler sur les tours.
- `holorobot.lidar.decode(octets)` décode un enregistrement brut, sans robot. Exemple : `tests/data/x4_raw.bin`.

Le pilote suit le protocole officiel du X4 et n'occupe que 4 % d'un cœur de la Pi. On n'utilise pas PyLidar3, qui est bogué : il fait la moyenne des directions sans mesure avec les vraies distances et corrige mal les angles. Sur les mêmes données, 40 % de ses points sont des fantômes.

Documentation complète, avec exemples et pièges courants : [`docs/lidar.md`](docs/lidar.md).

### Tests

`python3 tests/test_lidar.py`, ou `python3 -m pytest tests` : ils rejouent 8 s d'octets bruts enregistrés sur MobileRobot-1.

## Contenu du dépôt

- `holorobot/` : la bibliothèque Python des étudiants (`pyproject.toml` pour l'installer).
- `notebooks/` : carnets JupyterLab pour les étudiants, par exemple `decouverte_robot.ipynb` (batterie, lidar et caméra, sans bouger les roues).
- `docs/` : sa documentation, module par module.
- `tests/` : tests de la bibliothèque, avec des enregistrements du robot dans `tests/data/`.
- `setup/` : scripts de préparation des Pi.
- `tools/` : outils de mise au point.
  - `dxl_scan.py` : cherche les moteurs Dynamixel sur le bus, sans rien faire bouger.
  - `wheel_test.py` : fait tourner chaque roue dans les deux sens et vérifie l'angle parcouru (roues en l'air).
  - `battery.py` : tension et charge estimée de la batterie, lues par les moteurs ; `--watch 60` pour la surveiller.
  - `camera_snapshot.py` : photo couleur et carte de profondeur de la RealSense, côte à côte ; `--rotate 180` si la caméra est montée tête en bas, comme sur MobileRobot-1.
  - `lidar_snapshot.py` : lit les informations du lidar et enregistre une image de quelques tours.
  - `lidar_stress.py` : essais de robustesse du pilote du lidar avec des threads, en 40 s environ.
