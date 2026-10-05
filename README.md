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
| Sens | une vitesse positive fait avancer les roues de gauche (4 et 2) et reculer celles de droite (8 et 1) : vérifié au sol |

Pour retrouver les moteurs, sans rien faire bouger : `python3 tools/dxl_scan.py`.

## Préparer une Raspberry Pi

Les Pi sont préparées avant d'être confiées aux étudiants.

1. **Carte SD** : Raspberry Pi OS Trixie 64 bits avec bureau, configurée au premier démarrage par cloud-init : nom du robot, wifi, SSH, et deux comptes. `etudiant` n'a pas les droits d'administration et ouvre le bureau automatiquement ; `admin` a `sudo`. *(Script à venir.)*
2. **Port série des moteurs** : `sudo setup/configure_serial.sh`, puis redémarrer. Le vrai UART passe sur les GPIO 14/15, le Bluetooth sur le mini-UART, et la console série est retirée.
3. **Environnement Python** : `sudo setup/install_robot_env.sh`. Il installe dans `/opt/robot/venv` Python 3.12, pypot, pyrealsense2, pyserial, matplotlib, ipython, JupyterLab, ipywidgets et Ultralytics (YOLO), ainsi que la bibliothèque `holorobot` (le dépôt est copié dans `/opt/robot/src` ; relancer le script la met à jour). Il place cet environnement en tête du `PATH` de tous les comptes. Les étudiants ne peuvent pas le modifier, mais chacun peut y ajouter des paquets pour lui seul avec `pip install --user`. Pour tenir sur une carte de 16 Go, PyTorch est pris en version CPU : celle de PyPI apporte pour ARM environ 5 Go de bibliothèques Nvidia, inutiles sur la Pi. L'environnement occupe ainsi 1,8 Go, et il reste environ 4 Go libres sur la carte.
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
- `scan.points()` donne la liste des (angle, distance), `scan.xy()` les coordonnées cartésiennes dans un tableau numpy (x vers le 0° du lidar, y vers sa gauche), et `for scan in lidar.scans():` permet de boucler sur les tours.
- `scan.segments()` trouve les segments de droite (murs, faces de meubles), du plus sûr au moins sûr : extrémités, longueur, distance et direction, équation cartésienne a·x + b·y + c = 0 et confiance entre 0 et 1. Environ 10 ms par tour sur la Pi.
- `holorobot.lidar.decode(octets)` décode un enregistrement brut, sans robot. Exemple : `tests/data/x4_raw.bin`.

Le pilote suit le protocole officiel du X4 et n'occupe que 4 % d'un cœur de la Pi. On n'utilise pas PyLidar3, qui est bogué : il fait la moyenne des directions sans mesure avec les vraies distances et corrige mal les angles. Sur les mêmes données, 40 % de ses points sont des fantômes.

Documentation complète, avec exemples et pièges courants : [`docs/lidar.md`](docs/lidar.md).

### Moteurs : `holorobot.motors`

```python
from holorobot.motors import Motors

with Motors() as motors:               # ouvre le bus, trouve les moteurs, les met en mode roue
    print(motors.ids)                  # [1, 2, 4, 8] sur MobileRobot-1
    motors.run({4: 90}, duration=2)    # le moteur 4 tourne à 90 °/s pendant 2 s, puis s'arrête
```

- Les vitesses sont en °/s, limitées à 720 °/s par défaut. `set_speeds()` envoie une consigne, `run()` la maintient pendant une durée, `stop()` freine, `release()` libère les roues.
- **Chien de garde** : sans nouvelle consigne pendant 0,5 s, toutes les roues s'arrêtent. Un carnet planté ou un navigateur fermé n'emporte donc pas le robot. Il vit dans le programme : si celui-ci est tué brutalement, les roues gardent leur dernière vitesse ; coupez alors l'alimentation.
- **Mode articulation** (contrôle en position) : `set_joint_mode()`, puis `move_to({id: angle})`. Pas utile pour les roues, mais de quoi faire, par exemple, une tourelle pan-tilt pour la caméra.
- **Un seul programme à la fois** : le bus est verrouillé, et un second programme (un autre carnet, par exemple) reçoit un message clair au lieu de brouiller les échanges. pypot, lui, se contente d'un avertissement.

Documentation complète, avec la cinématique des roues mecanum et holonomes : [`docs/moteurs.md`](docs/moteurs.md). Les schémas de `docs/img/` sont produits par `docs/img/schemas.py`.

### Tests

`python3 tests/test_lidar.py`, `python3 tests/test_motors.py`, ou `python3 -m pytest tests`. Les premiers rejouent 8 s d'octets bruts enregistrés sur MobileRobot-1 et vérifient la détection des segments sur des scans simulés ; les seconds vérifient la logique des moteurs avec un faux bus, sans robot : mode roue et mode articulation, limites et valeurs refusées, chien de garde, changement d'identifiant, fermeture.

## Contenu du dépôt

- `holorobot/` : la bibliothèque Python des étudiants (`pyproject.toml` pour l'installer).
- `notebooks/` : carnets JupyterLab pour les étudiants : `tableau_de_bord.ipynb` (batterie, lidar et ses segments de droite, caméra et objets reconnus par YOLO, moteurs : liste, températures, identifiants, curseurs de vitesse et de position) et `locomotion_holonome.ipynb` (des moteurs jusqu'à la fonction de pilotage du robot).
- `docs/` : sa documentation, module par module, et `memo_projet.md`, le mémo de l'enseignant (état des robots, réseau et accès, procédures, incidents connus).
- `tests/` : tests de la bibliothèque, avec des enregistrements du robot dans `tests/data/`.
- `setup/` : scripts de préparation des Pi.
- `supervision/` et `bin/supervision` : l'outil de supervision des robots, pour l'enseignant (voir [`docs/supervision.md`](docs/supervision.md)).
- `tools/` : outils de mise au point.
  - `dxl_scan.py` : cherche les moteurs Dynamixel sur le bus, sans rien faire bouger.
  - `wheel_test.py` : fait tourner chaque roue dans les deux sens et vérifie l'angle parcouru (roues en l'air).
  - `battery.py` : tension et charge estimée de la batterie, lues par les moteurs ; `--watch 60` pour la surveiller.
  - `camera_snapshot.py` : photo couleur et carte de profondeur de la RealSense, côte à côte ; `--rotate 180` si la caméra est montée tête en bas, comme sur MobileRobot-1.
  - `lidar_snapshot.py` : lit les informations du lidar et enregistre une image de quelques tours.
  - `lidar_stress.py` : essais de robustesse du pilote du lidar avec des threads, en 40 s environ.
