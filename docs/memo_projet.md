# Mémo du projet « Robot holonome »

Mémo de travail de l'enseignant : tout ce qu'il faut savoir pour reprendre le projet, faire tourner les robots pendant le TP et intervenir en cas de problème. Les documents pour les étudiants sont le [README](../README.md), [docs/moteurs.md](moteurs.md), [docs/lidar.md](lidar.md) et les carnets.

**Aucun secret dans ce fichier ni dans le dépôt** (dépôt public) : ni mot de passe, ni clé Tailscale, ni code wifi. Le paragraphe [Secrets](#secrets) dit seulement où ils sont gardés.

*État au 4 octobre 2026 au soir, veille du premier TP (lundi 5 octobre 2026).*

---

## 1. Le projet en bref

- **Le cours** : un projet étudiant où chaque groupe construit un robot holonome avec le même matériel (Raspberry Pi 4, moteurs Dynamixel, lidar, caméra RealSense) et le programme en Python depuis un navigateur (JupyterLab). Chaque groupe dessine sa propre mécanique (impression 3D, découpe laser) : 4 roues mecanum ou 3 roues holonomes selon les robots.
- **MobileRobot-1** est le robot de référence, sur lequel tout a été mis au point. Les robots **2 à 6** sont des clones de sa carte SD (préparés le 4 octobre 2026).
- **Contraintes réseau** : ni les Pi ni le routeur ne doivent être reliés au réseau de l'université. Les robots sont sur le wifi d'un **routeur 4G** (`DU_ROBOT_1`), et les étudiants y accèdent depuis les PC de l'université par Internet, grâce à **Tailscale Funnel**.
- **Dépôt** : `github.com/moogly-jungle/TeachingHolonomousRobot` (public). Bibliothèque `holorobot`, carnets, documentation, outils, scripts de préparation des Pi.
- **Page du cours** : https://moogly-jungle.github.io/Enseignement/RobotHolonome/ (dépôt `moogly-jungle/moogly-jungle.github.io`, fichier `Enseignement/RobotHolonome/index.html`). Liens vers le JupyterLab de chaque robot, premiers pas, règles d'or, feuilles Jupyter, documentation.

## 2. État des robots

| Robot | Nom de la Pi | Adresse JupyterLab | SSH (alias Mac / Dell) | Wifi | État |
|---|---|---|---|---|---|
| 1 | MobileRobot-1 | https://holobot1.tail5610aa.ts.net | `du-robot-holobot1` / `holobot1` | DU_ROBOT_1 (192.168.9.100) | en service ; moteurs, lidar, caméra montés |
| 2 | MobileRobot-2 | https://holobot2.tail5610aa.ts.net | `du-robot-holobot2` / `holobot2` | DU_ROBOT_1 (.102) | en service ; pas encore de moteurs branchés |
| 3 | MobileRobot-3 | https://holobot3.tail5610aa.ts.net | `du-robot-holobot3` / `holobot3` | — | **injoignable depuis le redémarrage du 4 octobre (20 h 20)** : à vérifier sur place |
| 4 | MobileRobot-4 | https://holobot4.tail5610aa.ts.net | `du-robot-holobot4` / `holobot4` | DU_ROBOT_1 (.104) | en service |
| 5 | MobileRobot-5 | https://holobot5.tail5610aa.ts.net | `du-robot-holobot5` / `holobot5` | DU_ROBOT_1 (.105) | en service |
| 6 | MobileRobot-6 | https://holobot6.tail5610aa.ts.net | `du-robot-holobot6` / `holobot6` | DU_ROBOT_1 (.106) | en service |

Les adresses 192.168.9.x sont celles qu'a données le routeur (DHCP), pas des adresses fixes. Les six robots sont listés sur la page du cours. Le robot 3, une fois rallumé, doit recevoir le carnet `mon_code.ipynb` et le ménage des dossiers du compte `etudiant` (voir [§ 6.6](#66-mises-à-jour-faites-sur-tous-les-robots)).

## 3. Matériel (MobileRobot-1)

- **Raspberry Pi 4 Model B**, Raspberry Pi OS Trixie 64 bits avec bureau.
- **Moteurs** : 4 Dynamixel MX-12W, protocole 1.0, sur `/dev/serial0` (UART PL011 des GPIO 14/15, carte maison sur le GPIO), 57 600 bauds (registre 33 = 58 824 bauds en réalité). Identifiants : **1 arrière droite, 2 arrière gauche, 4 avant gauche, 8 avant droite**. Vitesse positive : les roues de gauche (4, 2) avancent, celles de droite (8, 1) reculent (vérifié au sol). Unité de vitesse du MX-12 : 0,916 tr/min, soit environ 5,5 °/s. Le moteur 1 chauffe plus que les autres au repos (57 °C contre 43 °C), coupure à 70 °C.
- **Galets** : montés en **O** sur MobileRobot-1 (losange vu de dessus), alors que les formules du carnet sont écrites pour un montage en **X**. Proposé : échanger les roues avant et arrière de chaque côté. Pas encore décidé.
- **Lidar** : YDLidar X4 sur `/dev/ttyUSB0` (adaptateur CP210x), 128 000 bauds. Son 0° regarde vers l'**arrière** du robot. Lu par notre pilote `holorobot.lidar` (PyLidar3 est bogué).
- **Caméra** : RealSense D435i, montée **tête en bas** à l'avant (images à retourner de 180°), relevée d'environ 14° (voulu : elle voit mieux ainsi) et légèrement penchée (roulis 6°). Son étalonnage avait été abîmé : restauré par `reset_to_factory_calibration()` le 2 octobre (ancienne table sauvegardée sur MobileRobot-1 dans `~admin`). Signe d'un étalonnage abîmé : des distances de « 65 m » et beaucoup de disparités négatives. La centrale inertielle n'apparaît pas avec cette version de pyrealsense2. Les distances de la caméra sont environ 9 % plus longues que celles du lidar : on ne sait pas encore lequel a tort (test au mètre ruban à faire).
- **Batterie** : 3 éléments Li-ion 18650 en série, environ 11 V. Elle alimente directement les moteurs ; la Pi passe par un convertisseur 5 V. La tension se lit par les moteurs (`voltages()` dans `holorobot.motors`, `tools/battery.py`).

## 4. Le dépôt

```
holorobot/      bibliothèque des étudiants : lidar.py, motors.py
notebooks/      carnets copiés chez les étudiants : decouverte_moteurs.ipynb, tableau_de_bord.ipynb
docs/           moteurs.md, lidar.md, img/ (schémas produits par img/schemas.py), ce mémo
tools/          battery.py, wheel_test.py, dxl_scan.py, lidar_snapshot.py, lidar_stress.py, camera_snapshot.py
tests/          test_motors.py (22 tests, faux bus), test_lidar.py (14 tests, enregistrement réel dans tests/data)
setup/          install_robot_env.sh, configure_serial.sh, install_jupyter.sh, règles udev RealSense
```

Tests : `python3 tests/test_motors.py` et `python3 tests/test_lidar.py` (il faut numpy ; sur un robot, avec `/opt/robot/venv/bin/python`).

### 4.1 `holorobot.motors` (détails dans docs/moteurs.md)

- `find_ids()`, `voltages()`, `status()` : lisent le bus sans rien changer aux moteurs (ni mode, ni couple).
- `Motors(ids=None, wheel_mode=True)` : ouvre le bus (verrou `flock` : un seul programme à la fois ; un second reçoit un message clair). Mode roue (`set_speeds`, `run`, `stop`, `release`) ou articulation (`set_joint_mode`, `set_positions`, `move_to`). Vitesses en °/s, limitées à 720 ; vitesse d'articulation d'au moins 6 °/s (en dessous, le MX-12W comprend « vitesse maximale »).
- **Chien de garde** : sans nouvelle consigne pendant 0,5 s, les roues s'arrêtent. Il vit dans le programme : un noyau tué brutalement laisse les roues tourner (couper l'alimentation).
- `move_to()` attend que le moteur soit arrivé (à 3° près) **et arrêté**. `close()` arrête et coupe le couple ; `close(hold=True)` libère le bus en laissant le couple (les roues freinent, les articulations gardent leur consigne).
- Robustesse : lectures réessayées (des réponses se perdent sur le bus), pause de 0,1 s après chaque écriture en EEPROM (mode, identifiant), ping relancé à l'ouverture si un moteur demandé ne répond pas.

### 4.2 `holorobot.lidar` (détails dans docs/lidar.md)

`Lidar()` (port exclusif, moteur du lidar piloté par DTR, thread de lecture), `get_scan()` / `latest()` → `Scan` (angles en degrés dans le sens horaire, distances en mètres), `scan.xy()` (tableau numpy, x vers le 0° du lidar, y vers sa gauche), `scan.nearest()`, `scan.segments()` (segments de droite avec une confiance entre 0 et 1 et l'équation a x + b y + c = 0).

### 4.3 Les carnets

- `decouverte_moteurs.ipynb` : des moteurs à la fonction de pilotage (vx, vy, ω) → vitesses des roues, roues mecanum et holonomes, contrôle en position (idée de tourelle pan-tilt). Repère du robot : x à droite, y devant, ω positif dans le sens trigonométrique vu de dessus.
- `tableau_de_bord.ipynb` : batterie, moteurs (liste, températures, changement d'identifiant, curseur de vitesse, curseur de position, relâcher), lidar et segments, caméra et YOLO. **Chaque cellule est indépendante** : elle ouvre ce dont elle a besoin et le referme. Les curseurs passent par des tâches de fond : le bouton *Arrêt* les arrête, pas *Interrupt Kernel*. Tant qu'une roue tourne au curseur, les autres cellules des moteurs répondent « bus déjà utilisé ».
- `mon_code.ipynb` : carnet vide (une cellule) pour le code des groupes. Il n'est **pas dans le dépôt** : il est déposé sur chaque robot (et dans l'image des cartes).

Les carnets du dépôt ont été générés par des scripts qui n'existent plus : on les modifie maintenant directement.

## 5. Système des Pi

- **Comptes** : `etudiant` (uid 1000, pas de sudo, ouvre le bureau automatiquement, fait tourner JupyterLab) et `admin` (sudo avec mot de passe). Les clés SSH du Dell et du Mac d'Olivier sont autorisées sur `admin` (et sur `etudiant` pour celle du Mac).
- **Python** : environnement partagé `/opt/robot/venv` (Python 3.12 par uv, PyTorch CPU, Ultralytics, pyrealsense2, pypot, JupyterLab, ipywidgets…), en tête du `PATH` de tous les comptes. `holorobot` y est installé en mode éditable depuis **`/opt/robot/src/TeachingHolonomousRobot`** (copie root du dépôt). Copie de travail d'admin : `~admin/TeachingHolonomousRobot`.
- **JupyterLab** : service systemd `jupyterlab`, compte `etudiant`, port 8888 sur toutes les interfaces (donc aussi `http://MobileRobot-N.local:8888` depuis le même réseau ; le Funnel le sert en https), dossier racine `/home/etudiant`, configuration `/etc/jupyter/robot_server_config.py` (mot de passe haché = celui du compte `etudiant` du robot). Carnets dans `~etudiant/notebooks`. Un seul noyau, `python3`, celui de `/opt/robot/venv`.
- **Port série** : `enable_uart=1`, `dtoverlay=miniuart-bt` (le Bluetooth reste disponible pour une manette), console série retirée (`setup/configure_serial.sh`).
- **Démarrage** : cloud-init (fichiers `user-data`, `network-config`, `meta-data` dans `/boot/firmware`) ; c'est `user-data` qui fixe le nom de la Pi à chaque démarrage. Mémoire d'échange : `rpi-swap` (zram et fichier `/var/swap`, recréé tout seul).
- **Compte etudiant** : les dossiers Music, Pictures, Public, Templates et Videos ont été retirés, et le bureau ne les recrée plus (`~/.config/user-dirs.dirs` les envoie vers `$HOME/`, `~/.config/user-dirs.conf` contient `enabled=False`).
- Sans pile, une Pi démarre à l'heure de sa dernière extinction (ou de l'image pour un clone), puis se met à l'heure par Internet : les dates des tout premiers messages d'un journal peuvent être fausses.

## 6. Réseau et accès

### 6.1 Wifi

- **`DU_ROBOT_1`** : routeur 4G du TP (2,4 GHz, réseau 192.168.9.0/24, passerelle 192.168.9.1). Profil NetworkManager, priorité **10** : c'est le réseau préféré de tous les robots. Le code wifi n'est que sur les robots (fichier NetworkManager lisible par root seul).
- **`RHOBAN`** : wifi du labo, profil netplan (cloud-init), priorité 0 : réseau de secours. Ethernet en DHCP aussi.
- MobileRobot-1 a basculé sur `DU_ROBOT_1` le 4 octobre à 16 h 57 (script `/usr/local/sbin/bascule_tp.sh`, prévu pour le lundi 8 h ; sa minuterie est désactivée puisqu'il a servi).

### 6.2 Tailscale et Funnel

- Les robots sont dans le tailnet d'Olivier (`tail5610aa.ts.net`), sous les noms `holobot1` à `holobot6`. Les robots 2 à 6 se sont inscrits au premier démarrage, avec une clé d'inscription par robot.
- **Funnel** ouvre deux portes sur Internet, sans rien ouvrir sur le routeur :
  - `https://holobotN.tail5610aa.ts.net` (port 443) → JupyterLab (`http://127.0.0.1:8888`) ;
  - port **10000** en TCP dans du TLS → SSH (`localhost:22`).
  - Commandes : `tailscale funnel --bg 8888` et `tailscale funnel --bg --tls-terminated-tcp=10000 tcp://localhost:22`. État : `tailscale funnel status`. La configuration survit aux redémarrages.
- **SSH par le Funnel : par clé seulement** (sshd refuse le mot de passe aux connexions qui arrivent du Funnel, c'est-à-dire de 127.0.0.1 ou ::1) ; en local, le mot de passe reste accepté.
- Configuration SSH côté client (Mac) :

```
Host du-robot-holobot1
    HostName holobot1.tail5610aa.ts.net
# … une entrée par robot, de 1 à 6 …
Host du-robot-holobot*
    User admin
    ProxyCommand openssl s_client -quiet -verify_quiet -connect %h:10000 -servername %h
```

  Sur le Dell, un seul bloc `Host holobot?` avec `HostName %h.tail5610aa.ts.net` et la même `ProxyCommand`.
- Un robot qu'on ne joint plus par le Funnel mais qui est sur le wifi du TP se joint **en rebond par un autre robot** : `ssh -J du-robot-holobot2 admin@MobileRobot-3.local` (les noms `.local` marchent entre robots du même réseau).
- Dans la console Tailscale (https://login.tailscale.com/admin) : désactiver l'expiration des clés de holobot2 à holobot6 (*Machines* → … → *Disable key expiry*), comme pour holobot1 ; révoquer les clés d'inscription si elles étaient réutilisables.

### 6.3 Surveillance du Funnel

Service `funnel-watchdog` sur chaque robot (`/usr/local/sbin/funnel_watchdog.sh`, nom du robot écrit dans la variable `H`). Chaque minute, il demande au DNS public (Cloudflare) les adresses des relais du Funnel, puis teste `https://holobotN…/lab` par chacun. Il relance `tailscaled` :

- après 3 vérifications de suite sans aucun relais qui réponde ;
- après 10 vérifications de suite avec au moins un relais muet ;
- après 10 vérifications de suite où le nom public n'existe pas alors qu'Internet marche ;
- jamais deux relances à moins de 10 minutes, ni dans les 10 premières minutes après son démarrage.

Journal : `journalctl -u funnel-watchdog` (changements d'état) ; relances dans `/var/log/funnel-watchdog.log`. Essai à blanc : `DRYRUN=1 INTERVAL=1 COOLDOWN=0 bash /usr/local/sbin/funnel_watchdog.sh`.

### 6.4 Comportements à connaître

- **Après un changement de réseau** (bascule de wifi), le Funnel peut rester muet quelques minutes : les relais ferment la connexion (« unexpected eof » côté client). La surveillance relance `tailscaled` et tout revient en 1 à 4 minutes.
- **Nom public d'un nouveau robot** : Tailscale peut mettre jusqu'à une vingtaine de minutes à publier `holobotN.tail5610aa.ts.net` (la documentation dit 10). Pendant ce temps, le Funnel marche déjà si l'on passe directement par un relais : `curl --resolve holobotN.tail5610aa.ts.net:443:<ip du relais> https://holobotN.tail5610aa.ts.net/lab`. Une relance de `tailscaled` a suffi pour le robot 3, pas pour le 6, qui a fini par être publié seul.
- **Cache DNS** : un nom demandé avant sa publication reste « inconnu » pendant 5 minutes dans le résolveur du réseau (cache négatif de `ts.net`). Les serveurs qui font foi : `dig @ns1.dnsimple.com holobotN.tail5610aa.ts.net`.
- Les relais du Funnel ne sont pas les mêmes pour tous les robots (176.58.90.x pour les uns, 185.40.234.x pour les autres).

### 6.5 Interventions courantes

Toutes les commandes `sudo` des robots demandent le mot de passe d'`admin`.

- **Déployer la bibliothèque et les carnets** (depuis une copie du dépôt) :

```
R=du-robot-holobot1
rsync -a --delete --exclude .git --exclude __pycache__ ./ $R:TeachingHolonomousRobot/
ssh $R
sudo rsync -a --delete --chown=root:root --exclude .git --exclude __pycache__ ~/TeachingHolonomousRobot/ /opt/robot/src/TeachingHolonomousRobot/
sudo /opt/robot/venv/bin/python -m compileall -q /opt/robot/src/TeachingHolonomousRobot/holorobot
```

- **Remplacer un carnet chez les étudiants** : ne jamais écraser sans regarder. Comparer d'abord, garder l'ancienne copie dans `~admin/sauvegardes_carnets/<date_heure>/`, puis `sudo install -o etudiant -g etudiant -m 644 ~/TeachingHolonomousRobot/notebooks/X.ipynb /home/etudiant/notebooks/`. Après un remplacement, l'onglet ouvert dans JupyterLab doit être rechargé (*File → Reload Notebook from Disk*), sinon la sauvegarde automatique réécrit l'ancienne version.
- **Exécuter un carnet sans navigateur** (validation, sous le compte etudiant) : le copier dans un dossier de `/tmp` appartenant à `etudiant`, puis `sudo -u etudiant env HOME=/home/etudiant /opt/robot/venv/bin/jupyter nbconvert --to notebook --execute --inplace X.ipynb`.
- **Vérifier un robot** : `ssh $R 'hostname; uptime -s; systemctl is-active jupyterlab tailscaled funnel-watchdog; nmcli -t -f ACTIVE,SSID device wifi | grep ^yes; tailscale funnel status'`.
- **Redémarrer un robot à distance** sans couper sa propre connexion : `sudo systemd-run --on-active=5 /bin/systemctl reboot`. Les robots 2, 4, 5 et 6 sont revenus seuls en 1 à 2 minutes ; le robot 3 n'est pas revenu (cause inconnue au moment d'écrire).
- **Tension de la batterie** : `/opt/robot/venv/bin/python tools/battery.py` dans la copie du dépôt.

### 6.6 Mises à jour faites sur tous les robots

Faites sur les robots 1, 2, 4, 5 et 6 le 4 octobre, **à refaire sur le robot 3** dès qu'il revient :

1. Carnet vide `~etudiant/notebooks/mon_code.ipynb` (une cellule de code vide, noyau `python3`), propriétaire `etudiant`, jamais écrasé s'il existe.
2. Dossiers Music, Pictures, Public, Templates et Videos de `~etudiant` supprimés (s'ils sont vides), et désactivés dans `~etudiant/.config/user-dirs.dirs` (valeur `"$HOME/"`), avec `enabled=False` dans `~etudiant/.config/user-dirs.conf`.
3. Nouvelle version de la surveillance du Funnel (contrôle du nom public).

## 7. Cartes SD des robots 2 à 6 (clones de MobileRobot-1)

Le nécessaire est sur le **Dell**, dans `~/Programmation/Enseignement/cartes_sd/` (dossier privé, hors dépôt) :

- `copier_systeme.sh` : copie le système de MobileRobot-1 par SSH, en trois archives (`/opt`, `/usr`, le reste), refaites seulement si elles sont abîmées (utile en cas de coupure).
- `construire_image.sh` : construit `maitre.img` (11 Go, fichier creux, 7,5 Go de données) avec le même partitionnement que MobileRobot-1, en retirant ce qui lui est propre (clés SSH, identifiant de machine, état Tailscale, journaux, carnet d'essais d'Olivier, fichiers d'essais d'admin), avec le wifi du TP en priorité, sans la minuterie de bascule, et avec les services de premier démarrage. L'image a reçu depuis les mises à jour du [§ 6.6](#66-mises-à-jour-faites-sur-tous-les-robots).
- `preparer_carte.sh N /dev/mmcblk0` : écrit l'image (seulement les zones utiles), agrandit la partition système à toute la carte, puis fait de la carte le robot N : nom (`user-data`, `/etc/hostname`, `/etc/hosts`…), identifiant de machine neuf, mot de passe `etudiant` et JupyterLab du robot N, nom du robot dans la surveillance, fichier `/boot/firmware/robot.conf` avec la clé d'inscription Tailscale. Garde-fous : refuse un disque de plus de 40 Go, et une carte déjà préparée pour un autre robot (sauf `FORCER=1`) ; `SANS_ECRITURE=1` reprend après une écriture déjà faite. Une carte de 16 Go s'écrit en 3 à 10 minutes selon sa vitesse.
- `ajouter_cle.sh /dev/mmcblk0` : ajoute la clé Tailscale à une carte déjà préparée.
- `premier_demarrage/` : les services posés dans l'image.
- Les mots de passe, leurs empreintes et les clés Tailscale (voir [Secrets](#secrets)).

**Premier démarrage d'un clone** (rien à faire à la main) :

1. `robot-cles-ssh.service` crée les clés SSH propres au robot.
2. `robot-premier-demarrage.service` (`/usr/local/sbin/robot_premier_demarrage.sh`) attend Internet et l'heure, inscrit le robot dans Tailscale sous le nom `holobotN`, ouvre les deux Funnel, efface la clé et renomme `robot.conf` en `robot.conf.fait`. Journal : `/var/log/robot_premier_demarrage.log`. Sans Internet, il réessaie au démarrage suivant.

Le lecteur de cartes du Dell (Realtek RTS525A) refuse certaines cartes (« card never left busy state » dans `journalctl -k`) : en prendre une autre ou passer par un lecteur USB. Le bureau monte d'office les partitions d'une carte insérée ; le script les démonte.

## 8. Incidents rencontrés et solutions

- **PyLidar3** : décodage faux (points fantômes, angles mal corrigés) → notre pilote `holorobot.lidar`.
- **Double ouverture du bus moteurs** : pypot ne fait qu'un avertissement → verrou `flock` dans `holorobot.motors`.
- **Premier mouvement perdu après un changement de mode** : le moteur ignore les commandes pendant l'écriture en EEPROM → pause de 0,1 s.
- **Carte SD pleine en installant YOLO** : PyPI fournit pour ARM un PyTorch avec ~5 Go de CUDA → PyTorch CPU (`UV_TORCH_BACKEND=cpu` dans `install_robot_env.sh`).
- **Imports très lents** (environnement en lecture seule) → bytecode précompilé à l'installation.
- **Caméra qui donne 65 m** : étalonnage abîmé → restauration de l'étalonnage d'usine.
- **Funnel muet après un changement de réseau** (3 octobre vers 1 h, puis 4 octobre 17 h) → relance de `tailscaled`, désormais automatique.
- **Nom public non publié** (robots 3 et 6) → patience, relance de `tailscaled`, contrôle ajouté à la surveillance.
- **Coupures de courant** (3 et 4 octobre) : les Pi redémarrent et tout revient seul ; une copie en cours par le réseau est à refaire.
- **Curseur de position qui semblait ne pas marcher** : le noyau était bloqué pendant chaque trajet et les mouvements du curseur s'accumulaient → tâche de fond qui va à la dernière position demandée.

## 9. Conventions de travail

- Tout en **français** : code commenté, documentation, carnets, messages de commit.
- **numpy** plutôt que le module `math`.
- Avant de pousser : tests (`tests/`), relecture des carnets, et **recherche des secrets** dans ce qui part (mots de passe, `tskey-`, codes wifi). Dépôt et site sont publics.
- Messages de commit en français, au nom d'Olivier Ly, sans ligne d'attribution ajoutée.
- Le **site** peut être publié directement. Pour le **dépôt du code**, pousser après accord d'Olivier.
- Le dépôt du site reçoit aussi les commits automatiques du baromètre de Lacanau (depuis le Dell, à 1 h 25, 7 h 25, 13 h 25 et 19 h 25) : toujours `git pull --rebase` avant de pousser, et ne jamais laisser de commit local non poussé.
- **Moteurs** : ne les faire tourner qu'avec le robot sur cale (roues libres) ou au sol dans un espace dégagé, avec quelqu'un à côté. Avant chaque série de mouvements, vérifier où est le robot.
- Ne jamais écraser le travail des étudiants (carnets) sans sauvegarde.
- L'historique des deux dépôts a été réécrit le 3 octobre au soir (messages de commit nettoyés) : un clone plus ancien se resynchronise avec `git fetch && git reset --hard origin/main`.

## Secrets

Jamais dans le dépôt ni sur le site, et jamais recopiés dans un document partagé.

- **Mots de passe** des comptes `etudiant` (un par robot, il ouvre aussi JupyterLab) et `admin` (le même sur les six robots) : fichier privé `mots_de_passe_robots.txt` dans `~/Programmation/Enseignement/cartes_sd/` sur le Dell. **En emporter une copie privée pour le TP.** Le mot de passe d'un robot est donné à son groupe en séance.
- **Clés d'inscription Tailscale** : une par robot, toutes utilisées ; à révoquer dans la console si elles étaient réutilisables.
- **Code du wifi du TP** : sur le routeur, et dans les profils NetworkManager des robots.

## 10. À faire et questions ouvertes

- **Robot 3** : rallumer et vérifier ; puis lui appliquer le [§ 6.6](#66-mises-à-jour-faites-sur-tous-les-robots).
- **Console Tailscale** : désactiver l'expiration des clés de holobot2 à holobot6.
- **Seuil de batterie** : le site dit 10,4 V, le README et le tableau de bord disent 3,3 V par élément (9,9 V). À trancher et aligner.
- **Galets de MobileRobot-1** montés en O : échanger les roues avant et arrière pour un montage en X ?
- **Distances caméra / lidar** : 9 % d'écart, test au mètre ruban.
- **Sécurité du tailnet** : les étudiants ont un terminal sur les robots, qui sont dans le tailnet personnel d'Olivier. Un compte Tailscale dédié aux robots, ou des règles d'accès qui les isolent, serait plus sûr.
- **Moteurs des robots 2 à 6** : à brancher (sur le robot 2, aucun moteur ne répondait le 4 octobre) ; leurs identifiants et leurs sens restent à découvrir (`find_ids()`, `tools/wheel_test.py`).
- **Kit des cartes** : uniquement sur le Dell ; une version sans secrets pourrait rejoindre `setup/` dans le dépôt.
- **Modèles YOLO** : chaque carnet télécharge `yolo26n.pt` dans son dossier ; pas de dossier commun.
