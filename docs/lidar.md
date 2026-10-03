# Le lidar : `holorobot.lidar`

Le lidar est le capteur bleu qui tourne sur le dessus du robot. C'est un **YDLidar X4** : un laser tourne sur lui-même environ **7 fois par seconde** et mesure la distance des obstacles tout autour, dans un plan horizontal, de **12 cm à 10 m** environ. Chaque tour donne environ **500 points**.

Le module `holorobot.lidar` lit ces mesures en tâche de fond et vous donne un **scan** par tour : la liste des directions où le lidar a vu quelque chose, avec la distance correspondante.

```python
from holorobot.lidar import Lidar

with Lidar() as lidar:
    scan = lidar.get_scan()
    print(len(scan), "points")
    angle, distance = scan.nearest()
    print(f"obstacle le plus proche : {distance:.2f} m à {angle:.0f}°")
```

## Conventions

- **Angles** : en degrés, de 0 à 360 (360 exclu), comptés **dans le sens des aiguilles d'une montre** vu de dessus, à partir de la direction 0° du lidar.
- **Distances** : en **mètres**.
- **Directions sans mesure** : elles sont simplement **absentes** du scan, elles ne valent jamais 0. Le lidar ne mesure rien quand l'obstacle est trop près (moins de 12 cm), trop loin, ou quand la surface renvoie mal le laser (vitre, surface très noire ou très brillante).
- **Ordre des points** : ils sont rangés dans l'ordre où le lidar les a mesurés, pas par angle croissant à partir de 0°. Pour les trier : `sorted(scan.points())`.
- **Repère du robot** : le 0° du lidar n'est pas forcément l'avant du robot. Sur MobileRobot-1, le lidar est à l'arrière et la caméra à l'avant. La conversion dans le repère du robot viendra avec la suite de la bibliothèque.

## Exemples

### Obstacle le plus proche, en continu

```python
from holorobot.lidar import Lidar

with Lidar() as lidar:
    for scan in lidar.scans():  # un scan par tour, sans fin (Ctrl-C pour arrêter)
        angle, distance = scan.nearest()
        print(f"{distance:.2f} m à {angle:.0f}°")
```

### Distance minimale dans un secteur

Le secteur peut passer par 0°, par exemple de 350° à 10°.

```python
def min_distance(scan, start, end):
    """Plus petite distance (m) mesurée entre les angles start et end, ou None."""
    width = (end - start) % 360
    return min((d for a, d in scan.points() if (a - start) % 360 <= width), default=None)

with Lidar() as lidar:
    scan = lidar.get_scan()
    print("de 350° à 10° :", min_distance(scan, 350, 10))
```

### Coordonnées cartésiennes

`scan.xy()` donne les points en mètres, dans un tableau numpy de N lignes (x, y), avec x vers le 0° du lidar et y vers sa gauche :

```python
with Lidar() as lidar:
    for x, y in lidar.get_scan().xy()[:5]:
        print(f"x = {x:+.2f} m, y = {y:+.2f} m")
```

### Segments de droite : murs et meubles

`scan.segments()` renvoie les segments de droite du scan, du plus sûr au moins sûr. Chacun a une confiance entre 0 et 1, et l'équation cartésienne de sa droite :

```python
with Lidar() as lidar:
    scan = lidar.get_scan()

for s in scan.segments():
    if s.confidence < 0.5:
        break                              # la liste est triée : les suivants sont moins sûrs
    a, b, c = s.equation                   # a·x + b·y + c = 0
    print(f"{s.length:.2f} m, à {s.distance:.2f} m vu à {s.bearing:.0f}° (confiance {s.confidence:.2f}) : "
          f"{a:+.3f}·x {b:+.3f}·y {c:+.3f} = 0")
```

Les coordonnées sont celles de `xy()` : en mètres, x vers le 0° du lidar, y vers sa gauche. L'équation est normalisée : $a^2 + b^2 = 1$, $(a, b)$ va du lidar vers la droite et $c = -$distance. Ainsi, $a\,x + b\,y + c$ est directement la distance signée d'un point $(x, y)$ à la droite, négative du côté du lidar. Sur la Pi, il faut environ 10 ms par tour.

**La confiance** est le produit de trois notes, chacune entre 0 et 1 :

- **alignement** : les mesures sont-elles bien sur une droite ? $\exp(-\tfrac12 (\text{rms}/\sigma)^2)$, où rms est l'écart moyen des mesures à la droite et $\sigma$ le bruit attendu du lidar à cette distance (environ 1 cm à 1 m) ;
- **densité** : y a-t-il des trous ? Mesures présentes divisées par mesures attendues sur l'angle que couvre le segment ;
- **nombre** : assez de mesures ? $1 - \exp(-n/15)$ pour $n$ mesures : 8 mesures donnent 0,4 et 30 mesures 0,86.

**Comment ils sont trouvés** : le tour est coupé en amas là où deux mesures voisines sont trop loin l'une de l'autre ; chaque amas est scindé tant qu'un point s'écarte trop de la corde qui joint ses extrémités (c'est ce qui sépare deux murs à un coin) ; une droite est ajustée sur chaque morceau, puis les morceaux voisins alignés sont fusionnés.

### Dessiner un scan

```python
import matplotlib
matplotlib.use("Agg")  # pour enregistrer une image sans écran ; à retirer sur le bureau de la Pi
import matplotlib.pyplot as plt
import numpy as np
from holorobot.lidar import Lidar

with Lidar() as lidar:
    scan = lidar.get_scan()

ax = plt.figure().add_subplot(projection="polar")
ax.set_theta_zero_location("N")  # 0° en haut
ax.set_theta_direction(-1)       # sens des aiguilles d'une montre, comme le lidar
ax.scatter(np.radians(scan.angles), scan.distances, s=2)
plt.savefig("scan.png")
```

L'outil `tools/lidar_snapshot.py` fait la même chose en plus complet : `python3 tools/lidar_snapshot.py --image lidar.png`.

### Avec des threads

Le pilote lit déjà le lidar dans **son propre thread**, en continu. Vous pouvez donc appeler `get_scan()` ou `latest()` depuis n'importe quel thread, même plusieurs à la fois. Un programme lent ne fait pas prendre de retard aux mesures : chaque appel donne le tour le plus récent, jamais un vieux tour resté en attente.

Exemple : une boucle de commande à 20 Hz qui consulte le dernier tour sans jamais se bloquer.

```python
import time
from holorobot.lidar import Lidar

with Lidar() as lidar:
    while True:
        scan = lidar.latest()                 # immédiat ; un tour arrive environ toutes les 0,14 s
        angle, distance = scan.nearest()
        if distance < 0.3:
            print("obstacle proche !")
        time.sleep(0.05)
```

Si un thread arrête le lidar (`stop()` ou `close()`) pendant qu'un autre attend dans `get_scan()`, ce dernier est réveillé tout de suite par une `RuntimeError`.

### Travailler sans robot

Le dépôt contient un enregistrement de 8 secondes du lidar de MobileRobot-1 (`tests/data/x4_raw.bin`). `decode()` le transforme en scans, sur n'importe quel ordinateur et même sans pyserial :

```python
from holorobot.lidar import decode

scans = decode(open("tests/data/x4_raw.bin", "rb").read())
print(len(scans), "tours ; le premier a", len(scans[0]), "points")
```

## Référence

### `Lidar(port="/dev/ttyUSB0")`

L'ouverture lit les informations du lidar et vérifie qu'il répond. Elle lève une `RuntimeError` si le port ne s'ouvre pas, par exemple parce qu'un autre programme l'utilise, ou si le lidar ne répond pas.

| Attribut | Contenu |
|---|---|
| `model` | code du modèle : 6 pour un X4 |
| `firmware` | version du micrologiciel, par exemple `"1.10"` |
| `hardware` | version du matériel |
| `serial_number` | numéro de série (16 chiffres) |
| `health` | état : `"bon"`, `"avertissement"` ou `"erreur"` |
| `error_code` | code d'erreur donné par le lidar (0 si tout va bien) |

| Méthode | Effet |
|---|---|
| `start()` | Lance le moteur et la lecture en tâche de fond, puis attend le premier tour complet. Lève `TimeoutError` si rien n'arrive en 5 s. |
| `get_scan(timeout=2.0)` | Attend le **prochain** tour complet et le renvoie : les données sont toujours fraîches, jamais un tour déjà vu. Lève `TimeoutError` si rien n'arrive à temps, `RuntimeError` si le lidar a cessé de répondre (câble débranché…). |
| `latest()` | Renvoie **sans attendre** le dernier tour complet reçu, qui peut avoir déjà été renvoyé ; `None` au tout début. Pratique dans une boucle de commande qui ne doit pas se bloquer. |
| `scans()` | Les tours successifs, sans fin : `for scan in lidar.scans(): ...` |
| `stop()` | Arrête la lecture et le moteur. Le lidar reste ouvert, `start()` peut le relancer. |
| `close()` | Arrête tout et libère le port. |

`with Lidar() as lidar:` appelle `start()` en entrant et `close()` en sortant, **même en cas d'erreur ou de Ctrl-C**. C'est la façon recommandée de s'en servir.

### `Scan`

Un tour complet. Il ne change plus une fois créé : on peut garder plusieurs scans dans une liste sans risque.

| Élément | Contenu |
|---|---|
| `angles` | tuple des angles, en degrés |
| `distances` | tuple des distances correspondantes, en mètres |
| `timestamp` | instant de la fin du tour (`time.time()`) ; `None` pour un enregistrement |
| `len(scan)` | nombre de points |
| `points()` | liste de couples `(angle, distance)` |
| `nearest()` | `(angle, distance)` du point le plus proche, ou `None` si le scan est vide |
| `xy()` | tableau numpy de N lignes `(x, y)`, en mètres : x vers le 0° du lidar, y vers sa gauche |
| `segments(min_points=8, min_length=0.2, split=0.03)` | segments de droite, du plus sûr au moins sûr : voir ci-dessous |

### `Segment`

Un segment de droite trouvé dans un scan. Coordonnées en mètres, dans le repère de `xy()`.

| Élément | Contenu |
|---|---|
| `start`, `end` | extrémités `(x, y)` |
| `points` | nombre de mesures qui le composent |
| `rms` | écart quadratique moyen des mesures à la droite, en m |
| `confidence` | confiance, de 0 à 1 : alignement × densité × nombre |
| `length`, `midpoint` | longueur (m) et milieu `(x, y)` |
| `angle` | orientation du segment, en degrés dans [0, 180), dans le sens des aiguilles d'une montre depuis le 0° du lidar |
| `distance`, `bearing` | distance la plus courte du lidar à la droite (m), et direction de ce point, en degrés comme les angles du scan |
| `equation` | `(a, b, c)` tels que a·x + b·y + c = 0, avec a² + b² = 1 et c = −distance |

Options de `segments()` (et de `find_segments(points, …)`, qui travaille sur une liste de `(angle, distance)`) : `min_points`, nombre minimal de mesures ; `min_length`, longueur minimale en m ; `split`, écart à la droite au-delà duquel un morceau est scindé (en m, plus 1 cm par mètre de distance).

### Fonctions de bas niveau

Elles ne servent que pour comprendre le pilote ou rejouer des données brutes :

- `decode(octets)` : liste des scans complets contenus dans un enregistrement brut.
- `parse_packets(octets, start=0)` : découpe un flux en paquets valides ; renvoie `(paquets, position)`.
- `packet_points(paquet)` : points `(angle, distance)` d'un paquet, sans les mesures nulles.
- `angle_correction(distance_mm)` : correction d'angle du X4, en degrés.
- `TurnAssembler` : regroupe les points des paquets en tours.

## Pièges courants

- **Un seul programme à la fois** peut utiliser le lidar. Si un autre programme l'occupe, l'ouverture échoue.
- **Utilisez `with`** : il garantit que le moteur s'arrête et que le port est libéré.
- **Un scan dure environ 0,14 s** (un tour). Si le robot bouge vite pendant ce temps, le scan est un peu déformé.
- **Le premier tour après le démarrage est écarté**, car le moteur n'est pas encore à sa vitesse. `start()` l'attend pour vous.
- **Des directions manquent** toujours un peu : rien vu, trop près, vitre… C'est normal. Ne supposez pas 360 points réguliers.
- **Segments : un même mur peut en donner plusieurs**, s'il est en partie caché par un meuble ou coupé par une porte. À l'inverse, une rangée d'objets alignés peut former un segment. Un segment qui revient d'un tour à l'autre est plus sûr que sa seule confiance ne le dit.
- **Les segments sont dans le repère du lidar**, pas dans celui du robot : pour passer de l'un à l'autre, il faut connaître la position et l'orientation du lidar sur votre robot.

## Comment ça marche, pour les curieux

Le lidar est branché en USB par un petit adaptateur (puce CP2102) qui crée un port série, `/dev/ttyUSB0`, à 128 000 bauds. La ligne DTR de ce port commande le moteur.

- **Commandes** : deux octets envoyés au lidar. `A5 60` démarre le scan, `A5 65` l'arrête, `A5 90` demande les informations de l'appareil, `A5 91` son état.
- **Paquets de mesures** : pendant le scan, le lidar envoie des paquets de la forme `AA 55 | CT | LSN | FSA | LSA | CS | échantillons`.
  - `LSN` donne le nombre d'échantillons, jusqu'à 40 par paquet.
  - `FSA` et `LSA` donnent l'angle du premier et du dernier échantillon (valeur / 2 / 64 = degrés).
  - `CS` est une somme de contrôle (OU exclusif de tous les mots de 16 bits), qui permet d'écarter un paquet abîmé.
  - Chaque échantillon vaut 4 fois la distance en millimètres, et 0 veut dire « pas de mesure ».
- **Angles** : ceux des échantillons intermédiaires sont répartis régulièrement entre FSA et LSA. Chaque angle reçoit ensuite une correction qui dépend de sa distance, parce que l'émetteur laser et le capteur ne sont pas au même endroit : jusqu'à −8° au loin. C'est la formule de `angle_correction`.
- **Tours** : un paquet spécial (bit 0 de `CT`) marque le début de chaque tour. Sur le X4, il arrive vers 347°, pas à 0°.
- **Lecture** : un fil d'exécution (thread) lit le port en continu et range les points par tour. `get_scan()` attend simplement le tour suivant.

Le décodage suit le protocole officiel (manuel de développement YDLIDAR et pilote YDLidar-SDK). Les tests (`python3 tests/test_lidar.py`) rejouent l'enregistrement du dépôt et vérifient que le pilote donne exactement les points d'un décodeur de référence. Sur la Pi, le pilote occupe environ 4 % d'un cœur.

## Pourquoi pas PyLidar3 ?

PyLidar3 est la bibliothèque Python qu'on trouve habituellement pour ce lidar. Elle est boguée : elle fait la moyenne des directions sans mesure (comptées comme 0) avec les vraies distances, corrige mal les angles et mélange plusieurs tours. Sur les mêmes données, 40 % de ses points sont des fantômes, contre 0,5 % avec un décodage conforme au protocole. D'où notre propre pilote.
