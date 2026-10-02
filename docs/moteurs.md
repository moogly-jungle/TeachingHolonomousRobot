# Les moteurs : `holorobot.motors`

Chaque roue du robot est entraînée par un moteur **Dynamixel MX-12W**. Les moteurs sont chaînés sur un même câble, le **bus série** du robot (`/dev/serial0`), et chacun y a un numéro, son **identifiant** (ID). On commande les moteurs avec la bibliothèque pypot ; le module `holorobot.motors` la rend plus simple et ajoute une sécurité, le **chien de garde**.

```python
from holorobot.motors import Motors

with Motors() as motors:               # ouvre le bus, trouve les moteurs, les met en mode roue
    print(motors.ids)                  # par exemple [1, 2, 4, 8]
    motors.run({4: 90}, duration=2)    # le moteur 4 fait tourner sa roue à 90 °/s pendant 2 s
```

Le carnet **`decouverte_moteurs.ipynb`** (dossier `notebooks`) reprend tout ceci pas à pas, jusqu'au pilotage du robot.

## Notions

- **Identifiant (ID)** : chaque moteur répond à son numéro. Sur MobileRobot-1 : 4 avant gauche, 8 avant droite, 2 arrière gauche, 1 arrière droite. Sur un autre robot, les numéros peuvent être différents : on les découvre avec `scan()`.
- **Mode roue et mode articulation** : en mode articulation, un Dynamixel va à une position et s'y arrête (c'est le mode d'un bras robotique). En **mode roue**, il tourne sans fin à la vitesse demandée : c'est celui qu'il faut pour des roues. Le réglage est mémorisé par le moteur.
- **Vitesse** : en degrés par seconde (°/s). Une vitesse **positive** fait tourner la roue dans le sens inverse des aiguilles d'une montre, vue du côté de la roue. Comme les moteurs de gauche et de droite sont montés en miroir, une même vitesse positive fait avancer le robot d'un côté et le fait reculer de l'autre.
- **Vitesse maximale** : 720 °/s par défaut, soit 2 tours de roue par seconde. Une consigne plus forte est ramenée à cette limite, réglable avec `max_speed`.
- **Chien de garde** : si aucune consigne n'arrive pendant 0,5 s, toutes les roues s'arrêtent. Une consigne de `set_speeds()` ne vaut donc que 0,5 s. `run()` renvoie la consigne régulièrement ; dans vos propres boucles, renvoyez-la au moins 4 fois par seconde. C'est ce qui arrête le robot si votre programme plante ou si vous fermez le navigateur.
- **Un seul programme à la fois** peut utiliser le bus : un second programme, ou un second objet `Motors()`, est refusé avec un message d'erreur. Dans un carnet, créez l'objet `Motors()` une seule fois, et fermez-le à la fin (`motors.close()`), ou redémarrez le noyau.

## Exemples

### Découvrir les moteurs

```python
from holorobot.motors import scan

print(scan())          # identifiants qui répondent, de 0 à 20 : par exemple [1, 2, 4, 8]
```

### Initialiser

```python
from holorobot.motors import Motors

motors = Motors()                      # trouve les moteurs et les met en mode roue
print(motors.ids, motors.models)       # [1, 2, 4, 8] {1: 'MX-12', 2: 'MX-12', ...}
print(motors.is_wheel_mode())          # True
print(motors.get_voltages())           # tension de la batterie, lue par chaque moteur
```

`Motors(wheel_mode=False)` ne touche pas au mode ; `motors.set_wheel_mode()` le règle ensuite.

### Faire tourner une roue

```python
motors.run({4: 90}, duration=2)        # 90 °/s pendant 2 s, puis arrêt
```

### Faire tourner plusieurs roues en même temps

```python
motors.run({1: 90, 2: 90, 4: 90, 8: 90}, duration=2)
```

Toutes les vitesses sont positives : les roues de gauche avancent et celles de droite reculent, donc le robot tourne sur lui-même.

### Commander en continu

```python
import time

end = time.monotonic() + 3
while time.monotonic() < end:
    motors.set_speeds({4: 60, 8: -60})  # à renvoyer souvent : le chien de garde veille
    time.sleep(0.05)
motors.stop()
```

### Fermer

```python
motors.close()                         # arrête les roues, coupe le couple, libère le bus
```

## Référence

### `scan(ids=range(21), port="/dev/serial0", baudrate=57600)`

Renvoie la liste des identifiants qui répondent, sans rien faire bouger.

### `Motors(ids=None, port="/dev/serial0", baudrate=57600, wheel_mode=True, watchdog=0.5, max_speed=720)`

Ouvre le bus et cherche les moteurs : ceux de `ids`, ou de 0 à 20. Lève une `RuntimeError` si aucun moteur ne répond (sont-ils alimentés ?) ou si le bus est déjà utilisé, par ce programme ou par un autre.

| Attribut | Contenu |
|---|---|
| `ids` | identifiants des moteurs trouvés |
| `models` | modèle de chaque moteur |
| `watchdog` | délai du chien de garde, en s |
| `max_speed` | vitesse maximale autorisée, en °/s |

| Méthode | Effet |
|---|---|
| `is_wheel_mode()` | vrai si tous les moteurs sont en mode roue |
| `set_wheel_mode()` | met les moteurs en mode roue |
| `set_speeds({id: vitesse})` | vitesses en °/s, valables `watchdog` secondes ; les moteurs absents gardent leur consigne |
| `run({id: vitesse}, duration)` | fait tourner pendant `duration` s, puis arrête ; Ctrl-C ou *Interrupt Kernel* arrête aussi |
| `stop()` | arrête toutes les roues, qui freinent (couple actif) |
| `release()` | arrête et coupe le couple : les roues tournent librement à la main |
| `get_speeds()`, `get_positions()` | vitesses (°/s) et positions (degrés, de −180 à 180) mesurées |
| `get_temperatures()`, `get_voltages()` | températures (°C) et tension d'alimentation (V) |
| `close()` | arrête, coupe le couple et libère le bus |

`with Motors() as motors:` appelle `close()` en sortant, même en cas d'erreur.

## Piloter le robot

### Le repère du robot

![Repère du robot et vitesse d'un point](img/repere_robot.png)

On décrit le mouvement du robot dans son propre repère : **x vers l'avant** (côté caméra), **y vers la gauche**. Un robot holonome peut se déplacer dans n'importe quelle direction tout en tournant : son mouvement à un instant donné est un **vecteur vitesse** $\vec v = (v_x, v_y)$, en m/s, et une **rotation instantanée** $\omega$, en rad/s, positive dans le sens inverse des aiguilles d'une montre vu de dessus.

Un point $P$ du robot, de coordonnées $(x_P, y_P)$, par exemple le centre d'une roue, a alors pour vitesse

$$\vec v_P = (\,v_x - \omega\,y_P\,,\; v_y + \omega\,x_P\,)$$

C'est le point de départ du calcul des vitesses des roues.

### Roues holonomes (base à 3 roues)

![Base à 3 roues holonomes](img/holonome3.png)

Une roue holonome porte sur sa jante des galets libres, perpendiculaires à la roue : elle **pousse** le robot dans la direction où elle roule, $\vec d_i$, et le laisse **glisser** dans la direction de son axe. Seule compte donc la composante de $\vec v_{P_i}$ le long de $\vec d_i$ : la jante doit avancer à la vitesse $\vec d_i \cdot \vec v_{P_i}$, ce qui donne la vitesse de rotation de la roue de rayon $r$ :

$$\omega_i = \frac{\vec d_i \cdot \vec v_{P_i}}{r}$$

Avec les roues placées à la distance $R$ du centre, à l'angle $\theta_i$, et qui roulent tangentiellement, $\vec d_i = (-\sin\theta_i, \cos\theta_i)$. Développer ce calcul pour chaque roue, c'est l'objet du carnet.

### Roues mecanum (base à 4 roues)

![Base mecanum vue de dessus](img/mecanum.png)

Une roue mecanum roule vers l'avant comme une roue normale, mais ses galets sont inclinés à 45°. Le galet en contact avec le sol tourne librement : le robot peut glisser **perpendiculairement à l'axe du galet**, mais pas le long de cet axe, $\vec u_i$. La vitesse du point de contact par rapport au sol ne doit donc pas avoir de composante selon $\vec u_i$ :

$$(\vec v_{P_i} - r\,\omega_i\,\vec x)\cdot \vec u_i = 0 \quad\Longrightarrow\quad \omega_i = \frac{\vec v_{P_i}\cdot\vec u_i}{r\,(\vec x\cdot\vec u_i)}$$

Les roues sont en $(\pm l_x, \pm l_y)$. Pour que le robot tourne bien sur lui-même, les galets du dessus doivent dessiner un **X** vu de dessus : vérifiez-le sur votre robot. Le galet au contact du sol est symétrique de celui du dessus, d'où la direction de $\vec u_i$ pour chaque roue. Là encore, le développement roue par roue est fait dans le carnet.

### De la vitesse de la roue à la consigne du moteur

Les formules donnent $\omega_i$ en rad/s, positive quand la roue fait avancer le robot (ou le pousse selon $\vec d_i$). Pour le moteur, il faut :

1. convertir en °/s : `math.degrees(omega_i)` ;
2. multiplier par le **sens** du moteur, +1 ou −1, selon le côté où il est monté ;
3. l'envoyer à l'identifiant de la roue : `motors.set_speeds({id: vitesse})`.

Le carnet vous fait trouver les identifiants et les sens de votre robot, puis écrire la fonction de pilotage.

## Pièges courants

- **Radians ou degrés** : les formules sont en rad/s, les moteurs en °/s.
- **Le sens de chaque moteur** dépend de son montage : vérifiez-le roue par roue, robot sur une cale.
- **Galets en O au lieu de X** : si, vus de dessus, les galets du dessus dessinent un losange, le robot avance et recule normalement, mais part du mauvais côté et tourne mal, car ses roues poussent presque vers son centre. Échangez les roues avant et arrière de chaque côté.
- **Glissement** : sur un sol lisse, les roues mecanum patinent un peu ; le robot ne suit pas exactement la consigne. C'est normal, et c'est pour cela qu'on a un lidar et une caméra.
- **Chien de garde** : si les roues s'arrêtent toutes seules, votre boucle n'envoie pas la consigne assez souvent.
