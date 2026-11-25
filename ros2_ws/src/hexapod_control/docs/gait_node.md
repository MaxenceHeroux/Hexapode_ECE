# `gait_node` — marche en trépied

Transforme une consigne de vitesse (`/cmd_vel`) en positions des bouts de patte, puis
en angles des 18 articulations (cinématique inverse). C'est le « cerveau » de la marche :
il ne parle pas aux servos (voir [`leg_server_bridge`](leg_server_bridge.md)).

Code : [`hexapod_control/gait_node.py`](../hexapod_control/gait_node.py) (ROS),
[`gait.py`](../hexapod_control/gait.py) (générateur de pas, sans ROS),
[`kinematics.py`](../hexapod_control/kinematics.py) (cinématique, sans ROS).

```
/cmd_vel ─┐                    ┌─► /joint_states ──► robot_state_publisher ──► /tf (URDF)
/body_height ─► gait_node ─────┼─► /tf : odom → base_footprint → base_link
paramètres ─┘   (50 Hz)         ├─► /tf : odom → leg_<n>_foot_target
                               ├─► /foot_targets (PoseArray, odom)
                               └─► /odom
```

---

## Lancer

```bash
ros2 launch hexapod_control hexapod.launch.py          # recommandé (voir lancement.md)
ros2 run hexapod_control gait_node --ros-args --params-file \
    $(ros2 pkg prefix hexapod_control)/share/hexapod_control/config/gait.yaml   # seul
```

Seul, il faut aussi `robot_state_publisher` avec l'URDF en `footprint:=false`
(c'est ce que fait `gait.launch.py`).

---

## Comment il marche

**Position neutre** de chaque pied : dans l'axe du montage de la patte (coxa à 0°), à
`stance_radius` de l'axe de la hanche, à `body_height` sous les hanches.

**Trépieds** : A = pattes 1, 3, 5 (avant gauche, milieu droite, arrière gauche),
B = 2, 4, 6. Pendant un cycle de `cycle_time` secondes, A est au sol la première moitié
pendant que B est en l'air, puis l'inverse.

| Phase | Ce que fait le pied (dans le repère du corps) |
|---|---|
| **Appui** (au sol) | il recule à la vitesse `-(v + ω × p)` : il reste **fixe dans le monde**, c'est lui qui pousse le corps |
| **Vol** (en l'air) | il va de son point de décollage au point d'atterrissage (en avant du neutre d'un demi-pas), sur une cloche de hauteur `step_height` |

- Les vitesses et la hauteur suivent des **rampes** (accélération 0,2 m/s², 1 rad/s²,
  hauteur 3 cm/s) : pas d'à-coup quand on change la consigne ou la hauteur en marchant.
- **Arrêt** : sans `/cmd_vel` depuis `cmd_timeout` (0,5 s), la consigne passe à 0 ; le
  robot finit son pas, remet les pieds au neutre et s'immobilise.
- Changer `stance_radius` à l'arrêt : le robot fait quelques pas sur place pour
  déplacer ses pieds.
- **Hors d'atteinte / hors butées** : les angles de la patte concernée sont figés et un
  avertissement est affiché (1 par seconde).

**Numérotation des pattes** : 1 avant gauche, 2 avant droite, 3 milieu droite,
4 arrière droite, 5 arrière gauche, 6 milieu gauche (IDs servo : voir
`hexapod_description/config/legs.yaml`).

---

## Entrées

| Topic | Type | Effet |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | `linear.x` avancer (+) / reculer, `linear.y` crabe (+ = gauche), `angular.z` tourner (+ = gauche). Limité à `max_linear`, `max_angular` |
| `/body_height` | `std_msgs/Float64` | hauteur du corps (m), bornée à [`min_height`, `max_height`] |

## Sorties

| Topic / TF | Type | Contenu |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | 18 articulations `leg_<n>_<coxa\|femur\|tibia>_joint` (rad, convention `hexapod_model`) |
| TF `odom → base_footprint` | | position du robot (intégration de la vitesse appliquée) |
| TF `base_footprint → base_link` | | hauteur du corps |
| TF `odom → leg_<n>_foot_target` | | bout de patte visé, **dans le repère général** |
| `/foot_targets` | `geometry_msgs/PoseArray` | les 6 bouts de patte visés (repère `odom`) |
| `/odom` | `nav_msgs/Odometry` | pose et vitesse |

L'odométrie est la consigne intégrée (pas une mesure) : elle dérive si les pieds glissent.

---

## Paramètres (`config/gait.yaml`)

| Paramètre | Défaut | Rôle | Modifiable en marche |
|---|---|---|---|
| `body_height` | 0,12 m | hauteur des axes de hanche au-dessus du sol | oui |
| `min_height`, `max_height` | 0,05 / 0,13 m | bornes de la hauteur | |
| `stance_radius` | 0,12 m | distance pied ↔ axe de hanche au neutre | oui |
| `step_height` | 0,03 m | levée des pattes en vol | oui |
| `cycle_time` | 1,0 s | durée d'un cycle (2 pas) ; plus long = pas plus courts et plus lents | oui |
| `max_linear`, `max_angular` | 0,10 m/s, 0,6 rad/s | vitesses max | |
| `cmd_timeout` | 0,5 s | arrêt sans consigne | |
| `rate` | 50 Hz | fréquence de calcul | |
| `legs_yaml` | (vide) | autre `legs.yaml` que celui de `hexapod_description` | |

**Couples rayon / hauteur atteignables en marche** (butées des tibias, simulé dans toutes
les directions) :

| `stance_radius` | `body_height` minimale |
|---|---|
| 0,10 – 0,115 m | 0,13 m |
| 0,12 – 0,13 m | 0,12 m |
| 0,14 m | 0,11 m |
| 0,16 m | 0,09 m |
| 0,17 m | 0,07 m |

Pattes serrées = bras de levier plus court = moins de couple et moins de glissement
(essai du 25/11/2026 : 22 % de charge à 0,12 / 0,12 contre 35 % à 0,17 / 0,09).

---

## Commandes utiles

```bash
# Piloter au clavier (AZERTY : i avancer, , reculer, j/l tourner, Maj+J/L crabe, k stop)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.08 -p turn:=0.5

# Une consigne à la main (10 Hz, sinon arrêt au bout de 0,5 s)
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.08}}"
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.4}}"

# Hauteur et réglages pendant la marche
ros2 topic pub -1 /body_height std_msgs/msg/Float64 "{data: 0.12}"
ros2 param set /gait_node body_height 0.12
ros2 param set /gait_node stance_radius 0.13
ros2 param set /gait_node cycle_time 1.4
ros2 param dump /gait_node

# Observer
ros2 topic echo /odom --field pose.pose.position
ros2 run tf2_ros tf2_echo odom leg_1_foot_target
```

## Tests (sans ROS)

```bash
cd ros2_ws/src/hexapod_control && python3 -m pytest test/
```

Vérifient que la cinématique retrouve ses propres poses, que la marche (avant, arrière,
crabe, rotation, changement de hauteur) reste atteignable et dans les butées, que les
pieds en appui ne glissent pas dans le monde, et que le robot s'arrête pieds au neutre.
