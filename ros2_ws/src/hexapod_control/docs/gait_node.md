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
  hauteur `height_rate` 1 cm/s) : pas d'à-coup quand on change la consigne ou la hauteur en marchant.
- **Arrêt** : sans `/cmd_vel` depuis `cmd_timeout` (0,5 s), la consigne passe à 0 ; le
  robot freine, repose les pattes qui étaient en l'air et s'arrête là (pas de pas
  supplémentaire pour revenir au neutre). Il ne fait des pas sur place que pour déplacer
  ses pieds après un changement de hauteur ou de `stance_radius`.
- Changer `stance_radius` à l'arrêt : le robot fait quelques pas sur place pour
  déplacer ses pieds.
- **Hors d'atteinte / hors butées** : les angles de la patte concernée sont figés et un
  avertissement est affiché (1 par seconde).

**Numérotation des pattes** : 1 avant gauche, 2 avant droite, 3 milieu droite,
4 arrière droite, 5 arrière gauche, 6 milieu gauche (IDs servo : voir
`hexapod_description/config/legs.yaml`).

---

## Figures : coucou, danse, cabrer, cœur

`/action` (`std_msgs/String`) : `coucou`, `danse`, `cabrer`, `coeur`, ou `stop`
([`actions.py`](../hexapod_control/actions.py)). Clavier : fenêtre **« Actions hexapode »**
(`ros2 run hexapod_control action_keys`, ouverte par `hexapode.sh`) : `1` coucou, `2` danse,
`3` cabrer, `4` cœur, `0`/espace stop, `r` remise en route des moteurs.

| Figure | Durée | Ce que fait le robot |
|---|---|---|
| `coucou` | 6 s | corps décalé vers l'arrière-droite, patte avant gauche levée de 10 cm, 3 allers-retours |
| `danse` | 10 s | pieds au sol, le corps se balance (cercle, roulis, tangage, lacet, haut-bas) |
| `cabrer` | 7 s | corps reculé de 3,5 cm et nez levé de 12°, les deux pattes avant en l'air qui battent |
| `coeur` | 9 s | patte avant gauche levée qui dessine 2 cœurs de ~7 cm en l'air |

- Entrée et sortie en douceur (1,2 s) ; la figure démarre quand la marche est arrêtée.
- `stop`, ou **toucher au clavier de marche**, arrête la figure en douceur (retour debout).
- Pendant une figure : pas de mise à niveau ; les pattes levées sont publiées « en l'air »
  sur `/leg_phases` (compliance souple), les autres « au sol ».
- Tests (`test/test_actions.py`) : toutes les poses atteignables et dans les butées à deux
  hauteurs ; arrêt en plein milieu sans saut (< 2 cm par pas de 20 ms), retour exact debout.

## Allures et glissement

| `gait` | Pieds au sol | `duty` par défaut | Usage |
|---|---|---|---|
| **`tripod`** (défaut) | 3, et 6 pendant le recouvrement | 0,60 | le plus efficace |
| `tetrapod` | 4 (paires 2-5, 3-6, 1-4) | 0,72 | plus d'appuis, mais moins bon en rotation sur ce robot |
| `wave` | 5 (une patte à la fois) | 0,86 | très stable, lent |

`duty` = part du cycle au sol. Au-dessus du minimum (0,5 / 0,67 / 0,83), les groupes se
**recouvrent** : la patte qui se pose est chargée avant que la suivante décolle. C'est ce
qui réduit le glissement : sans recouvrement, le robot passe un instant sur des pieds pas
encore chargés (retard des servos).

**Cadence** : le pas fait ~80 % de la longueur permise (`step_reach`) ; la période va de
`cycle_time` (vite) à 2 × `cycle_time` (lent), et les pas sont raccourcis de moitié
pendant un changement de hauteur.

Essai du 26/11/2026, rotation sur place commandée à 0,4 rad/s pendant 6 s (rotation réelle
mesurée à l'IMU, aide à la rotation active) :

| Réglage | Gauche | Droite |
|---|---|---|
| trépied sans recouvrement, coxas souples | 31 % | 32 % |
| + coxas raides au sol | 41 % | 37 % |
| + aide à la rotation par le gyroscope (`turn_assist`) | 36 % | 71 % |
| **+ trépied avec recouvrement (`duty` 0,6)** | **65 %** | **74 %** |
| tétrapode | 24 % | 72 % |
| onde | 26 % | 30 % |

Changer d'allure (appliquée à l'arrêt) : `ros2 param set /gait_node gait tetrapod`,
`ros2 param set /gait_node duty 0.65`.

## Stabilisation par l'IMU

Avec `/imu/data` (paquet `hexapod_imu`, sur la Pi), `gait_node` corrige la marche entre le
générateur de pas et la cinématique inverse ([`stabilizer.py`](../hexapod_control/stabilizer.py)) :

| Correction | Comment | Réglages |
|---|---|---|
| **Tourner pour de vrai** (`turn_assist`) | la rotation mesurée par le gyroscope est comparée à la rotation demandée ; la commande est augmentée (jusqu'à 2,5×, dans `max_angular`) pour rattraper le patinage des pieds | `turn_assist`, `turn_kp` 0,5, `turn_ki` 1,5, `turn_boost` 1,5 |
| **Dérive en crabe** | glissement latéral invisible pour l'IMU : vitesse latérale ajoutée = `drift_comp` × avance (mesuré : ~10 cm à gauche par mètre) | `drift_comp` −0,10 |
| **Avancer droit** (tenue de cap) | quand on ne demande pas de tourner, le cap au départ est mémorisé ; l'écart mesuré par l'IMU est corrigé par une petite rotation (PI, au plus 0,3 rad/s). Relâché dès qu'on tourne | `heading_hold`, `heading_kp` 1,2, `heading_ki` 0,2, `heading_max` |
| **Corps horizontal** (compensation active) | le roulis / tangage mesuré est compensé en inclinant le corps par rapport aux pieds (PI, ±10° max) : sol en pente ou qui bouge **lentement** → le haut reste droit. Si l'inclinaison demandée sort des butées, elle est réduite (½, puis 0) | `level`, `level_kp` 0,3, `level_ki` 3,0, `level_max_deg` 10 |
| **Ne plus descendre à chaque pas** | pendant qu'un trépied est en l'air, les pattes au sol poussent 3 mm plus bas (les servos plient sous la charge) | `sag_comp` 0,003 m |
| **Sécurité chute** | inclinaison > 35° : la marche s'arrête | `fall_deg` |
| Odométrie | le cap de `/odom` vient de l'IMU (plus juste que la consigne) | |

Sans IMU (nœud arrêté, plus de message depuis 0,5 s) : marche normale, inclinaison
ramenée doucement à plat ; message `plus d'IMU : marche sans stabilisation`.

Tests hors ROS (`test/test_stabilizer.py`) : sol incliné de 6° / −4° → corps ramené à
moins de 0,3° ; sol qui oscille de ±5° à 0,15 Hz → corps à moins de 2° ; dérive de cap de
0,05 rad/s → cap tenu à moins de 2° ; corps incliné de 8° → pieds atteignables. Essai du
26/11/2026 debout : roulis −0,01°, tangage 0,00° (au plus 0,15°).

Désactiver / régler en marche : `ros2 param set /gait_node level false`,
`ros2 param set /gait_node sag_comp 0.005`, `ros2 param set /gait_node heading_kp 2.0`.

## Entrées

| Topic | Type | Effet |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | `linear.x` avancer (+) / reculer, `linear.y` crabe (+ = gauche), `angular.z` tourner (+ = gauche), limités à `max_linear`, `max_angular` ; `linear.z` × `height_gain` = vitesse de montée / descente du corps (touches `t` / `b`) |
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
| `min_height`, `max_height` | 0,05 / 0,15 m | bornes de la hauteur (en plus de ce que les pattes permettent) | |
| `stance_radius` | 0,12 m | distance pied ↔ axe de hanche **voulue** au neutre (adaptée à la hauteur) | oui |
| `step_height` | 0,03 m | levée des pattes en vol | oui |
| `cycle_time` | 0,8 s | durée d'un cycle (2 pas) à vitesse max ; jusqu'à 2× plus lent à basse vitesse | oui |
| `step_reach` | 0,045 m | demi-pas max : pas jusqu'à 9 cm ; la consigne est réduite (même direction) si le pas dépasserait. Plus grand = pieds plus écartés (carte de faisabilité) | au démarrage |
| `max_linear`, `max_angular` | 0,20 m/s, 0,8 rad/s | vitesses max | oui |
| `cmd_timeout` | 0,5 s | arrêt sans consigne | |
| `height_gain` | 0,04 | `t` / `b` : vitesse de la hauteur = `linear.z` × 0,04 (≈ 4 mm/s à la vitesse de départ du clavier) | oui |
| `height_rate` | 0,01 m/s | vitesse max de changement de hauteur (toutes commandes) | oui |
| `rate` | 50 Hz | fréquence de calcul | |
| `legs_yaml` | (vide) | autre `legs.yaml` que celui de `hexapod_description` | |

**Hauteur et écart des pieds sont liés** (butées des tibias : pattes serrées = corps
haut). Au démarrage, le nœud calcule une carte des couples (rayon, hauteur) où la marche
reste dans les butées (6 pattes, pied neutre ± 2,5 cm au sol et en haut du pas). Ensuite :

- `body_height` est bornée à ce que les pattes permettent (50 – 150 mm) ;
- `stance_radius` est le rayon **voulu** : le rayon appliqué est le plus proche possible
  à la hauteur demandée (s'écarte quand on descend, se resserre quand on monte) ; le
  robot fait des pas sur place pour déplacer ses pieds ;
- pendant le changement, le corps ne monte / descend que si les pieds restent atteignables
  (il attend les pas), et un pied dont le pas passerait hors butées lève moins la patte.

| Hauteur demandée | Rayon appliqué (voulu 0,12 m) |
|---|---|
| 50 mm | 175 mm |
| 70 mm | 170 mm |
| 90 mm | 155 mm |
| 110 mm | 140 mm |
| 120 mm | 130 mm |
| ≥ 130 mm | 120 mm |

**Cadence** : `cycle_time` à vitesse max ; plus lente quand on va moins vite (jusqu'à
2 × `cycle_time` à mi-vitesse et en dessous) : pas plus courts **et** moins fréquents.

Pattes serrées = bras de levier plus court = moins de couple et moins de glissement
(essai du 25/11/2026 : 22 % de charge à 0,12 / 0,12 contre 35 % à 0,17 / 0,09).

---

## Commandes utiles

```bash
# Piloter au clavier (AZERTY : i avancer, , reculer, j/l tourner, Maj+J/L crabe, k stop)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.05 -p turn:=0.3
# vitesse : q / z = +/-10 % (w/x avance seule, e/c rotation seule) ; plafonnee a max_linear / max_angular

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
