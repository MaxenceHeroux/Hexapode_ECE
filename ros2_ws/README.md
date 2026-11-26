# `ros2_ws` — espace de travail ROS 2 de l'hexapode

Plan d'ensemble : [`../tps_archi.md`](../tps_archi.md).

## ▶ Tout lancer en une commande

```bash
~/Hexapode2026/Hexapode_ECE/ros2_ws/hexapode.sh          # simulation (aucun servo ne bouge)
~/Hexapode2026/Hexapode_ECE/ros2_ws/hexapode.sh dry      # + pont vers le robot, sans envoi (vérification)
~/Hexapode2026/Hexapode_ECE/ros2_ws/hexapode.sh robot    # + pont : LE ROBOT SE LÈVE PUIS MARCHE (confirmation demandée)
~/Hexapode2026/Hexapode_ECE/ros2_ws/hexapode.sh robot-distance   # pareil SANS CÂBLE : hotspot Wi-Fi de la Pi (voir ../hotspot.md)
```

Lance : environnement ROS (CycloneDDS), URDF, marche (`gait_node`), pont Foxglove **et
l'application Foxglove déjà connectée** (`ws://localhost:8765`), fenêtre de pilotage
clavier « Teleop hexapode », et selon le mode le pont vers le robot. Arrête d'abord une
éventuelle session oubliée (deux marches en même temps = robot incohérent, port 8765 pris).
Arguments en plus passés au lancement : `./hexapode.sh robot rviz:=true`.
**Ctrl+C** arrête tout (sur le robot : gel, couple gardé ; `ESTOP` via `nc` pour le relâcher).

Clavier (AZERTY, dans « Teleop hexapode ») : `i` avancer, `,` reculer, `j`/`l` tourner,
Maj+`J`/`L` crabe, `t`/`b` monter/descendre le corps, `k` stop, `q`/`z` vitesse +/−10 % (`w`/`x` avance seule,
`e`/`c` rotation seule). Le terminal du lancement affiche la vitesse reçue et la hauteur. Hauteur : `ros2 param set /gait_node body_height 0.12`.

## Environnement (PC et Pi)

```bash
source /opt/ros/humble/setup.bash        # (setup.zsh sous zsh)
source ~/.../Hexapode_ECE/ros2_ws/hexapode_env.sh
```

`hexapode_env.sh` règle `ROS_DOMAIN_ID=42`, CycloneDDS en unicast PC
(`192.168.137.1`) ↔ Pi (`192.168.137.10`) (`src/hexapod_bringup/config/cyclonedds.xml`)
et charge `install/` s'il existe.

Compiler : `cd ros2_ws && colcon build --symlink-install`

## Paquets

| Paquet | Rôle |
|---|---|
| `hexapod_description` | URDF/xacro, maillages SolidWorks, `config/legs.yaml` (généré depuis la calibration de `leg_gui`) |
| `hexapod_control` | Marche en trépied : `/cmd_vel` → bouts de patte → cinématique inverse → `/joint_states` |
| `hexapod_imu` | IMU LSM6DSV16X (carte Neuro IMU, I2C sur la Pi) → `/imu/data` ; calibration ([README](src/hexapod_imu/README.md)) |

## Voir le modèle

```bash
ros2 launch hexapod_description display.launch.py      # RViz + curseurs des 18 articulations
```

Après une recalibration (`leg_gui_config.json`) :

```bash
python3 src/hexapod_description/scripts/gen_legs_yaml.py --host 192.168.137.10   # + butées des servos
python3 src/hexapod_description/scripts/check_fk.py                               # URDF = hexapod_model ?
```

## Lancement complet (recommandé)

```bash
ros2 launch hexapod_control hexapod.launch.py                          # simulation + Foxglove
ros2 launch hexapod_control hexapod.launch.py robot:=dry               # + pont vers le robot, sans envoi
ros2 launch hexapod_control hexapod.launch.py robot:=true teleop:=true # LE ROBOT BOUGE + clavier
```

Docs détaillées : [`src/hexapod_control/docs/`](src/hexapod_control/docs/)
([lancement](src/hexapod_control/docs/lancement.md),
[gait_node](src/hexapod_control/docs/gait_node.md),
[leg_server_bridge](src/hexapod_control/docs/leg_server_bridge.md)).

## Marche (simulation, aucun servo ne bouge)

```bash
ros2 launch hexapod_control gait.launch.py              # + foxglove_bridge (port 8765) s'il est installé
ros2 launch hexapod_control gait.launch.py rviz:=true   # avec RViz
```

| Entrée | Effet |
|---|---|
| `/cmd_vel` (`geometry_msgs/Twist`) | avancer / reculer (`linear.x`), crabe (`linear.y`), tourner (`angular.z`) ; max 0,10 m/s et 0,6 rad/s ; arrêt si plus de consigne depuis 0,5 s |
| `/body_height` (`std_msgs/Float64`) | hauteur du corps (m, axes des hanches au-dessus du sol, 0,05–0,13), changée en douceur même en marchant |
| paramètres de `/gait_node` | `body_height`, `stance_radius`, `step_height`, `cycle_time` (modifiables en marche) |

Sorties : `/joint_states` (18 articulations), TF `odom → base_footprint → base_link`,
`odom → leg_<n>_foot_target` (bouts de patte visés dans le repère général),
`/foot_targets` (`PoseArray`, repère `odom`), `/odom`.

Réglages : `src/hexapod_control/config/gait.yaml`. Tests hors ROS :
`cd src/hexapod_control && python3 -m pytest test/`.

### Sur le vrai robot (pont provisoire, avant la phase P2)

`leg_server_bridge` envoie `/joint_states` au `leg_server` de la Pi (TCP 5555) :

```bash
ros2 launch hexapod_control gait.launch.py                                   # marche
ros2 run hexapod_control leg_server_bridge                                   # SIMULATION : n'envoie rien
ros2 run hexapod_control leg_server_bridge --ros-args -p send:=true -p ramp_time:=8.0   # LE ROBOT BOUGE
```

- Démarrage : 18 servos présents, aucun en alarme ; compliance 8/100 (fémurs, tibias) ;
  consigne = position actuelle puis couple ON ; **rampe** de `ramp_time` s vers la pose de marche
  (le robot se lève, toutes les pattes ensemble).
- **Gel** (plus de consigne, couple gardé) : consigne hors butées, servo absent / en alarme /
  trop chaud, charge > `max_load` (85 %) plus de 0,6 s, ou Ctrl+C. Couper le couple :
  `nc 192.168.137.10 5555` puis `ESTOP`.
- Essai du 25/11/2026 : lever en 8 s (charge max 54 %), marche OK ; pattes serrées à
  120 mm, corps à 120 mm → charge ~22 % à l'arrêt (pics ~83 % pendant certains pas).

### Foxglove

Sur la Pi, le pont est installé (`ros-humble-foxglove-bridge`). Dans Foxglove (application
ou app.foxglove.dev) : *Open connection* → *Foxglove WebSocket* → `ws://192.168.137.10:8765`.

- **3D** : repère fixe `odom`, le robot (maillages via `package://`) et les `leg_<n>_foot_target`.
- **Teleop** : topic `/cmd_vel` → les flèches font marcher le robot.
- **Publish** : topic `/body_height`, type `std_msgs/msg/Float64`, `{"data": 0.11}` → hauteur.
- **Parameters** : `/gait_node` → `body_height`, `step_height`, `cycle_time`…

Le clavier marche aussi : `ros2 run teleop_twist_keyboard teleop_twist_keyboard`.
