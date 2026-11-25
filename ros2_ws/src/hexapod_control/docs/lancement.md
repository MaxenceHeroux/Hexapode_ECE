# Lancer l'hexapode : `hexapod.launch.py`

Un seul fichier pour la simulation de la marche, avec ou sans le vrai robot.

```bash
cd ~/Hexapode2026/Hexapode_ECE/ros2_ws
source install/setup.zsh            # (setup.bash sous bash) ; ROS + hexapode_env.sh déjà dans ~/.zshrc
```

| Commande | Ce qui se lance |
|---|---|
| `ros2 launch hexapod_control hexapod.launch.py` | **Simulation** : URDF, marche, pont Foxglove |
| `… hexapod.launch.py rviz:=true` | + RViz (repère `odom`, pieds visés, odométrie) |
| `… hexapod.launch.py teleop:=true` | + un terminal de pilotage clavier |
| `… hexapod.launch.py robot:=dry` | + pont vers le robot **sans envoi** (vérifie la connexion, les servos, les butées) |
| `… hexapod.launch.py robot:=true` | + pont vers le robot : **LE ROBOT SE LÈVE PUIS MARCHE** |
| `… hexapod.launch.py robot:=true teleop:=true` | essai complet sur le robot |

## Arguments

| Argument | Défaut | Rôle |
|---|---|---|
| `robot` | `false` | `false` simulation seule, `dry` pont sans envoi, `true` le robot bouge |
| `rviz` | `false` | ouvre RViz |
| `foxglove` | `true` | `foxglove_bridge` sur le port 8765 (s'il est installé) |
| `teleop` | `false` | terminal `teleop_twist_keyboard` sur `/cmd_vel` |
| `host` | `192.168.137.10` | IP du `leg_server` |

## Nœuds lancés

| Nœud | Toujours | Doc |
|---|---|---|
| `robot_state_publisher` | oui | publie le modèle (URDF `hexapod_description`, `footprint:=false`) et le TF des pattes |
| `gait_node` | oui | [gait_node.md](gait_node.md) |
| `foxglove_bridge` | `foxglove:=true` | WebSocket `ws://<machine>:8765` |
| `rviz2` | `rviz:=true` | `config/gait.rviz` |
| `leg_server_bridge` | `robot:=dry` ou `true` | [leg_server_bridge.md](leg_server_bridge.md) |
| `teleop` | `teleop:=true` | dans une fenêtre « Teleop hexapode » |

`gait.launch.py` (inclus) ne lance que URDF + marche + Foxglove (+ RViz).

## Essai sur le robot, pas à pas

1. Sur la Pi : `ssh hexapode "pgrep -a leg_server"` (sinon le lancer, voir `Dyna_test/dyna.md` §2.1).
2. Batterie ≥ 11 V, robot posé au sol, quelqu'un à côté de l'alimentation.
3. `ros2 launch hexapod_control hexapod.launch.py robot:=dry` : regarder les messages du pont
   (servos présents, pas de « hors butées »), puis Ctrl+C.
4. `ros2 launch hexapod_control hexapod.launch.py robot:=true teleop:=true` : le robot se lève
   en 8 s (rampe), puis suit la marche.
5. Dans la fenêtre « Teleop hexapode » (clavier AZERTY) :

   | Touche | Effet |
   |---|---|
   | `i` / `,` | avancer / reculer (maintenir la touche) |
   | `j` / `l` | tourner à gauche / à droite |
   | Maj + `J` / `L` | crabe gauche / droite |
   | `k` (ou toute autre touche) | stop |
   | `a` / `z` | vitesse +10 % / −10 % |

6. Hauteur : `ros2 param set /gait_node body_height 0.12` (voir les couples rayon / hauteur
   dans [gait_node.md](gait_node.md)).
7. Arrêter : Ctrl+C dans le terminal du lancement → le pont **gèle** (couple gardé, robot
   debout). Pour le reposer : `nc 192.168.137.10 5555` puis `ESTOP` (il s'affaisse), ou
   `python3 Dyna_test/gui/test_torque.py` pour un lever / reposer contrôlé.

## Foxglove

*Open connection* → *Foxglove WebSocket* → `ws://localhost:8765` (simulation sur le PC)
ou `ws://192.168.137.10:8765` (si lancé sur la Pi). Panneaux utiles : **3D** (repère
`odom`), **Teleop** (`/cmd_vel`), **Parameters** (`/gait_node`), **Plot**
(`/joint_states`).
