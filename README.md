# Hexapode ECE

Robot hexapode (6 pattes, 18 servos Dynamixel AX-12A) du FabLab ECE MAKERS,
piloté par une Raspberry Pi 4 sous Ubuntu 22.04 / ROS 2 Humble. Ce dépôt
contient la carte électronique d'alimentation, le code de pilotage des servos,
le modèle et la marche sous ROS 2, et la documentation de mise en route.

## Démarrage rapide

Tout se lance depuis le PC (Ubuntu + ROS 2 Humble) avec **une seule commande** :

```bash
cd ~/Hexapode2026/Hexapode_ECE/ros2_ws
./hexapode.sh                   # simulation : marche + Foxglove + clavier (aucun servo ne bouge)
./hexapode.sh dry               # + pont vers le robot SANS envoi (vérifie servos et butées)
./hexapode.sh robot             # le robot se lève puis marche — PC et Pi reliés par CÂBLE
./hexapode.sh robot-distance    # pareil SANS CÂBLE : le PC rejoint le hotspot Wi-Fi de la Pi (voir hotspot.md)
./hexapode.sh dry-distance      # vérification sans câble
```

Le script règle l'environnement ROS (CycloneDDS, liaison câble ou Wi-Fi), arrête une
session oubliée, ouvre **Foxglove** (déjà connecté) et une fenêtre **« Teleop hexapode »**,
puis lance la marche et, selon le mode, le pont vers le robot. `robot-distance` allume en
au hotspot de la Pi (sauf si le câble est branché : il reste prioritaire), attend la Pi et
démarre `leg_server` dessus si besoin.

**Piloter** (clavier AZERTY, dans « Teleop hexapode ») :

| Touche | Effet |
|---|---|
| `i` / `,` | avancer / reculer (maintenir la touche) |
| `j` / `l` | tourner à gauche / à droite |
| Maj + `J` / `L` | crabe gauche / droite |
| `t` / `b` | monter / descendre le corps (doucement) |
| `k` | stop |

**Figures et remise en route** (fenêtre « Actions hexapode ») : `1` coucou, `2` danse,
`3` cabrer, `4` cœur, `0` arrêter ; **`r` remet les moteurs en route** quand ça bloque.

| `q` / `z` | vitesse +10 % / −10 % (max 0,20 m/s) |

**Arrêter** : Ctrl+C dans le terminal du lancement → le robot se fige, couple gardé.
Le relâcher : `nc 192.168.137.10 5555` (ou `10.42.0.10` en Wi-Fi) puis `ESTOP`.
Après `robot-distance`, Ctrl+C **ramène aussi le PC sur un Wi-Fi avec internet** (ou à tout
moment : `./hexapode.sh internet`).

Avant un essai sur le robot : batterie ≥ 11 V, robot posé au sol, quelqu'un à côté de
l'alimentation. Détails : [ros2_ws/src/hexapod_control/docs/lancement.md](ros2_ws/src/hexapod_control/docs/lancement.md).

## Index

| Document | Contenu |
|---|---|
| [setup.md](setup.md) | Mise en route de la Raspberry Pi : connexion SSH (`ssh hexapode`), réseau et partage de connexion par câble, VS Code, mises à jour, sauvegardes, synchronisation git PC ↔ Pi |
| [hotspot.md](hotspot.md) | **Sans câble** : hotspot Wi-Fi de la Pi, le PC s'y connecte (`ssh hexapode-wifi`, `hexapode.sh robot-distance`) ; câble prioritaire ; dépannage |
| [ros2_ws/README.md](ros2_ws/README.md) | Espace de travail ROS 2 : environnement, paquets, lancement, Foxglove |
| [ros2_ws/src/hexapod_imu/README.md](ros2_ws/src/hexapod_imu/README.md) | IMU : nœud `/imu/data` sur la Pi, calibration, dépannage |
| [PCB/Neuro_IMU/branchement.md](PCB/Neuro_IMU/branchement.md) | Carte IMU maison (LSM6DSV16X) : branchement I2C sur la Pi, brochage de la Pi 4 |
| [ros2_ws/src/hexapod_control/docs/](ros2_ws/src/hexapod_control/docs) | Nœuds ROS : [lancement](ros2_ws/src/hexapod_control/docs/lancement.md), [marche `gait_node`](ros2_ws/src/hexapod_control/docs/gait_node.md), [pont `leg_server_bridge`](ros2_ws/src/hexapod_control/docs/leg_server_bridge.md) |
| [Dyna_test/dyna.md](Dyna_test/dyna.md) | Servos Dynamixel : `leg_server` (Pi) et son protocole, interface graphique des 6 pattes et cinématique (PC), réglages des servos, port `/dev/u2d2` |
| [Dyna_test/test_torque.md](Dyna_test/test_torque.md) | `test_torque` : lever le robot (`--home` étoile de mer), options, essais |
| [Dyna_test/Torque_calcul.md](Dyna_test/Torque_calcul.md) | Calcul du couple demandé aux servos, essais du 25/11/2026, résultats et graphiques |
| [tps_archi.md](tps_archi.md) | Plan d'action ROS 2 : URDF, ros2_control, interface web, réseau ROS, capteurs, simulation et apprentissage par renforcement |
| [graphify-out/GRAPH_REPORT.md](graphify-out/GRAPH_REPORT.md) | Graphe de connaissances du dépôt (`graph.html` à ouvrir dans un navigateur) |

## Dossiers

| Dossier | Contenu |
|---|---|
| [`ros2_ws/`](ros2_ws) | ROS 2 : `hexapode.sh` (lancement), `hexapode_env.sh` (environnement), `hexapod_description` (URDF, maillages SolidWorks), `hexapod_control` (marche, pont vers le robot), `hexapod_bringup` (CycloneDDS) |
| [`Dyna_test/`](Dyna_test) | code Dynamixel : `src/leg_server.cpp`, `gui/` (interface, modèle, `test_torque`, calibration `leg_gui_config.json`, mesures), règle udev, réglages, docs, `graphs/` |
| [`PCB/`](PCB) | carte d'alimentation / Dynamixel (KiCad, Gerbers, fichiers JLCPCB) |
| [`graphify-out/`](graphify-out) | graphe de connaissances du dépôt (graphify) |

## Sans ROS (outils de calibration et d'essai)

```bash
ssh hexapode "cd ~/Hexapode_ECE/Dyna_test && (nohup ./bin/leg_server > /tmp/leg_server.log 2>&1 &)"   # sur la Pi
python3 Dyna_test/gui/leg_gui.py                    # interface des pattes, sur le PC
python3 Dyna_test/gui/test_torque.py --home         # lever / reposer le robot
```

Ne pas les utiliser en même temps que `hexapode.sh robot` (un seul programme doit
commander les servos).
