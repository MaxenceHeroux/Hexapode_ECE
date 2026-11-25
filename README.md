# Hexapode ECE

Robot hexapode (6 pattes, 18 servos Dynamixel AX-12A) du FabLab ECE MAKERS,
piloté par une Raspberry Pi 4 sous Ubuntu 22.04 / ROS 2 Humble. Ce dépôt
contient la carte électronique d'alimentation, le code de pilotage des servos
et la documentation de mise en route.

## Index

| Document | Contenu |
|---|---|
| [setup.md](setup.md) | Mise en route de la Raspberry Pi : connexion SSH (`ssh hexapode`), réseau et partage de connexion, VS Code, mises à jour, sauvegardes, synchronisation git PC ↔ Pi |
| [Dyna_test/dyna.md](Dyna_test/dyna.md) | Servos Dynamixel : `leg_server` (Pi), interface graphique des 6 pattes et cinématique (PC), réglages des servos, port `/dev/u2d2`, `test_torque` |
| [Dyna_test/Torque_calcul.md](Dyna_test/Torque_calcul.md) | Calcul du couple demandé aux servos, essais du 25/11/2026, résultats et graphiques |
| [tps_archi.md](tps_archi.md) | Plan d'action pour passer sous ROS 2 : URDF, ros2_control, interface web, réseau ROS, capteurs, simulation et apprentissage par renforcement (à discuter) |

## Dossiers

| Dossier | Contenu |
|---|---|
| [`Dyna_test/`](Dyna_test) | code Dynamixel : `src/leg_server.cpp`, `gui/` (interface, modèle, `test_torque`, mesures), règle udev, réglages, `dyna.md`, `Torque_calcul.md`, `graphs/` |
| [`PCB/`](PCB) | carte d'alimentation / Dynamixel (KiCad, Gerbers, fichiers JLCPCB) |

## Démarrage rapide

```bash
ssh hexapode "cd ~/Hexapode_ECE/Dyna_test && ./bin/leg_server"   # sur la Pi
python Dyna_test/gui/leg_gui.py                                   # interface, sur le PC
```
