# `hexapod_control` — locomotion de l'hexapode

| Doc | Contenu |
|---|---|
| [docs/lancement.md](docs/lancement.md) | **`hexapod.launch.py`** : simulation, pont vers le robot, teleop, Foxglove |
| [docs/gait_node.md](docs/gait_node.md) | nœud de marche : `/cmd_vel` → bouts de patte → `/joint_states` |
| [docs/leg_server_bridge.md](docs/leg_server_bridge.md) | pont `/joint_states` → `leg_server` (vrai robot), sécurités |

```bash
ros2 launch hexapod_control hexapod.launch.py                         # simulation
ros2 launch hexapod_control hexapod.launch.py robot:=true teleop:=true # sur le robot
```
