# `hexapod_imu` — centrale inertielle (LSM6DSV16X)

Carte maison « Neuro IMU » ([PCB/Neuro_IMU/branchement.md](../../../PCB/Neuro_IMU/branchement.md))
en **I2C sur la Raspberry Pi** (bus 1, adresse **0x6A**). Le nœud tourne **sur la Pi**.

| Fichier | Rôle |
|---|---|
| `hexapod_imu/lsm6dsv16x.py` | pilote I2C (sans ROS) : accéléromètre ±4 g, gyroscope ±1000 °/s, 240 Hz, température |
| `hexapod_imu/calibration.py` | calibration, mise dans les axes du robot, filtre d'orientation (Mahony 6 axes) |
| `hexapod_imu/calibrate.py` | `ros2 run hexapod_imu calibrate_imu` |
| `hexapod_imu/imu_node.py` | nœud ROS `imu_node` |
| `config/imu_calib.yaml` | calibration en service (générée, à commiter) |

## Lancer

`hexapode.sh dry|robot|…-distance` le démarre tout seul sur la Pi. À la main, sur la Pi :

```bash
ssh hexapode
cd ~/Hexapode_ECE/ros2_ws && source install/setup.bash
ros2 launch hexapod_imu imu.launch.py            # rate:=200 possible
```

| Topic | Type | Contenu |
|---|---|---|
| `/imu/data_raw` | `sensor_msgs/Imu` | accélération (m/s²) et vitesse angulaire (rad/s) calibrées, **axes du robot** (x avant, y gauche, z haut) |
| `/imu/data` | `sensor_msgs/Imu` | idem + **orientation** (quaternion, filtre de Mahony) |
| `/imu/temperature` | `sensor_msgs/Temperature` | 1 Hz |

Repère `imu_link` (URDF, `use_imu:=true`, activé dans `gait.launch.py`) : axes alignés sur
`base_link`, position provisoire `(0, 0, 0,025)` m à corriger selon le montage réel
(`hexapod_description/urdf/sensors.xacro`).

**Le lacet (cap) n'a pas de référence** (pas de magnétomètre) : il dérive lentement.
Roulis et tangage sont recalés en permanence sur la gravité.

## Calibration

Sur la Pi, robot **posé à plat, immobile** :

```bash
ros2 run hexapod_imu calibrate_imu            # biais du gyro, échelle de l'accéléro, niveau
ros2 run hexapod_imu calibrate_imu --axes     # + sens de montage : soulever l'avant quand demandé
```

Écrit `config/imu_calib.yaml` dans les sources (à commiter, puis relancer le nœud).

Calibration du **26/11/2026** (29 °C, robot posé, 1246 mesures sur 10 s immobile) :

| Grandeur | Valeur |
|---|---|
| Biais du gyro (X, Y, Z capteur) | −0,234 / −0,201 / +0,133 °/s |
| Gravité mesurée | 9,928 m/s² → échelle 0,9878 |
| Montage | carte à plat, **avant du robot = −X du capteur** (lacet 180°) : avant soulevé → gravité sur −X |
| Niveau | carte inclinée de 1,0° (roulis) et 1,2° (tangage) par rapport au robot posé, corrigé |
| Vérification | au repos : roulis = tangage = lacet = 0,00° ; avant soulevé ~20° → tangage −19,5° (nez en haut = négatif, REP-103) |
| Bruit au repos | ~0,03 m/s², ~0,5 °/s (vibrations des servos) |

## Dépannage

| Symptôme | Cause / solution |
|---|---|
| `i2cdetect -y 1` ne montre pas `6a` | **couper l'alimentation de la carte quelques secondes** : un ancien réglage avait désactivé l'I2C du capteur (bit gardé tant qu'il est alimenté, constaté le 26/11/2026) ; sinon câblage, voir `branchement.md` |
| `WHO_AM_I = 0x..` au démarrage | ce n'est pas un LSM6DSV16X (`0x70`) ou mauvaise adresse (`address:=0x6b` si R4 montée) |
| `Permission denied` sur `/dev/i2c-1` | utilisateur dans le groupe `i2c` / `dialout` (fait), ou lancer avec `sudo` pour tester |
| Roulis / tangage pas à 0 robot posé à plat | refaire `calibrate_imu` |
| Le lacet tourne tout seul | normal sans magnétomètre (dérive lente) ; refaire la calibration si > 1 °/min à l'arrêt |
