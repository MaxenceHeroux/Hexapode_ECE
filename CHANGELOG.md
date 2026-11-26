# Historique des versions

## v1.0.0 — 26/11/2026 : l'hexapode marche sous ROS 2

Première version où le robot complet marche, se stabilise et se pilote sans fil sous
ROS 2 Humble.

### Robot et modèle
- **URDF** (`hexapod_description`) : corps + 6 pattes, maillages extraits du modèle
  SolidWorks (COLLADA, même rendu dans RViz et Foxglove), géométrie du CAD ;
  `legs.yaml` généré depuis la calibration `leg_gui` + butées des servos ;
  écart avec `hexapod_model.py` < 0,001 mm.
- **Calibration** versionnée (`Dyna_test/gui/leg_gui_config.json`) : positions des hanches
  et longueurs du CAD, sens des coxas vérifié sur le robot.

### Marche (`hexapod_control`)
- `/cmd_vel` → bouts de patte → cinématique inverse → `/joint_states`, à 50 Hz.
- Allures **trépied avec recouvrement des appuis** (défaut), tétrapode, onde.
- Cadence bornée par la longueur de pas ; vitesse max réelle 0,22 m/s, 0,8 rad/s ;
  accélération 0,6 m/s².
- Hauteur du corps 50–150 mm réglable en marchant (`t` / `b`, doux), écart des pieds
  adapté automatiquement aux butées (carte de faisabilité).
- Arrêt sans pas supplémentaire.

### Stabilisation par l'IMU
- Tenue de cap (avancer droit), mise à niveau active (corps horizontal sur sol incliné),
  anti-affaissement, arrêt si bascule > 35°.
- Rotation asservie au gyroscope : rotation réelle passée de **31 % à 65–74 %** de la
  consigne (avec le recouvrement des appuis).
- Compensation de la dérive en crabe (`drift_comp`).

### Figures
- **coucou, danse, cabrer, cœur** (`/action`, fenêtre « Actions hexapode »), entrée /
  sortie en douceur, testées sur le robot sans incident.

### Pont vers le robot (`leg_server_bridge`, provisoire avant ros2_control)
- Position **et vitesse** de chaque servo à chaque consigne (`GOALSPD`) : mouvements fluides.
- Compliance par articulation et par phase : raide au sol, souple en l'air.
- **Remise en route** des moteurs : automatique (servo bloqué ou en alarme) et à la
  main (touche `r`, service `/reset_servos`) ; mode « porté en l'air » (touche `p`).
- Robuste aux décrochages USB de l'U2D2 et aux pertes de paquets.
- `leg_server` : nouvelles commandes `GOALSPD` et `SYNCW`.

### IMU (`hexapod_imu`) et carte Neuro IMU
- Carte maison LSM6DSV16X (`PCB/Neuro_IMU`), branchement I2C et brochage de la Pi 4.
- Pilote, calibration (biais gyro, échelle, niveau, sens de montage), `/imu/data` à
  100 Hz avec orientation (filtre de Mahony).

### Réseau, lancement, outils
- CycloneDDS en unicast ; **hotspot Wi-Fi de la Pi** au démarrage, câble prioritaire.
- Services systemd sur la Pi : `leg_server` et IMU au démarrage.
- `hexapode.sh` : `sim`, `dry`, `robot`, `robot-distance`, `dry-distance`, `internet`
  (tout lancer en une commande, Foxglove et claviers compris).
- `test_torque --home` (étoile de mer), graphe de connaissances graphify.

### Connu / à faire
- Glissement résiduel des pieds (~30 % en rotation) : patins, zéro des coxas, entraxe
  fémur → genou à mesurer (66 mm CAD contre 75 mm ancienne calibration).
- AX-12A à leur limite (pics de charge à 95 % en figures, cabrage limité à −4,5°).
- Pas encore : plugin ros2_control, LiDAR / caméra, simulation et RL (voir `tps_archi.md`).
