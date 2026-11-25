# Architecture cible : l'hexapode sous ROS 2

Ce document est le **plan d'action** pour faire passer l'hexapode sous ROS 2 :
un modèle complet du robot (URDF), un pilotage par une interface web servie par
la Raspberry Pi et par le réseau ROS, et un environnement prêt pour ajouter des
capteurs et faire de l'apprentissage par renforcement (RL).

> **Statut : proposition à discuter.** Rien n'est encore implémenté. Les points
> marqués **[À décider]** sont listés au §9 avec une recommandation.

Documents liés : [setup.md](setup.md) (Pi, réseau, SSH),
[Dyna_test/dyna.md](Dyna_test/dyna.md) (servos, serveur et interface actuels),
[Dyna_test/Torque_calcul.md](Dyna_test/Torque_calcul.md) (couple et essais).

---

## Sommaire

1. [Objectifs](#1-objectifs)
2. [Point de départ](#2-point-de-départ)
3. [Contraintes techniques](#3-contraintes-techniques)
4. [Architecture cible](#4-architecture-cible)
5. [Paquets ROS 2](#5-paquets-ros-2)
6. [Interfaces ROS (topics, services, repères)](#6-interfaces-ros-topics-services-repères)
7. [Réseau, interface web, capteurs](#7-réseau-interface-web-capteurs)
8. [Simulation et apprentissage par renforcement](#8-simulation-et-apprentissage-par-renforcement)
9. [Décisions à prendre](#9-décisions-à-prendre)
10. [Phases du projet](#10-phases-du-projet)
11. [Risques](#11-risques)
12. [Correspondance avec l'existant](#12-correspondance-avec-lexistant)
13. [Glossaire](#13-glossaire)

---

## 1. Objectifs

| # | Objectif | Ce que ça veut dire concrètement |
|---|---|---|
| O1 | **Tout sous ROS 2** | Le robot se pilote uniquement par des topics, services et actions ROS 2 Humble. Plus de protocole maison entre les programmes. |
| O2 | **URDF complet** | Un modèle du robot (géométrie, articulations, butées, masses, inerties) utilisé à la fois par RViz, le contrôle, la simulation et le RL. Une seule source de vérité. |
| O3 | **Interface web servie par la Pi** | Ouvrir un navigateur (PC, téléphone) sur l'adresse de la Pi pour voir le robot en 3D, le faire bouger, lire l'état des servos, et l'arrêter en urgence. |
| O4 | **Pilotage par le réseau ROS** | Un PC sous ROS 2 sur le même réseau voit les topics du robot (RViz, clavier, scripts, politique RL). |
| O5 | **Environnement extensible** | Ajouter un capteur = l'accrocher dans l'URDF + lancer son pilote. Conventions ROS standard (REP-103, REP-105) partout. |
| O6 | **Prêt pour le RL** | Une simulation fidèle au robot réel, un environnement d'apprentissage, et un moyen de faire tourner la politique entraînée sur la Pi. |

---

## 2. Point de départ

Ce qui existe et fonctionne au 25/11/2026 (détails dans [dyna.md](Dyna_test/dyna.md)) :

| Élément | État |
|---|---|
| Raspberry Pi 4 | Ubuntu 22.04, **ROS 2 Humble desktop installé**, accessible en `ssh hexapode` (câble Ethernet, partage de connexion Windows) |
| Servos | 18 × AX-12A (protocole 1.0, 1 Mbaud) sur un U2D2, nom fixe `/dev/u2d2` |
| Disposition | 6 pattes × (coxa, fémur, tibia) ; IDs, sens, offsets et montage connus et sauvegardés |
| `leg_server` (C++) | Seul programme qui parle aux servos : SyncWrite, lecture d'état, reconnexion automatique de l'U2D2, reset du mode sécurité, réglages au démarrage, plusieurs clients TCP |
| `hexapod_model.py` | Configuration des pattes, cinématique directe et inverse (tibia cintré), **vérifiée sur 12 000 poses** |
| `leg_gui.py` | Interface PC : vue des 6 pattes, réglages, calibration, cinématique inverse |
| `test_torque.py` | Lever complet à 6 pattes **réussi** (49 poses, 17 % de charge moyenne des fémurs debout) |
| Mesures | Couple, charge, température, tension enregistrés ; graphiques ([Torque_calcul.md](Dyna_test/Torque_calcul.md)) |
| CAD | Modèle SolidWorks *Hexapod Bas1ma Haz1r* (assemblages, pièces, STL) |

Ces éléments ne sont pas jetés : ils sont **portés** dans les paquets ROS 2
(voir §12).

---

## 3. Contraintes techniques

Ces contraintes dictent plusieurs choix du plan.

### 3.1 Les AX-12A limitent la fréquence de la boucle de contrôle

- En protocole 1.0, les AX-12A **n'ont pas de lecture groupée** (ni SyncRead
  ni BulkRead) : il faut interroger les servos **un par un**.
- Mesure actuelle : lire l'état complet (adresses 0–49) des 18 servos prend
  **environ 40 ms**. En ne lisant que la position (2 octets), on devrait
  descendre vers **~20 ms** (à confirmer en phase P2).
- L'écriture des consignes, elle, reste groupée (SyncWrite, un paquet pour 18
  servos).

**Conséquence :** boucle de contrôle réelle de l'ordre de **25 à 50 Hz**. La
politique RL devra être entraînée à cette fréquence, et les lectures lentes
(température, tension, charge) seront faites en tâche de fond à plus basse
fréquence (par exemple 1 servo par cycle).

### 3.2 Un seul maître sur le bus des servos

Le bus série ne supporte pas deux programmes qui envoient en même temps. Le
pilote ROS (`hexapod_hardware`) **remplace** `leg_server` pendant qu'il tourne.
On garde `leg_server` comme outil de secours et de calibration, mais jamais
lancé en même temps que ROS.

### 3.3 La Pi 4 ne sert qu'au robot réel

Une Pi 4 n'a ni la puissance ni la carte graphique pour la simulation 3D ou
l'entraînement. **La simulation et le RL se font sur le PC** ; la Pi exécute
le contrôle, les pilotes de capteurs et la politique déjà entraînée (un petit
réseau de neurones s'exécute sans problème en temps réel sur la Pi).

### 3.4 Couple disponible

Les mesures montrent que le robot tient debout confortablement à 6 pattes avec
les bouts de patte près du corps (17 % du couple max), mais qu'**une patte
seule ne peut pas le lever** et que la marche en trépied, pattes écartées, est
à la limite ([Torque_calcul.md](Dyna_test/Torque_calcul.md) §2.3). Le contrôle
et le RL doivent garder les bouts de patte dans une zone raisonnable, et la
simulation doit connaître ces limites.

### 3.5 Réseau

Le PC et la Pi sont reliés par un câble Ethernet, la Pi obtient son adresse par
le partage de connexion Windows. La découverte automatique de ROS 2 (DDS, par
multicast) passe mal sur ce type de lien et sous WSL. Il faudra configurer le
DDS en **unicast** avec les adresses des machines (§7.1).

---

## 4. Architecture cible

```
┌────────────────────────────── Raspberry Pi 4 (robot réel) ──────────────────────────────┐
│                                                                                          │
│   Navigateur ──HTTP──► serveur web (page de pilotage)                                    │
│   (PC / tél.)  ──WS──► foxglove_bridge et/ou rosbridge ─┐                                │
│                                                         ▼                                │
│  /cmd_vel, /body_pose ──► hexapod_control ──► /leg_controller/commands                   │
│  (web, clavier, RL)        démarche, IK,           │                                     │
│                            lever / s'asseoir        ▼                                    │
│                                         controller_manager (ros2_control)               │
│                                           ├─ joint_state_broadcaster ──► /joint_states   │
│                                           └─ leg_controller (positions des 18 servos)   │
│                                                         │                                │
│                                         hexapod_hardware (plugin C++)                   │
│                                           SyncWrite, lectures, reset, reconnexion        │
│                                                         │                                │
│   robot_state_publisher ◄── URDF          capteurs (IMU, LiDAR, caméra…)                 │
│        └─► /tf, /robot_description           └─► /imu/data, /scan, /camera/…            │
└─────────────────────────────────────────────────────────┼────────────────────────────────┘
                                                          │ /dev/u2d2 (USB)
                                                     U2D2 ─── 18 × AX-12A

            ◄═══════════ réseau ROS 2 (DDS, câble Ethernet) ═══════════►

┌──────────────────────────────────── PC (ROS 2) ─────────────────────────────────────────┐
│  RViz (robot, capteurs)   téléopération (clavier, manette)   Foxglove Studio            │
│  simulation (même URDF, mêmes contrôleurs)     entraînement RL → politique exportée     │
└──────────────────────────────────────────────────────────────────────────────────────────┘
```

Principes :

- **Une seule description du robot** (URDF/xacro) pour le robot réel, la
  simulation, RViz et le RL.
- **ros2_control** sépare le contrôle du matériel : les mêmes contrôleurs
  pilotent soit le vrai robot (`hexapod_hardware`), soit la simulation, sans
  changer le reste.
- **Le robot fonctionne seul** : tout ce qui est nécessaire pour marcher et
  être piloté depuis le web tourne sur la Pi. Le PC est optionnel.
- **Sécurité au plus près du matériel** : l'arrêt d'urgence, le délai de
  sécurité (watchdog) et les butées sont gérés dans `hexapod_hardware`, pas
  dans l'interface.

---

## 5. Paquets ROS 2

Espace de travail dans le dépôt : `ros2_ws/src/` (les dossiers `build/`,
`install/` et `log/` sont déjà exclus par le `.gitignore`).

### 5.1 `hexapod_description` : le modèle du robot

| Contenu | Détail |
|---|---|
| `urdf/hexapod.urdf.xacro` | Le robot complet : corps, puis 6 × une macro `leg` |
| `urdf/leg.xacro` | Macro d'une patte : paramètres `prefix`, position et angle de montage, côté (gauche/droite) ; 3 articulations `revolute` (coxa, fémur, tibia) |
| `urdf/sensors.xacro` | Emplacements de capteurs (IMU, LiDAR, caméra), activables par argument |
| `urdf/ros2_control.xacro` | Déclaration des articulations pour ros2_control, avec un argument `use_sim` (matériel réel ou simulation) |
| `meshes/` | Maillages simplifiés issus de SolidWorks (STL, réduits pour l'affichage) |
| `config/legs.yaml` | IDs, sens, offsets, butées de chaque servo (repris de `leg_gui_config.json`) |
| `launch/display.launch.py` | Affichage dans RViz avec curseurs d'articulations (`joint_state_publisher_gui`) |

Points clés :

- **Nommage** : `leg_<n>_<coxa|femur|tibia>_joint`, et liens
  `leg_<n>_<coxa|femur|tibia>_link`, `leg_<n>_foot_link`.
- **Butées** : issues des butées EEPROM des servos (déjà symétrisées), en
  radians.
- **Masses et inerties** : à partir de SolidWorks (propriétés de masse) ou
  calculées sur des formes simples, puis **recalées sur la masse pesée** du
  robot.
- **Tibia cintré** : modélisé tel quel (courbure −45°, décalage du bout), pour
  que la cinématique ROS donne le même résultat que `hexapod_model`.
- **Repère du corps** : `base_link` au centre du corps, x vers l'avant, y vers
  la gauche, z vers le haut (REP-103), comme dans `hexapod_model`.

### 5.2 `hexapod_hardware` : le pilote des servos (C++)

Plugin `hardware_interface::SystemInterface` de ros2_control, qui reprend tout
ce que `leg_server` sait faire :

| Fonction | Origine dans `leg_server` |
|---|---|
| Ouvrir `/dev/u2d2` à 1 Mbaud, rouvrir après un décrochage USB | `open_port`, `ensure_port` |
| `write()` : envoyer les 18 consignes en un SyncWrite | `sync_goals` |
| `read()` : lire les positions (rapide), et en tâche de fond la charge, la température, la tension, les alarmes | `state_json` |
| Conversions angle ↔ position servo (sens, offset) | `hexapod_model.raw_to_joint` / `joint_to_raw` |
| Réglages au démarrage (pente, punch, vitesse, limite de couple) | `leg_settings.txt` |
| Reset du mode sécurité | `reset_servo` |
| Couple coupé à l'arrêt, servos absents gérés | `torque_all`, `present` |

Interfaces exposées :

- **Commande** : `position` pour chaque articulation (en radians).
- **État** : `position`, `effort` (charge), et à basse fréquence
  `temperature` / `voltage` publiés comme diagnostics.

À évaluer d'abord : le paquet communautaire `dynamixel_hardware`. S'il gère
correctement le protocole 1.0 et les particularités des AX (pas de lecture
groupée, mode sécurité), on l'utilise ; sinon, plugin maison à partir du code
de `leg_server` (le plus probable).

### 5.3 `hexapod_bringup` : lancement et configuration

| Fichier | Rôle |
|---|---|
| `launch/robot.launch.py` | Lance tout sur la Pi : description, ros2_control, contrôleurs, contrôle, pont web, capteurs |
| `config/controllers.yaml` | `joint_state_broadcaster`, `leg_controller` (positions), fréquence de boucle |
| `config/servo_settings.yaml` | Réglages des servos (remplace `leg_settings.txt`) |
| `systemd/hexapod.service` | Démarrage automatique au boot de la Pi (le robot reste sans couple tant qu'on ne le lève pas) |

### 5.4 `hexapod_control` : locomotion

| Nœud / fonction | Rôle |
|---|---|
| Cinématique | Portage de `hexapod_model` (IK / FK des pattes, repère corps ↔ patte) |
| `stand` / `sit` (actions) | Portage de `test_torque` : rapprocher les pattes par trépieds, se lever en attendant que chaque pas soit atteint, se rasseoir |
| Démarche | Démarche en trépied pilotée par `/cmd_vel` (vitesses x, y et rotation) ; d'autres démarches (ondulante, tétrapode) ensuite |
| Pose du corps | `/body_pose` : hauteur, roulis, tangage, lacet du corps, pattes au sol |
| Sécurité | Limite les bouts de patte à la zone où le couple suffit (§3.4) |

### 5.5 `hexapod_msgs` : messages propres au robot

| Type | Contenu |
|---|---|
| `ServoState.msg` | ID, position, charge, température, tension, couple ON, alarmes |
| `ServoStateArray.msg` | Les 18 |
| `ResetSafety.srv` | Reset du mode sécurité (un servo ou tous) |
| `SetServoSettings.srv` | Pente, punch, marge, vitesse, limite de couple |
| `Stand.action` / `Sit.action` | Se lever / s'asseoir, avec progression et résultat |

### 5.6 `hexapod_web` : interface web servie par la Pi

Voir §7.2.

### 5.7 `hexapod_sim` et `hexapod_rl`

Voir §8.

---

## 6. Interfaces ROS (topics, services, repères)

### 6.1 Topics

| Topic | Type | Sens | Rôle |
|---|---|---|---|
| `/robot_description` | `std_msgs/String` | Pi → | URDF |
| `/joint_states` | `sensor_msgs/JointState` | Pi → | Positions et efforts des 18 articulations |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | Pi → | Repères |
| `/servo_states` | `hexapod_msgs/ServoStateArray` | Pi → | État détaillé des servos (basse fréquence) |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Pi → | Alarmes, température, tension, reconnexions de l'U2D2 |
| `/cmd_vel` | `geometry_msgs/Twist` | → Pi | Vitesse de marche |
| `/body_pose` | `geometry_msgs/Pose` | → Pi | Pose du corps pattes au sol |
| `/leg_controller/commands` | `std_msgs/Float64MultiArray` | → Pi | Consignes directes des 18 articulations (RL, tests) |
| `/imu/data` | `sensor_msgs/Imu` | Pi → | IMU |
| `/scan` | `sensor_msgs/LaserScan` | Pi → | LiDAR |
| `/camera/image_raw` | `sensor_msgs/Image` | Pi → | Caméra |

### 6.2 Services et actions

| Nom | Type | Rôle |
|---|---|---|
| `/estop` | `std_srvs/Trigger` | Arrêt d'urgence : couple coupé partout |
| `/reset_safety` | `hexapod_msgs/ResetSafety` | Sortie du mode sécurité |
| `/servo_settings` | `hexapod_msgs/SetServoSettings` | Réglages des servos |
| `/stand`, `/sit` | `hexapod_msgs/Stand`, `Sit` | Se lever, s'asseoir |

### 6.3 Arbre des repères (REP-105)

```
map ── odom ── base_footprint ── base_link ─┬─ leg_<n>_coxa_link ── leg_<n>_femur_link ── leg_<n>_tibia_link ── leg_<n>_foot_link   (× 6)
                                            ├─ imu_link
                                            ├─ laser_link
                                            └─ camera_link
```

`odom` sera d'abord estimé par la cinématique des pattes (puis fusionné avec
l'IMU), `map` viendra plus tard avec le LiDAR (SLAM).

---

## 7. Réseau, interface web, capteurs

### 7.1 Réseau ROS 2

| Réglage | Valeur proposée | Pourquoi |
|---|---|---|
| Implémentation DDS | **CycloneDDS** (`rmw_cyclonedds_cpp`) | Simple à configurer en unicast, robuste |
| Découverte | **Unicast**, adresses des pairs listées (`192.168.137.10` pour la Pi, `192.168.137.1` pour le PC) | Le multicast passe mal par le partage de connexion Windows et sous WSL |
| `ROS_DOMAIN_ID` | Une valeur fixe (ex. `42`) | Ne pas se mélanger avec d'autres robots ROS du FabLab |
| Fichier | `hexapod_bringup/config/cyclonedds.xml`, chargé par `CYCLONEDDS_URI` sur la Pi et le PC | Même configuration des deux côtés |

Test de validation : `ros2 topic echo /joint_states` sur le PC pendant que le
robot tourne.

### 7.2 Interface web servie par la Pi

Deux niveaux, complémentaires :

| | **Foxglove** (via `foxglove_bridge`) | **Page de pilotage maison** (via `rosbridge`) |
|---|---|---|
| Rôle | Mise au point : 3D du robot (URDF), courbes, topics, diagnostics | Pilotage simple, utilisable sur téléphone |
| Contenu | Tout ce que publie le robot | Joystick virtuel (`/cmd_vel`), hauteur du corps, boutons se lever / s'asseoir / **arrêt d'urgence**, état des servos (charge, température, alarmes), vue 3D simple |
| Effort | Très faible : un nœud à lancer | Une page HTML/JavaScript (bibliothèque `roslib.js`), servie par un petit serveur web sur la Pi |
| Accès | Foxglove (application ou web) se connecte à `ws://192.168.137.10:8765` | `http://192.168.137.10:8080` dans n'importe quel navigateur |

L'**arrêt d'urgence** doit être visible en permanence et marcher même si le
reste de la page est figé (appel direct du service `/estop`).

### 7.3 Ajouter un capteur

Règle : **un capteur = un emplacement dans l'URDF + un pilote ROS standard +
une ligne dans le lancement.** Rien d'autre à toucher.

1. Ajouter un lien (`<capteur>_link`) et sa position dans `urdf/sensors.xacro`,
   activable par un argument (`use_imu:=true`).
2. Lancer son pilote ROS 2 publiant un message standard (`sensor_msgs/...`)
   avec `frame_id` = ce lien.
3. L'ajouter à `robot.launch.py` (conditionnel), et à la simulation (§8) avec
   un capteur simulé équivalent.

Capteurs prévus :

| Capteur | Intérêt | Priorité |
|---|---|---|
| **IMU** (BNO085 ou équivalent, I2C) | Équilibre, orientation du corps, **indispensable pour le RL** | Haute |
| **LiDAR LD** (déjà utilisé en ROS 2 sur ce robot) | Cartographie, évitement d'obstacles | Moyenne |
| **Caméra** (déjà utilisée via v4l2) | Vision, téléopération | Moyenne |
| **Contacts au pied** | Savoir quelles pattes touchent le sol ; à défaut, estimer par la charge du tibia | Moyenne |
| Tension de la batterie | Déjà lue par les servos ; à publier en diagnostic | Haute (gratuit) |

---

## 8. Simulation et apprentissage par renforcement

### 8.1 Simulation

- **Même URDF, mêmes contrôleurs** : en simulation, `hexapod_hardware` est
  remplacé par le plugin de simulation de ros2_control (`use_sim:=true`).
- **Modèle des servos AX-12A** : une simple consigne de position idéale ne
  suffit pas. Il faut reproduire ce qu'on a mesuré :

| Caractéristique | Valeur à reproduire | Source |
|---|---|---|
| Couple max | 1,5 N·m à 12 V, proportionnel à la tension | doc ROBOTIS, [Torque_calcul.md](Dyna_test/Torque_calcul.md) §2.2 |
| Souplesse | Pente 8 / punch 100, écart ~1 unité sous charge | Essai 1 |
| Vitesse max | Réglage `SPEED` | `leg_settings` |
| Fréquence et délai | 25–50 Hz, délai de lecture mesuré en P2 | §3.1 |
| Mise en sécurité | Couple coupé si surcharge prolongée | Essai 2 |

### 8.2 Environnement RL (à préciser avec toi)

| Élément | Proposition |
|---|---|
| Interface | Environnement **Gymnasium** standard |
| Action | Consigne de position des 18 articulations (ou écart à une démarche de référence, plus facile à apprendre), à la fréquence réelle (§3.1) |
| Observation | Positions des 18 articulations, orientation et vitesses angulaires (IMU), commande de vitesse demandée, action précédente ; éventuellement contacts des pieds |
| Récompense | Suivre la vitesse demandée ; pénaliser l'énergie (charge des servos), les chocs, le corps trop bas ou incliné ; forte pénalité si chute |
| Fin d'épisode | Chute, corps au sol, articulation en butée trop longtemps |

### 8.3 Du simulateur au vrai robot (sim2real)

1. **Variations aléatoires en simulation** (domain randomization) : masse,
   frottements, couple des servos, délais, bruit des capteurs.
2. **Validation** : même politique, comportement comparé en simulation et sur
   le robot réel, à partir des mesures (mêmes topics enregistrés en `rosbag`).
3. **Exécution sur la Pi** : la politique est exportée (format ONNX) et
   exécutée par un nœud `hexapod_rl/policy_node` qui publie sur
   `/leg_controller/commands`.
4. **Garde-fous** : le nœud de politique passe par les mêmes limites de
   sécurité que `hexapod_control` (zone de travail, butées, arrêt
   d'urgence).

---

## 9. Décisions à prendre

| # | Question | Options | Recommandation |
|---|---|---|---|
| D1 | **ROS 2 côté PC** | Ubuntu en dual boot / WSL2 / Windows natif | **Ubuntu en dual boot** (22.04 + Humble) : le plus fiable pour ROS, la simulation et le réseau |
| D2 | **Interface web** | Foxglove / page maison / les deux | **Les deux** : Foxglove tout de suite pour la mise au point, la page maison pour piloter |
| D3 | **Simulateur et RL** | Gazebo Fortress / MuJoCo / Isaac Lab | **Dépend de ta carte graphique** : Isaac Lab si carte NVIDIA puissante, sinon **MuJoCo** pour le RL ; Gazebo Fortress pour vérifier l'intégration ROS |
| D4 | **Modèle 3D** | Export automatique SolidWorks → URDF / xacro écrit à la main | **xacro écrit à la main** (propre, paramétrable) **avec les maillages SolidWorks pour l'affichage**, masses issues de SolidWorks puis recalées par pesée |
| D5 | **Matériel** | IMU, contacts au pied, AX-18F pour les fémurs | **IMU indispensable** avant le RL ; contacts souhaitables ; AX-18F à envisager selon les essais de marche |
| D6 | **Outils actuels** | Garder `leg_server` / `leg_gui` ou tout basculer | **Les garder** comme outils de secours et de calibration (jamais en même temps que ROS) |
| D7 | **Pilote des servos** | `dynamixel_hardware` communautaire / plugin maison | **Évaluer** le communautaire en début de P2, plugin maison sinon |

Les décisions **D1, D2, D3 et D5** conditionnent l'environnement et le
matériel : ce sont les premières à trancher.

---

## 10. Phases du projet

Ordre conseillé : **P0 → P1 → P2 → P3**, puis **P4 et P5 en parallèle**,
puis **P6**. La simulation peut démarrer dès P1, mais ne devient fiable
qu'avec les mesures de P2.

### P0 — Décisions et préparation

- Trancher D1 à D7.
- Peser le robot (masse totale, et si possible une patte seule).
- Installer ROS 2 Humble sur le PC ; configurer CycloneDDS des deux côtés.
- Récupérer dans SolidWorks les propriétés de masse des pièces et exporter
  les maillages simplifiés.
- Créer `ros2_ws/` dans le dépôt.

**Terminé quand :** le PC et la Pi voient mutuellement leurs topics
(`ros2 topic list`, `ros2 run demo_nodes_cpp talker` / `listener` entre les
deux machines).

### P1 — Description du robot (URDF)

- Écrire `hexapod.urdf.xacro`, la macro `leg`, les capteurs optionnels.
- Reporter IDs, sens, offsets, butées dans `legs.yaml`.
- Vérifier la cinématique : pour une série de poses, comparer la position des
  bouts de patte calculée par ROS (TF) et par `hexapod_model`.

**Terminé quand :** le robot s'affiche correctement dans RViz, les
articulations bougent dans le bon sens avec les bonnes butées, et l'écart avec
`hexapod_model` est **inférieur à 2 mm**.

### P2 — Pilote des servos (ros2_control)

- Évaluer `dynamixel_hardware`, sinon écrire `hexapod_hardware` à partir de
  `leg_server`.
- Contrôleurs `joint_state_broadcaster` et `leg_controller`.
- Arrêt d'urgence, watchdog (couple coupé ou maintien si plus de commande
  depuis N ms), reset de sécurité, diagnostics.
- **Mesurer la fréquence de boucle atteinte** (positions seules, puis avec
  l'état complet en tâche de fond).

**Terminé quand :** `ros2 control list_hardware_interfaces` montre les 18
articulations ; le robot réel bouge depuis `rqt` ; `/estop` coupe le couple ;
la fréquence mesurée est notée dans ce document.

### P3 — Locomotion

- Porter la cinématique et la séquence de lever (`stand` / `sit`).
- Démarche en trépied sur `/cmd_vel` ; pose du corps sur `/body_pose`.
- Enregistrer les essais (`rosbag`) et les comparer aux mesures de
  `test_torque`.

**Terminé quand :** le robot **se lève, marche en avant, recule, tourne** sur
commande, sans mise en sécurité des servos.

### P4 — Interface web et réseau ROS

- `foxglove_bridge` sur la Pi, mise en page Foxglove enregistrée dans le dépôt.
- Page de pilotage maison (joystick, hauteur, se lever / s'asseoir, arrêt
  d'urgence, état des servos).
- Démarrage automatique (service systemd).
- Téléopération depuis le PC (clavier / manette).

**Terminé quand :** on pilote le robot **depuis un téléphone** sur la page de
la Pi, **et** depuis le PC via le réseau ROS.

### P5 — Capteurs

- IMU (pilote, emplacement URDF, calibration), publiée sur `/imu/data`.
- LiDAR et caméra (pilotes déjà utilisés auparavant sur ce robot).
- Odométrie : cinématique des pattes + IMU (`robot_localization`).

**Terminé quand :** chaque capteur apparaît au bon endroit dans RViz, et un
capteur supplémentaire s'ajoute en suivant uniquement la procédure du §7.3.

### P6 — Simulation et RL

- Simulation avec le même URDF et les mêmes contrôleurs, modèle de servo
  AX-12A calé sur les mesures.
- Environnement Gymnasium, entraînement sur le PC.
- Export de la politique, nœud d'exécution sur la Pi, essais progressifs sur
  le robot (suspendu d'abord, puis au sol).

**Terminé quand :** une politique entraînée en simulation fait marcher le vrai
robot à la fréquence mesurée en P2.

---

## 11. Risques

| Risque | Effet | Parade |
|---|---|---|
| Fréquence de boucle limitée par les AX (§3.1) | Contrôle et RL moins réactifs | Lire seulement les positions dans la boucle, le reste en tâche de fond ; entraîner à la fréquence réelle ; à terme, servos XL/XM (protocole 2.0, lecture groupée) |
| Décrochages de l'U2D2 sous charge (observés) | Perte de contrôle passagère | Reconnexion automatique dans `hexapod_hardware` (déjà au point dans `leg_server`) ; câblage (masse, câble USB court, ferrite) |
| Couple insuffisant en trépied pattes écartées | Mises en sécurité des fémurs | Zone de travail limitée dans le contrôle ; pattes près du corps ; AX-18F si besoin |
| Écart simulation / robot réel (servos « mous ») | Politique RL qui ne marche pas sur le robot | Modèle de servo calé sur les mesures ; variations aléatoires ; validation par `rosbag` |
| Réseau ROS par le partage de connexion | Topics invisibles entre PC et Pi | CycloneDDS en unicast, `ROS_DOMAIN_ID` fixe, test dès P0 |
| Batterie faible (9,7 V mesurés) | Couple réduit, comportements différents | Publier la tension, alerte sous un seuil, recharge avant les essais |
| Charge mal répartie entre pattes (offsets) | Une patte force plus que les autres | Recalibration des offsets avant P3 ([Torque_calcul.md](Dyna_test/Torque_calcul.md) §3.4) |

---

## 12. Correspondance avec l'existant

| Aujourd'hui | Sous ROS 2 |
|---|---|
| `leg_server` (TCP, port 5555) | `hexapod_hardware` (plugin ros2_control) + services `/estop`, `/reset_safety`, `/servo_settings` |
| Commandes `GOALSID`, `STATE` | Topics `/leg_controller/commands`, `/joint_states`, `/servo_states` |
| `leg_settings.txt` | `hexapod_bringup/config/servo_settings.yaml` |
| `leg_gui_config.json` (pattes, IDs, sens, offsets) | `hexapod_description/config/legs.yaml` + URDF |
| `hexapod_model.py` (cinématique) | URDF (géométrie) + `hexapod_control` (IK / démarche) ; gardé comme référence de validation (P1) |
| `leg_gui.py` | Foxglove + page web de pilotage (et `leg_gui` gardé pour la calibration hors ROS) |
| `test_torque.py` | Actions `/stand`, `/sit` + enregistrements `rosbag` |
| `gui/logs/`, `plot_torque.py` | `rosbag` + scripts d'analyse (mêmes graphiques) |
| Démarrage manuel de `leg_server` | Service systemd `hexapod.service` |

---

## 13. Glossaire

| Terme | Sens |
|---|---|
| **ROS 2** | Robot Operating System : ensemble de bibliothèques et d'outils pour faire communiquer les programmes d'un robot (topics, services, actions) |
| **Topic** | Canal de messages publié par un programme et lu par d'autres (ex. `/joint_states`) |
| **Service / action** | Appel ponctuel avec réponse (service) ou avec progression (action, ex. se lever) |
| **URDF / xacro** | Format de description du robot (pièces, articulations, masses) ; xacro permet des macros et des paramètres |
| **TF** | Arbre des repères du robot (où est chaque pièce à chaque instant) |
| **ros2_control** | Cadre ROS 2 qui sépare les contrôleurs du matériel (réel ou simulé) |
| **DDS / CycloneDDS** | La couche réseau de ROS 2, et l'une de ses implémentations |
| **rosbridge / foxglove_bridge** | Passerelles qui exposent les topics ROS à un navigateur web |
| **rosbag** | Enregistrement des topics pour les rejouer et les analyser |
| **RL** | Apprentissage par renforcement : un réseau de neurones apprend une politique de contrôle par essais en simulation |
| **Sim2real** | Faire fonctionner sur le vrai robot une politique apprise en simulation |
| **Domain randomization** | Faire varier aléatoirement les paramètres de la simulation pour que la politique résiste aux écarts avec la réalité |
| **REP-103 / REP-105** | Conventions ROS pour les unités, les axes et les noms de repères |
