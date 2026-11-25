# `leg_server_bridge` — pont ROS → vrai robot

Fait bouger les 18 servos à partir de `/joint_states` (publié par
[`gait_node`](gait_node.md)) en passant par le `leg_server` de la Pi
([`Dyna_test/dyna.md`](../../../../Dyna_test/dyna.md) §2). **Provisoire** : il sera
remplacé par le plugin ros2_control `hexapod_hardware` (phase P2 de
[`tps_archi.md`](../../../../tps_archi.md)).

Code : [`hexapod_control/leg_server_bridge.py`](../hexapod_control/leg_server_bridge.py).

```
/joint_states ─► leg_server_bridge ──TCP 5555──► leg_server (Pi) ──► U2D2 ──► 18 × AX-12A
  (rad)          radians → positions    GOALSID 25 Hz
                 (sens, offsets)        STATE 5 Hz (sécurité)
```

---

## Lancer

```bash
ros2 launch hexapod_control hexapod.launch.py robot:=dry    # vérifie, N'ENVOIE RIEN
ros2 launch hexapod_control hexapod.launch.py robot:=true   # LE ROBOT BOUGE
```

ou seul (avec `gait_node` déjà lancé) :

```bash
ros2 run hexapod_control leg_server_bridge                                        # simulation
ros2 run hexapod_control leg_server_bridge --ros-args -p send:=true -p ramp_time:=8.0
```

**Avant** : `leg_server` tourne sur la Pi (`ssh hexapode "pgrep -a leg_server"`),
batterie chargée (≥ 11 V), robot posé au sol, quelqu'un à côté de l'alimentation.
La calibration (`Dyna_test/gui/leg_gui_config.json` → `legs.yaml`) doit être à jour.

---

## Déroulé

1. **Connexion** au `leg_server`, lecture de l'état : les 18 servos doivent répondre,
   aucun en alarme, température < `temp_max`. Sinon le nœud s'arrête.
2. Avec `send:=true` : **réglages** (vitesse `speed`, limite de couple max, compliance
   pente 8 / punch 100 sur fémurs et tibias), consigne = **position actuelle**, puis
   **couple ON** (aucun saut).
3. **Rampe** : pendant `ramp_time` s, la consigne passe de la position réelle à la pose
   demandée par la marche (toutes les pattes ensemble : le robot se lève).
4. **Suivi** : chaque `/joint_states` est converti et envoyé (`GOALSID`, 25 Hz).
   Toutes les 2 s : écart consigne / position, charge max, tension.

Conversion (comme `hexapod_model.joint_to_raw`) :
`raw = 512 + (±(angle − offset)) / (300/1024 °)`, signe `−` si le servo est monté
inversé (`inverted` dans `legs.yaml`).

---

## Sécurité : le GEL

Le pont **gèle** (n'envoie plus aucune consigne, **le couple reste actif** : le robot
n'est pas lâché) si :

| Cause | Seuil |
|---|---|
| consigne hors des butées EEPROM d'un servo | butées lues au démarrage |
| servo qui ne répond plus | |
| servo en alarme (tension, surchauffe, surcharge, hors plage) ou limite de couple à 0 | |
| température | ≥ `temp_max` (62 °C) |
| charge | > `max_load` (85 %) pendant plus de `load_time` (0,6 s) |
| liaison TCP perdue | |
| Ctrl+C | |

Après un gel : relancer le pont (il repart de la position réelle, avec la rampe).

**Couper le couple** (robot relâché, il s'affaisse) :

```bash
nc 192.168.137.10 5555      # puis taper : ESTOP
```

Sortir un servo du mode sécurité : `RESET ALL` dans la même connexion
(voir [`dyna.md`](../../../../Dyna_test/dyna.md) §2.5).

---

## Paramètres (`config/bridge.yaml`)

| Paramètre | Défaut | Rôle |
|---|---|---|
| `send` | `false` | `true` = envoie aux servos (fixé par `robot:=true` au lancement) |
| `host`, `port` | `192.168.137.10`, 5555 | `leg_server` |
| `rate` | 25 Hz | fréquence d'envoi (lecture d'état à `rate` / 5) |
| `speed` | 100 | vitesse des servos (1..1023) |
| `ramp_time` | 8 s | durée du lever au démarrage |
| `max_load`, `load_time` | 85 %, 0,6 s | gel en surcharge |
| `temp_max` | 62 °C | gel en surchauffe |
| `legs_yaml` | (vide) | autre `legs.yaml` |

---

## Points à connaître

- **Un seul maître** : le pont et `test_torque` / `leg_gui` partagent le `leg_server` ;
  ne pas faire bouger le robot depuis deux programmes en même temps.
- **Sens des servos** : si une patte part à l'envers, c'est son `inverted` dans la
  calibration. Test du 25/11/2026 : les coxas des pattes 1 et 2 sont inversées, celles
  de 3 à 6 non (vérifié en faisant tourner chaque coxa de +20° et en regardant le pied).
- **L'USB de l'U2D2 décroche** parfois (voir `test_torque.md`) : `leg_server` se
  reconnecte, mais un `STATE` raté pendant ce temps provoque un gel.
